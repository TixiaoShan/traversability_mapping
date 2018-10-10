#include "utility.h"



class TraversabilityCloud{

private:

    ros::NodeHandle nh;

    ros::Subscriber subLaserCloud;
    ros::Publisher pubFullCloud;

    static const int laserCloudArrayNum = laserArrayCloudWidth * laserArrayCloudDepth * laserArrayCloudHeight;

    static const int rootCubeIndexX = laserArrayCloudWidth / 2;
    static const int rootCubeIndexY = laserArrayCloudDepth / 2;
    static const int rootCubeIndexZ = laserArrayCloudHeight / 2;

    bool validArray[laserCloudArrayNum];
    bool arrayUpdated[laserCloudArrayNum];

    pcl::PointCloud<PointType>::Ptr laserCloudArray[laserCloudArrayNum];
    pcl::PointCloud<PointType>::Ptr laserCloudArray2[laserCloudArrayNum];

    pcl::VoxelGrid<PointType> downSizeFilter;

    pcl::PointCloud<PointType>::Ptr laserCloudAllPoints;
    pcl::PointCloud<PointType>::Ptr laserCloudIn;

    int pubCount;
    int receiveCount;

    int cubeX, cubeY, cubeZ, cubeInd;

    double lastPubTime;

public:

    TraversabilityCloud():
    nh("~"),
    pubCount(0),
    receiveCount(0),
    lastPubTime(0){

        if (recordFullPointCloud == false)
            return;

        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_registered", 1, &TraversabilityCloud::laserCloudHandler, this);

        pubFullCloud = nh.advertise<sensor_msgs::PointCloud2> ("/full_pointcloud", 1);

        downSizeFilter.setLeafSize(fullCloudResolution, fullCloudResolution, fullCloudResolution);

        allocateMemory();
    }

    void allocateMemory(){

        laserCloudAllPoints.reset(new pcl::PointCloud<PointType>());
        laserCloudIn.reset(new pcl::PointCloud<PointType>());

        for (int i = 0; i < laserCloudArrayNum; i++){
            validArray[i] = false;
            arrayUpdated[i] = false;
            laserCloudArray[i].reset(new pcl::PointCloud<PointType>());
            laserCloudArray2[i].reset(new pcl::PointCloud<PointType>());

        }
    }


    void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
        // 1. skip some frame
        // if (receiveCount < laserReceiveSkipNum){
        //     ++receiveCount;
        //     return;
        // }
        // receiveCount = 0;

        // 2. conversion
        laserCloudIn->clear();
        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);

        PointType thisPoint;

        /**
        Calculate which cude this point belongs to
        */

        for (int i = 0; i < laserCloudIn->points.size(); ++i){
            // LOAM z - x - y
            thisPoint.x = laserCloudIn->points[i].z;
            thisPoint.y = laserCloudIn->points[i].x;
            thisPoint.z = laserCloudIn->points[i].y;

            savePoint(thisPoint);
        }
        /*
        Downsize cloud array
        */
        for (int ind = 0; ind < laserCloudArrayNum; ++ind){
            if (validArray[ind] && arrayUpdated[ind]){
                laserCloudArray[ind]->clear();
                downSizeFilter.setInputCloud(laserCloudArray2[ind]);
                downSizeFilter.filter(*laserCloudArray[ind]);
                // update laserCloudArray2 for next new point cloud input
                pcl::PointCloud<PointType>::Ptr laserCloudTemp = laserCloudArray[ind];
                laserCloudArray[ind] = laserCloudArray2[ind];
                laserCloudArray2[ind] = laserCloudTemp;
                // reset arrayUpdated
                arrayUpdated[ind] = false;
            }
        }

        publishFullCloud();      
    }

    void savePoint(PointType &point){

        cubeX = int((point.x + laserCloudCubeWidth/2) / laserCloudCubeWidth) + rootCubeIndexX;
        cubeY = int((point.y + laserCloudCubeDepth/2) / laserCloudCubeDepth) + rootCubeIndexY;
        cubeZ = int((point.z + laserCloudCubeHeight/2) / laserCloudCubeHeight) + rootCubeIndexZ;

        if (point.x + laserCloudCubeWidth/2 < 0)  cubeX--;
        if (point.y + laserCloudCubeDepth/2 < 0)  cubeY--;
        if (point.z + laserCloudCubeHeight/2 < 0) cubeZ--;

        if (cubeX >= 0 && cubeX < laserArrayCloudWidth && 
            cubeY >= 0 && cubeY < laserArrayCloudDepth && 
            cubeZ >= 0 && cubeZ < laserArrayCloudHeight){

            cubeInd = cubeX + laserArrayCloudWidth * cubeY + laserArrayCloudWidth * laserArrayCloudDepth * cubeZ;

            laserCloudArray2[cubeInd]->push_back(point);

            validArray[cubeInd] = true;
            arrayUpdated[cubeInd] = true;
        }
    }

    void publishFullCloud(){
        // set publish full cloud interval
        if (ros::Time::now().toSec() - lastPubTime < 3)
            return;
        else
            lastPubTime = ros::Time::now().toSec();
        // no subscribers, return
        if (pubFullCloud.getNumSubscribers() == 0)
            return;

        /*
        Extract all points from array to publish
        */
        laserCloudAllPoints->clear();

        for (int ind = 0; ind < laserCloudArrayNum; ++ind)
            if (validArray[ind]){
                *laserCloudAllPoints += *laserCloudArray2[ind];
            }
        /*
        Publish all points
        */
        sensor_msgs::PointCloud2 laserCloudFullResTemp;
        pcl::toROSMsg(*laserCloudAllPoints, laserCloudFullResTemp);
        laserCloudFullResTemp.header.stamp = ros::Time::now();
        laserCloudFullResTemp.header.frame_id = "map";
        pubFullCloud.publish(laserCloudFullResTemp);
    }

};






int main(int argc, char** argv){

    ros::init(argc, argv, "traversability_mapping");
    
    TraversabilityCloud TC;
   
    ROS_INFO("\033[1;32m---->\033[0m Traversability Record Full Cloud %s.",
                recordFullPointCloud == true ? "\033[1;31mEnabled\033[0m" : "\033[1;31mDisabled\033[0m");

    ros::spin();

    return 0;
}



#include <pcl/octree/octree.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/octree/octree_impl.h>
pcl::octree::OctreePointCloudVoxelCentroid<PointType>::Ptr laserCloudFullOctree(new pcl::octree::OctreePointCloudVoxelCentroid<PointType>(fullCloudResolution));
pcl::PointCloud<PointType>::Ptr laserCloudPubPointsVoxel(new pcl::PointCloud<PointType>());
laserCloudFullOctree.reset(new pcl::octree::OctreePointCloudVoxelCentroid<PointType>(fullCloudResolution));
laserCloudFullOctree->setInputCloud(laserCloudAllPoints);
laserCloudFullOctree->addPointsFromInputCloud();
std::vector<PointType, Eigen::aligned_allocator<PointType> > voxelList;
laserCloudFullOctree->getOccupiedVoxelCenters(voxelList);
// save octree to point cloud
laserCloudPubPointsVoxel->clear();
for(int voxInd = 0; voxInd < voxelList.size(); voxInd++){
    laserCloudPubPointsVoxel->points.push_back(voxelList[voxInd]);
}