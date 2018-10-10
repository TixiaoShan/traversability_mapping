#include "utility.h"

class TraversabilityFilter{
private:

    ros::NodeHandle nh;
    // point cloud to range image
    ros::Subscriber subCloud;

    pcl::PointCloud<PointType>::Ptr laserCloudIn;
    pcl::PointCloud<PointType>::Ptr laserCloudOut1;
    pcl::PointCloud<PointType>::Ptr laserCloudOut2;

    tf::TransformListener listener;
    tf::StampedTransform transform;

    ros::Publisher pubCloud;
    ros::Publisher pubLaserScan;

    sensor_msgs::LaserScan laserScan; // pointcloud to laser scan

    PointType robotPoint;

    bool _groundFillingFlag;

    float **minHeight;
    float **maxHeight;
    bool **initFlag;

public:
    TraversabilityFilter():
        nh("~"),
        _groundFillingFlag(false){

        subCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &TraversabilityFilter::cloudHandler, this);

        pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/filtered_pointcloud", 1);
        pubLaserScan = nh.advertise<sensor_msgs::LaserScan> ("/pointcloud_2_laserscan", 1);  

        allocateMemory();

        pointcloud2laserscanInitialization();
    }

    void allocateMemory(){

        laserCloudIn.reset(new pcl::PointCloud<PointType>());
        laserCloudOut1.reset(new pcl::PointCloud<PointType>());
        laserCloudOut2.reset(new pcl::PointCloud<PointType>());

        
        initFlag = new bool*[_filterHeightMapArrayLength];
        for (int i = 0; i < _filterHeightMapArrayLength; ++i)
            initFlag[i] = new bool[_filterHeightMapArrayLength];

        minHeight = new float*[_filterHeightMapArrayLength];
        for (int i = 0; i < _filterHeightMapArrayLength; ++i)
            minHeight[i] = new float[_filterHeightMapArrayLength];

        maxHeight = new float*[_filterHeightMapArrayLength];
        for (int i = 0; i < _filterHeightMapArrayLength; ++i)
            maxHeight[i] = new float[_filterHeightMapArrayLength];

        resetParams();
    }

    void resetParams(){

        laserCloudIn->clear();
        laserCloudOut1->clear();
        laserCloudOut2->clear();

        for (int i = 0; i < _filterHeightMapArrayLength; ++i)
            for (int j = 0; j < _filterHeightMapArrayLength; ++j)
                initFlag[i][j] = false;
    }


    ~TraversabilityFilter(){}

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

        if (getTransformation() == false) return;

        extractPointCloud(laserCloudMsg);

        fillGround();

        transformCloud();

        buildHeightMap();

        publishCloud();

        publishLaserScan();

        resetParams();
    }

    bool getTransformation(){
        // Listen to the TF transform and prepare for point cloud transformation
        try{listener.lookupTransform("map","base_link", ros::Time(0), transform); }
        catch (tf::TransformException ex){ /*ROS_ERROR("Transfrom Failure.");*/ return false; }

        robotPoint.x = transform.getOrigin().x();
        robotPoint.y = transform.getOrigin().y();
        robotPoint.z = transform.getOrigin().z();

        return true;
    }

    void extractPointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

        pcl::PointCloud<PointType> laserCloudTemp;
        pcl::fromROSMsg(*laserCloudMsg, laserCloudTemp);

        // extract point cloud
        int cloudSize = laserCloudTemp.points.size();
        for (int i = 0; i < cloudSize; ++i){

            // float verticalAngle = atan2(laserCloudTemp.points[i].z,
            //                             sqrt(laserCloudTemp.points[i].x * laserCloudTemp.points[i].x + laserCloudTemp.points[i].y * laserCloudTemp.points[i].y)) * 180 / M_PI;
            // int rowIdn = (verticalAngle + (-15.1)) / 2.0;

            // if (rowIdn <= 10)

            if (laserCloudTemp.points[i].z <= 0.5)
            
                laserCloudIn->push_back(laserCloudTemp.points[i]);
        }
    }

    void fillGround(){
        // fill the ground below the robot when start mapping
        if (_groundFillingFlag == true) return;

        _groundFillingFlag = true;

        float fillingRadius = _sensorHeight / std::tan(15.0 / 180.0 * M_PI);

        for (float radius = 0; radius <= fillingRadius; radius += 0.1) {
            for (float theta = -M_PI; theta <= M_PI; theta += M_PI/36){
                PointType thisPoint;
                thisPoint.x = radius * std::cos(theta);
                thisPoint.y = radius * std::sin(theta);
                thisPoint.z = -_sensorHeight;
                laserCloudIn->push_back(thisPoint);
            }
        }
    }

    void transformCloud(){

        laserCloudIn->header.frame_id = "base_link";
        laserCloudIn->header.stamp = 0; // don't use the latest time, we don't have that transform in the queue yet

        pcl::PointCloud<PointType> laserCloudTemp;
        pcl_ros::transformPointCloud("map", *laserCloudIn, laserCloudTemp, listener);
        *laserCloudIn = laserCloudTemp;
    }

    void buildHeightMap(){

    	float roundedX = float(int(robotPoint.x * 10.0f)) / 10.0f;
    	float roundedY = float(int(robotPoint.y * 10.0f)) / 10.0f;

        // height map origin
        PointType filterHeightMapOriginPoint;
        filterHeightMapOriginPoint.x = roundedX - _rangeLimit;
        filterHeightMapOriginPoint.y = roundedY - _rangeLimit;

        // convert from point cloud to height map
        int cloudSize = laserCloudIn->points.size();
        for (int i = 0; i < cloudSize; ++i){

            int idx = (laserCloudIn->points[i].x - filterHeightMapOriginPoint.x) / mapResolution;
            int idy = (laserCloudIn->points[i].y - filterHeightMapOriginPoint.y) / mapResolution;
            // points out of boundry
            if (idx < 0 || idy < 0 || idx >= _filterHeightMapArrayLength || idy >= _filterHeightMapArrayLength)
                continue;

            float diffX = laserCloudIn->points[i].x - robotPoint.x;
            float diffY = laserCloudIn->points[i].y - robotPoint.y;

            if (std::sqrt(diffX*diffX + diffY*diffY) > _rangeLimit)
                continue;

            if (initFlag[idx][idy] == false){
                minHeight[idx][idy] = laserCloudIn->points[i].z;
                maxHeight[idx][idy] = laserCloudIn->points[i].z;
                initFlag[idx][idy] = true;
            } else {
                minHeight[idx][idy] = std::min(minHeight[idx][idy], laserCloudIn->points[i].z);
                maxHeight[idx][idy] = std::max(maxHeight[idx][idy], laserCloudIn->points[i].z);
            }
        }

        // convert from height map to point cloud
        for (int i = 0; i < _filterHeightMapArrayLength; ++i){
            for (int j = 0; j < _filterHeightMapArrayLength; ++j){
                // no point at this grid
                if (initFlag[i][j] == false)
                    continue;
                // conver grid to point
                PointType thisPoint;
                thisPoint.x = filterHeightMapOriginPoint.x + i * mapResolution + mapResolution / 2.0;
                thisPoint.y = filterHeightMapOriginPoint.y + j * mapResolution + mapResolution / 2.0;
                thisPoint.z = maxHeight[i][j];

                if (maxHeight[i][j] - minHeight[i][j] > 0.05){
                    thisPoint.intensity = 1; // obstacle
                    laserCloudOut1->push_back(thisPoint);
                }else{
                    thisPoint.intensity = -1; // free
                    laserCloudOut2->push_back(thisPoint);
                }
            }
        }
    }

    void publishCloud(){
        sensor_msgs::PointCloud2 laserCloudTemp;
        pcl::toROSMsg(*laserCloudOut1 + *laserCloudOut2, laserCloudTemp);
        laserCloudTemp.header.stamp = ros::Time::now();
        laserCloudTemp.header.frame_id = "map";
        pubCloud.publish(laserCloudTemp);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////// Point Cloud to Laser Scan  ///////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void pointcloud2laserscanInitialization(){

        laserScan.header.frame_id = "base_link"; // assume laser has the same frame as the robot

        laserScan.angle_min = -M_PI;
        laserScan.angle_max =  M_PI;
        laserScan.angle_increment = 1.0f / 180 * M_PI;
        laserScan.time_increment = 0;

        laserScan.scan_time = 0.1;
        laserScan.range_min = 0.3;
        laserScan.range_max = 100;

        int range_size = std::ceil((laserScan.angle_max - laserScan.angle_min) / laserScan.angle_increment);
        laserScan.ranges.assign(range_size, laserScan.range_max + 1.0);
    }

    void updateLaserScan(){

        try{listener.lookupTransform("base_link","map", ros::Time(0), transform);}
        catch (tf::TransformException ex){ /*ROS_ERROR("Transfrom Failure.");*/ return; }

        laserCloudOut1->header.frame_id = "map";
        laserCloudOut1->header.stamp = 0;

        pcl::PointCloud<PointType> laserCloudTemp;
        pcl_ros::transformPointCloud("base_link", *laserCloudOut1, laserCloudTemp, listener);

        int cloudSize = laserCloudTemp.points.size();
        for (int i = 0; i < cloudSize; ++i){
            PointType *point = &laserCloudTemp.points[i];
            float x = point->x;
            float y = point->y;
            float range = std::sqrt(x*x + y*y);
            float angle = std::atan2(y, x);
            int index = (angle - laserScan.angle_min) / laserScan.angle_increment;
            laserScan.ranges[index] = std::min(laserScan.ranges[index], range);
        } 
    }

    void publishLaserScan(){

        updateLaserScan();

        laserScan.header.stamp = ros::Time::now();
        pubLaserScan.publish(laserScan);
        // initialize laser scan for new scan
        std::fill(laserScan.ranges.begin(), laserScan.ranges.end(), laserScan.range_max + 1.0);
    }
};




int main(int argc, char** argv){

    ros::init(argc, argv, "traversability_mapping");
    
    TraversabilityFilter TFilter;

    ROS_INFO("\033[1;32m---->\033[0m Traversability Filter Started.");

    ros::spin();

    return 0;
}