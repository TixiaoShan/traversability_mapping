#include "utility.h"

class TraversabilityFilter{
private:

    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    // point cloud to range image
    image_transport::Publisher  pubImage;
    ros::Subscriber subCloud;
    pcl::PointCloud<PointType>::Ptr laserCloudIn;

    tf::TransformListener listener;
    tf::StampedTransform transform;

    ros::Publisher pubCloud;
    ros::Publisher pubLaserScan;

    cv_bridge::CvImagePtr rangeImageFilter;

    int pubCount;

    pcl::PointCloud<PointType>::Ptr laserCloudOut;
    pcl::PointCloud<PointType>::Ptr laserCloudMatrix[NUM_ROWS];
    pcl::PointCloud<PointType>::Ptr laserCloudMatrix2[NUM_ROWS];

    sensor_msgs::LaserScan laserScan;
    float ringGroundRange[8];

    vector< vector<float> > rangeMatrix;

public:
    TraversabilityFilter():
        nh("~"),
        it(nh),
        pubCount(0){

        subCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &TraversabilityFilter::cloudHandler, this);
        pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/filtered_pointcloud", 100);
        pubLaserScan = nh.advertise<sensor_msgs::LaserScan> ("/pointcloud_2_laserscan", 1);  

        pubImage = it.advertise("/image_out", 1);

        allocateMemory();

        pointcloud2laserscanInitialization();

    }

    void allocateMemory(){

        laserCloudIn.reset(new pcl::PointCloud<PointType>());
        laserCloudOut.reset(new pcl::PointCloud<PointType>());

        for (int i = 0; i < NUM_ROWS; ++i){
            laserCloudMatrix[i].reset(new pcl::PointCloud<PointType>());
            laserCloudMatrix2[i].reset(new pcl::PointCloud<PointType>());
        }

        rangeMatrix.resize(NUM_ROWS);
        for (int i = 0; i < NUM_ROWS; ++i)
            rangeMatrix[i].resize(Horizon_SCAN);
    }

    ~TraversabilityFilter(){}
    /*
        cloudHadler:
        Receive velodyne point cloud and convert it to range image
        */
    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

        

        // 1. Convert ros message to pcl point cloud
        laserCloudIn->clear();
        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*laserCloudIn,*laserCloudIn, indices);

        PointType thisPoint;
        

        // 2. Point cloud to range image (full resolution)
        cv::Mat rangeImage;
        rangeImage = cv::Mat::zeros(N_SCAN, Horizon_SCAN, cv_bridge::getCvType("mono16"));

        int cloudSize = laserCloudIn->points.size();

        float verticalAngle, horizonAngle, range;
        int rowIdn, columnIdn;

        for (int i = 0; i < cloudSize; ++i){

            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = -laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;

            // 2.1 Find the row and column index in the iamge for this point
            verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
            rowIdn = -(verticalAngle - 15.1) / ang_res_y;

            horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
            if (horizonAngle <= -90)
                columnIdn = -int(horizonAngle / ang_res_x) - 450; 
            else if (horizonAngle >= 0)
                columnIdn =  -int(horizonAngle / ang_res_x) + 1350;
            else
                columnIdn =  1350 - int(horizonAngle / ang_res_x);

            // 2.2 Normalize the range
            range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);

            range = (!std::isinf(range)) ? 
                    std::max(0.0f, std::min(1.0f, factor * (range + offset))) : 
                    0.0;

            rangeImage.at<ushort>(rowIdn, columnIdn) = static_cast<ushort>((range) * std::numeric_limits<ushort>::max());
        }

        // 3. Publish range image
        sensor_msgs::ImagePtr msgImage;
        msgImage = cv_bridge::CvImage(std_msgs::Header(), "mono16", rangeImage).toImageMsg();
        msgImage->header = laserCloudMsg->header;
        pubImage.publish(msgImage);
        imageHandler(msgImage);
        return;
    }

    /*
        imageHandler:
        Receive the range image from cloudHandler and process it
        */
    void imageHandler(const sensor_msgs::ImageConstPtr& msgImage){

        imageProcess(msgImage);

        // publishLaserScan();
        publishLaserScanFusion();
        
    }

    void imageProcess(const sensor_msgs::ImageConstPtr& msgImage){
        

        // 1. Create range image and get row column information
        rangeImageFilter = cv_bridge::toCvCopy(msgImage, msgImage->encoding);

        int cols = rangeImageFilter->image.cols;
        int rows = rangeImageFilter->image.rows;

        PointType point;
        
        // 3. Convert from range image to point cloud
        ushort range_img; float range;
        for (int rowCount = 0; rowCount < NUM_ROWS; ++rowCount){
            // 3.0 we save the bottom ring as 0 row in laserCloudMatrix and rangeMatrix
            int j = rows - rowCount - 1;
            for (int i = 0; i < cols; ++i){   
                // 3.1 Calculate range value
                range_img = rangeImageFilter->image.at<ushort>(j, i);
                range = rescaleRange(range_img); // Rescale range

                point.x = sin(pcl::deg2rad(ang_start_x - i * ang_res_x)) * range;
                point.y = -cos(pcl::deg2rad(ang_start_x - i * ang_res_x)) * range;
                point.z = sin(pcl::deg2rad(ang_start_y - j * ang_res_y)) * range;
                // 3.2 No valid range value, set this point is invalid point
                if (range_img == 0)
                    point.intensity = -1; // no range information, mark as invalid point
                else
                    point.intensity = 0;
                // 3.3 Set the ring above the horizon as obstacle (similar to Hokuyo scan)
                if (rowCount == 8 && range_img != 0)
                    point.intensity = 100;
                // 3.4 Save point and range information
                laserCloudMatrix[rowCount]->push_back(point);
                rangeMatrix[rowCount][i] = range;
            }
        }

        
        // 4. Transform point
        // Listen to the TF transform and prepare for point cloud transformation
        try{listener.lookupTransform("map","base_link", ros::Time(0), transform); } 
        catch (tf::TransformException ex){ /*ROS_ERROR("Transfrom Failure.");*/ return; }

        for (int i = 0; i < NUM_ROWS; ++i){
            laserCloudMatrix[i]->header.frame_id = "base_link";
            pcl_ros::transformPointCloud("map", *laserCloudMatrix[i], *laserCloudMatrix2[i], listener);
            pcl::PointCloud<PointType>::Ptr laserCloudTemp = laserCloudMatrix2[i];
            laserCloudMatrix2[i] = laserCloudMatrix[i];
            laserCloudMatrix[i] = laserCloudTemp;
        }
        // 5. Filter
        curbFilter();
        slopeFilter();
        // 6. Publish
        publishCloud();
    }



    void curbFilter(){
        // 1. Range filter
        float diff[Horizon_SCAN - 1];
        bool breakFlag;
        for (int i = 0; i < NUM_ROWS - 1; ++i){
            // calculate range difference
            for (int j = 0; j < Horizon_SCAN - 1; ++j)
                diff[j] = rangeMatrix[i][j] - rangeMatrix[i][j+1];

            for (int j = rangeCompareNeighborNum; j < Horizon_SCAN - rangeCompareNeighborNum; ++j){
                breakFlag = false;
                // point is two far away, skip comparison since it can be inaccurate
                if (rangeMatrix[i][j] > RANGE_LIMIT)
                    continue;
                // make sure all points have valid range info
                for (int k = -rangeCompareNeighborNum; k <= rangeCompareNeighborNum; ++k)
                    if (rangeMatrix[i][j+k] == min_range){
                        breakFlag = true;
                        break;
                    }
                if (breakFlag == true) continue;
                // range difference should be monotonically increasing or decresing
                for (int k = -rangeCompareNeighborNum; k < rangeCompareNeighborNum-1; ++k)
                    if (diff[j+k] * diff[j+k+1] <= 0){
                        breakFlag = true;
                        break;
                    }
                if (breakFlag == true) continue;
                // the range difference between the start and end point of neighbor points is smaller than a threashold, then continue
                if (abs(rangeMatrix[i][j-rangeCompareNeighborNum] - rangeMatrix[i][j+rangeCompareNeighborNum]) /rangeMatrix[i][j] < 0.03)
                    continue;
                // if "continue" is not used at this point, it is very likely to be an obstacle point
                laserCloudMatrix[i]->points[j].intensity = 100;
            }
        }
    }

    void slopeFilter(){
        // 2. Slope filter
        float diffX, diffY, diffZ, angle;
        for (int j = 0; j < Horizon_SCAN; ++j){
            for (int i = 0; i < NUM_ROWS - 1; ++i){
                // 2.1 point without range value cannot be compared
                if (laserCloudMatrix[i]->points[j].intensity == -1 || laserCloudMatrix[i+1]->points[j].intensity == -1){
                    laserCloudMatrix[i]->points[j].intensity == -1;
                    continue;
                }
                // 2.2 Point that has been verified by range filter
                if (laserCloudMatrix[i]->points[j].intensity == 100)
                    continue;
                // 2.3  Two range filters here
                //      if a point's range is larger than 8 or 9 th ring point's range
                //      if a point's range is larger than the upper point's range
                //      then this point is very likely on obstacle. i.e. a point under the car or on a pole
                if ( ((rangeMatrix[NUM_ROWS - 1][j] != min_range && rangeMatrix[i][j] > rangeMatrix[NUM_ROWS - 1][j]) // 9th ring (the first ring above horizon)
                    || (rangeMatrix[7][j] != min_range && rangeMatrix[i][j] > rangeMatrix[7][j])) // 8th ring (the first ring below horizon)
                    || (rangeMatrix[i][j] > rangeMatrix[i+1][j]) ){
                    laserCloudMatrix[i]->points[j].intensity = 100;
                    continue;
                }
                // 2.4 Calculate slope angle
                diffX = laserCloudMatrix[i+1]->points[j].x - laserCloudMatrix[i]->points[j].x;
                diffY = laserCloudMatrix[i+1]->points[j].y - laserCloudMatrix[i]->points[j].y;
                diffZ = laserCloudMatrix[i+1]->points[j].z - laserCloudMatrix[i]->points[j].z;

                angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;
                // 2.5 Slope angle is larger than threashold, mark as obstacle point
                if (angle < -ANGLE_LIMIT || angle > ANGLE_LIMIT){
                    laserCloudMatrix[i]->points[j].intensity = 100;
                    continue;
                }
            }
        }
    }

    void publishCloud(){
        // 1. Save point to laserCloud from laserCloudMatrix       
        for (int j = 1; j < Horizon_SCAN - 1; ++j){
            for (int i = 0; i < PUB_ROWS; ++i){
                // 1.1 point that has no range information or too far away is discarded
                if (laserCloudMatrix[i]->points[j].intensity == -1 ||
                    rangeMatrix[i][j] >= RANGE_LIMIT)
                    continue;
                // 1.2 filter single obstacle point
                if (laserCloudMatrix[i]->points[j].intensity == 100 &&
                    laserCloudMatrix[i]->points[j-1].intensity == 0 &&
                    laserCloudMatrix[i]->points[j+1].intensity == 0)
                    continue;

                laserCloudOut->push_back(laserCloudMatrix[i]->points[j]);

                if (laserCloudMatrix[i]->points[j].intensity == 100)
                    break;
            }
        }

        // 2. Publish message
        sensor_msgs::PointCloud2 laserCloudFull;
        pcl::toROSMsg(*laserCloudOut, laserCloudFull);

        laserCloudFull.header.stamp = ros::Time::now(); // the laser cloud time should be older since odometry is received after velodyne_points
        laserCloudFull.header.frame_id = "map";

        pubCloud.publish(laserCloudFull);
        // 3. Clear cloud
        laserCloudOut->clear();
        for (int i = 0; i < NUM_ROWS; ++i){
            laserCloudMatrix[i]->clear();
            laserCloudMatrix2[i]->clear();
        }
    }

    float rescaleRange(ushort range_img){
        float range = static_cast<float>(range_img) /
                      static_cast<float>(std::numeric_limits<ushort>::max());
        return (range - offset*factor) / factor;
    }

    void pointcloud2laserscanInitialization(){
        laserScan.header.frame_id = "base_link"; // assume laser has the same frame as the robot

        laserScan.angle_min = -M_PI;
        laserScan.angle_max =  M_PI;
        laserScan.angle_increment = ang_res_x / 180 * M_PI;
        laserScan.time_increment = 0;

        laserScan.scan_time = 0.1; // 10Hz
        laserScan.range_min = 0.4;
        laserScan.range_max = 80;//localMapLength / 2;

        int range_size = std::ceil((laserScan.angle_max - laserScan.angle_min) / laserScan.angle_increment);
        laserScan.ranges.assign(range_size, laserScan.range_max + 1.0);

        for (int i = 0; i < 8; ++i){
            ringGroundRange[i] =  sensorHeight / std::sin((15.1 - i * ang_res_y) / 180 * M_PI);
        }
    }

    void publishLaserScan(){
        // consider the ring above horizon as lidar scan
        if (NUM_ROWS < 9)
            return;
        // initliaze range values
        laserScan.header.stamp = ros::Time::now();
        std::fill(laserScan.ranges.begin(), laserScan.ranges.end(), laserScan.range_max + 1.0);
        // update range array
        for (int i = 0; i < Horizon_SCAN; ++i){
            if (rangeMatrix[8][i] == min_range || rangeMatrix[8][i] > laserScan.range_max)
                continue;
            laserScan.ranges[Horizon_SCAN - i - 1] = rangeMatrix[8][i];
        }
        pubLaserScan.publish(laserScan);
    }

    void publishLaserScanFusion(){
        if (NUM_ROWS < 9)
            return;
        // initliaze range values
        laserScan.header.stamp = ros::Time::now();
        std::fill(laserScan.ranges.begin(), laserScan.ranges.end(), laserScan.range_max + 1.0);
         // update range array
        for (int i = 0; i < Horizon_SCAN; ++i){
            if (rangeMatrix[8][i] == min_range || rangeMatrix[8][i] > laserScan.range_max)
                continue;
            laserScan.ranges[Horizon_SCAN - i - 1] = rangeMatrix[8][i];
            for (int row = 0; row < 8; ++row){
                if (rangeMatrix[row][i] <= 0.8 * ringGroundRange[row])
                    laserScan.ranges[Horizon_SCAN - i - 1] = std::min(laserScan.ranges[Horizon_SCAN - i - 1], 
                                                        float(rangeMatrix[row][i] * std::cos((15.1 - row * ang_res_y) / 180 * M_PI)));
            }
        }
        pubLaserScan.publish(laserScan);
    }
};




int main(int argc, char** argv){

    ros::init(argc, argv, "traversability_mapping");
    
    TraversabilityFilter TFilter;

    ROS_INFO("\033[1;32m---->\033[0m Traversability Point Cloud Filter Started.");

    ros::spin();
    return 0;
}





