#include "utility.h"

class TraversabilityMapping{

private:

    // ROS Node Handle
    ros::NodeHandle nh;

    // Mutex Memory Lock
    std::mutex mtx;

    // Transform Listener (get robot position)
    tf::TransformListener listener;
    tf::StampedTransform transform;

    // Subscriber
    ros::Subscriber subFilteredGroundCloud;

    // Publisher
    ros::Publisher pubOccupancyMapLocal;
    ros::Publisher pubOccupancyMapLocalHeight;
    ros::Publisher pubElevationCloud;

    // Point Cloud Pointer
    pcl::PointCloud<PointType>::Ptr laserCloud; // save input filtered laser cloud for mapping
    pcl::PointCloud<PointType>::Ptr laserCloudElevation; // a cloud for publishing elevation map

    // Occupancy Grid Map
    nav_msgs::OccupancyGrid occupancyMap2D; // local occupancy grid map
    elevation_msgs::OccupancyElevation occupancyMap2DHeight; // customized message that includes occupancy map and elevation info

    int pubCount;
    pcl::VoxelGrid<PointType> downSizeFilter;
    
    // Map Arrays
    int mapArrayCount;
    int **mapArrayInd; // it saves the index of this submap in vector mapArray
    int **predictionArrayFlag;
    vector<childMap_t*> mapArray;

    // Local Map Extraction
    PointType robotPoint;
    PointType localMapOriginPoint;
    grid_t localMapOriginGrid;

    // Global Variables for Traversability Calculation
    cv::Mat matCov, matEig, matVec;

    // Lists for New Scan
    vector<mapCell_t*> newScanList;
    vector<mapCell_t*> newScanList2; // for passing to prediction list: observingList
    vector<mapCell_t*> observingList; // all grids that are observed in each new scan (updated every scan)

public:
    TraversabilityMapping():
        nh("~"),
        pubCount(10),
        mapArrayCount(0){
        // subscribe to traversability filter
        subFilteredGroundCloud = nh.subscribe<sensor_msgs::PointCloud2>("/filtered_pointcloud", 1, &TraversabilityMapping::cloudHandler, this);
        // publish local occupancy and elevation grid map
        pubOccupancyMapLocal = nh.advertise<nav_msgs::OccupancyGrid> ("/occupancy_map_local", 1);
        pubOccupancyMapLocalHeight = nh.advertise<elevation_msgs::OccupancyElevation> ("/occupancy_map_local_height", 1);
        // publish elevation map for visualization
        pubElevationCloud = nh.advertise<sensor_msgs::PointCloud2> ("/elevation_pointcloud", 1);

        allocateMemory(); 
    }

    ~TraversabilityMapping(){}

    

    void allocateMemory(){
        // allocate memory for point cloud
        laserCloud.reset(new pcl::PointCloud<PointType>());
        laserCloudElevation.reset(new pcl::PointCloud<PointType>());
        
        // initialize array for cmap
        mapArrayInd = new int*[mapArrayLength];
        for (int i = 0; i < mapArrayLength; ++i)
            mapArrayInd[i] = new int[mapArrayLength];

        for (int i = 0; i < mapArrayLength; ++i)
            for (int j = 0; j < mapArrayLength; ++j)
                mapArrayInd[i][j] = -1;

        // initialize array for predicting elevation sub-maps
        predictionArrayFlag = new int*[mapArrayLength];
        for (int i = 0; i < mapArrayLength; ++i)
            predictionArrayFlag[i] = new int[mapArrayLength];

        for (int i = 0; i < mapArrayLength; ++i)
            for (int j = 0; j < mapArrayLength; ++j)
                predictionArrayFlag[i][j] = false;

        // Matrix Initialization
        matCov = cv::Mat (3, 3, CV_32F, cv::Scalar::all(0));
        matEig = cv::Mat (1, 3, CV_32F, cv::Scalar::all(0));
        matVec = cv::Mat (3, 3, CV_32F, cv::Scalar::all(0));

        downSizeFilter.setLeafSize(_visualizationDownSampleResolution, _visualizationDownSampleResolution, _visualizationDownSampleResolution);

        initializeLocalOccupancyMap();
    }

    void resetParams(){
        newScanList2 = newScanList;
        newScanList.clear();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////// Mapping /////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

        // Convert Point Cloud
        pcl::fromROSMsg(*laserCloudMsg, *laserCloud);

        std::lock_guard<std::mutex> lock(mtx);

        // Register New Scan
        updateElevationMap();

        // publish local occupancy grid map
        publishMap();

        // Reset for New Scan
        resetParams();
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////// Register Cloud /////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void updateElevationMap(){
        int cloudSize = laserCloud->points.size();
        for (int i = 0; i < cloudSize; ++i){
            laserCloud->points[i].z -= 0.2; // for visualization
            updateElevationMap(&laserCloud->points[i]);
        }
    }

    bool updateElevationMap(PointType *point){
        // Find point index in global map
        grid_t thisGrid;
        if (findPointGridInMap(&thisGrid, point) == false) return false;
        // Get current cell pointer
        mapCell_t *thisCell = grid2Cell(&thisGrid);
        // Save new scan (for passing to prediction thread)
        newScanList.push_back(thisCell);
        // register elevation
        thisCell->updateElevationAlphaBeta(point->z);
        // register traversability
        thisCell->updateTraversabilityAlphaBeta(point->intensity);
 
        thisCell->observedNonTraversableFlag = (point->intensity == 1) ? true : false;
        
        return true;
    }

    mapCell_t* grid2Cell(grid_t *thisGrid){
        return mapArray[mapArrayInd[thisGrid->cubeX][thisGrid->cubeY]]->cellArray[thisGrid->gridX][thisGrid->gridY];
    }

    bool findPointGridInMap(grid_t *gridOut, PointType *point){
        // Calculate the cube index that this point belongs to. (Array dimension: mapArrayLength * mapArrayLength)
        grid_t thisGrid;
        getPointCubeIndex(&thisGrid.cubeX, &thisGrid.cubeY, point);
        // Decide whether a point is out of pre-allocated map
        if (thisGrid.cubeX >= 0 && thisGrid.cubeX < mapArrayLength && 
            thisGrid.cubeY >= 0 && thisGrid.cubeY < mapArrayLength){
            // Point is in the boundary, but this sub-map is not allocated before
            // Allocate new memory for this sub-map and save it to mapArray
            if (mapArrayInd[thisGrid.cubeX][thisGrid.cubeY] == -1){
                childMap_t *thisChildMap = new childMap_t(mapArrayCount, thisGrid.cubeX, thisGrid.cubeY);
                mapArray.push_back(thisChildMap);
                mapArrayInd[thisGrid.cubeX][thisGrid.cubeY] = mapArrayCount;
                ++mapArrayCount;
            }
        }else{
            // Point is out of pre-allocated boundary, report error (you should increase map size)
            ROS_ERROR("Point cloud is out of elevation map boundary. Change params ->mapArrayLength<-. The program will crash!");
            return false;
        }
        // sub-map id
        thisGrid.mapID = mapArrayInd[thisGrid.cubeX][thisGrid.cubeY];
        // Find the index for this point in this sub-map (grid index)
        thisGrid.gridX = (int)((point->x - mapArray[thisGrid.mapID]->originX) / mapResolution);
        thisGrid.gridY = (int)((point->y - mapArray[thisGrid.mapID]->originY) / mapResolution);
        if (thisGrid.gridX < 0 || thisGrid.gridY < 0 || thisGrid.gridX >= mapCubeArrayLength || thisGrid.gridY >= mapCubeArrayLength)
            return false;

        *gridOut = thisGrid;
        return true;
    }

    void getPointCubeIndex(int *cubeX, int *cubeY, PointType *point){
        *cubeX = int((point->x + mapCubeLength/2.0) / mapCubeLength) + rootCubeIndex;
        *cubeY = int((point->y + mapCubeLength/2.0) / mapCubeLength) + rootCubeIndex;

        if (point->x + mapCubeLength/2.0 < 0)  --*cubeX;
        if (point->y + mapCubeLength/2.0 < 0)  --*cubeY;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////// Predict Map /////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void BGK(){

        ros::Rate rate(3); // Hz
        
        while (ros::ok()){

            elevationMapPrediction(); // Predict Elevation Map

            traversabilityMapCalculation();

            traversabilityMapPrediction();

            resetBGKParams();

            rate.sleep();
        }
    }

    void resetBGKParams(){

        if (_makeElevationPredictionFlag == false)
            return;

        // reset observation list
        int listSize = observingList.size();
        for (int i = 0; i < listSize; ++i){
            observingList[i]->observingFlag = false;
            observingList[i]->observedNonTraversableFlag = false;
        }
        observingList.clear();

        // clear elevetion prediction related data
        for (int i = 0; i < mapArrayLength; ++i){
            for (int j = 0; j < mapArrayLength; ++j){
                predictionArrayFlag[i][j] = false;
                if (mapArrayInd[i][j] != -1)
                    mapArray[mapArrayInd[i][j]]->clearInferenceData();
            }
        }
    }

    void elevationMapPrediction(){

        if (_makeElevationPredictionFlag == false)
            return;

        observingList = newScanList2;

        // find the elevation prediction array
        int listSize = observingList.size();
        for (int i = 0; i < listSize; ++i){
            // Mark new scan grids (they are used as new training data)
            observingList[i]->observingFlag = true;

            grid_t thisGrid = observingList[i]->grid;
            predictionArrayFlag[thisGrid.cubeX][thisGrid.cubeY] = true;
            predictionArrayFlag[thisGrid.cubeX-1][thisGrid.cubeY] = true;
            predictionArrayFlag[thisGrid.cubeX+1][thisGrid.cubeY] = true;
            predictionArrayFlag[thisGrid.cubeX][thisGrid.cubeY-1] = true;
            predictionArrayFlag[thisGrid.cubeX][thisGrid.cubeY+1] = true;
        }

        for (int i = 0; i < mapArrayLength; ++i){
            for (int j = 0; j < mapArrayLength; ++j){
                // no need to predict
                if (predictionArrayFlag[i][j] == false)
                    continue;

                // Sub-map (batch) not existing, create new one
                if (mapArrayInd[i][j] == -1){
                    childMap_t *thisChildMap = new childMap_t(mapArrayCount, i, j);
                    mapArray.push_back(thisChildMap);
                    mapArrayInd[i][j] = mapArrayCount;
                    ++mapArrayCount;
                }

                // Training data
                vector<float> xTrainVec;
                vector<float> yTrainVec;

                for (int k = -1; k <= 1; ++k){
                    for (int l = -1; l <= 1; ++l){
                        // skip four corners
                        if (std::abs(k) + std::abs(l) == 2)
                            continue;
                        // Sub-map (batch) not existing
                        if (i+k < 0 || i+k >= mapArrayLength || j+l < 0 || j+l >= mapArrayLength) continue;
                        int mapInd = mapArrayInd[i+k][j+l];
                        if (mapInd == -1) continue;
                        // Find training data for this batch, and append
                        mapArray[mapInd]->findTrainingData("elevation");
                        xTrainVec.insert(std::end(xTrainVec), std::begin(mapArray[mapInd]->xTrain), std::end(mapArray[mapInd]->xTrain));
                        yTrainVec.insert(std::end(yTrainVec), std::begin(mapArray[mapInd]->yTrain), std::end(mapArray[mapInd]->yTrain));
                    }
                }

                if (xTrainVec.size() == 0 || yTrainVec.size() == 0)
                    continue;

                Eigen::MatrixXf xTrain = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(xTrainVec.data(), xTrainVec.size() / 2, 2);
                Eigen::MatrixXf yTrain = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(yTrainVec.data(), yTrainVec.size(), 1);

                // Testing data
                int mapInd = mapArrayInd[i][j];
                mapArray[mapInd]->findTestingData(); // Find testing data for this batch

                vector<float> xTestVec = mapArray[mapInd]->xTest; // testing data vector
                vector<mapCell_t*> testList = mapArray[mapInd]->testList; // testing data list

                if (xTestVec.size() == 0)
                    continue;

                Eigen::MatrixXf xTest = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(xTestVec.data(), xTestVec.size() / 2, 2);

                // Predict
                Eigen::MatrixXf Ks; // covariance matrix
                covSparse(xTest, xTrain, Ks); // sparse kernel

                Eigen::MatrixXf ybar = (Ks * yTrain).array();
                Eigen::MatrixXf kbar = Ks.rowwise().sum().array();

                mapArray[mapInd]->Ks = Ks;
                mapArray[mapInd]->kbar = kbar;

                // Update Elevation with Prediction
                for (int k = 0; k < testList.size(); ++k){

                    if (std::isnan(ybar(k,0)) || std::isnan(kbar(k,0)))
                        continue;

                    if (ybar(k,0) == 0 && kbar(k,0) == 0) // too far away to training points, all zero
                        continue;

                    mapCell_t* thisCell = testList[k];

                    float alphaElev = thisCell->alphaElev;
                    float betaElev = thisCell->betaElev;
                    alphaElev += ybar(k,0);
                    betaElev  += kbar(k,0) - ybar(k,0);
                    float elevation = alphaElev / (alphaElev + betaElev);

                    if (!std::isnan(elevation))
                        thisCell->updateElevationAlphaBeta(alphaElev, betaElev, elevation);
                }
            }
        }
    }



    void dist(const Eigen::MatrixXf &xStar, const Eigen::MatrixXf &xTrain, Eigen::MatrixXf &d) const {
        d = Eigen::MatrixXf::Zero(xStar.rows(), xTrain.rows());
        for (int i = 0; i < xStar.rows(); ++i) {
            d.row(i) = (xTrain.rowwise() - xStar.row(i)).rowwise().norm();
        }
    }

    void covSparse(const Eigen::MatrixXf &xStar, const Eigen::MatrixXf &xTrain, Eigen::MatrixXf &Kxz) const {
        dist(xStar/(_predictionKernalSize+0.1), xTrain/(_predictionKernalSize+0.1), Kxz);
        Kxz = (((2.0f + (Kxz * 2.0f * 3.1415926f).array().cos()) * (1.0f - Kxz.array()) / 3.0f) +
              (Kxz * 2.0f * 3.1415926f).array().sin() / (2.0f * 3.1415926f)).matrix() * 1.0f;
        // Clean up for values with distance outside length scale, possible because Kxz <= 0 when dist >= _predictionKernalSize
        for (int i = 0; i < Kxz.rows(); ++i)
            for (int j = 0; j < Kxz.cols(); ++j)
                if (Kxz(i,j) < 0) Kxz(i,j) = 0;
    }

    void covMaterniso3(const Eigen::MatrixXf &xStar, const Eigen::MatrixXf &xTrain, Eigen::MatrixXf &Kxz) const {
        dist(1.73205 / (_predictionKernalSize+0.1) * xStar, 1.73205 / (_predictionKernalSize+0.1) * xTrain, Kxz);
        Kxz = ((1 + Kxz.array()) * exp(-Kxz.array())).matrix() * 1.0f;
    }


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////// Traversability Calculation ///////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void traversabilityMapCalculation(){

        if (_makeElevationPredictionFlag == false)
            return;

        // no new scan, return
        if (observingList.size() == 0)
            return;

        int listSize = observingList.size();

        for (int i = 0; i < listSize; ++i){

            mapCell_t *thisCell = observingList[i];

            // if grid non-traversable decided by traversability_filter
            if (thisCell->observedNonTraversableFlag == true){
                thisCell->updateTraversabilityAlphaBeta(1);
                continue;
            }
            
            vector<float> xyzVector = findNeighborElevations(thisCell);

            if (xyzVector.size() <= 2)
                continue;

            Eigen::MatrixXf matPoints = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(xyzVector.data(), xyzVector.size() / 3, 3);

            // min and max elevation
            float minElevation = matPoints.col(2).minCoeff();
            float maxElevation = matPoints.col(2).maxCoeff();
            float maxDifference = (maxElevation - minElevation) < 0.05 ? 0 : maxElevation - minElevation;

            // if (maxElevation - minElevation > _stepHeightThreshold){
            //     thisCell->updateTraversabilityAlphaBeta(1);
            //     continue;
            // }

            // find maximum step
            // float maximumStep = -FLT_MAX;
            // for (int j = 0; j < matPoints.rows(); ++j){
            //     for (int k = j+1; k < matPoints.rows(); ++k){
            //         float thisStepDiff = std::abs(matPoints(j,2) - matPoints(k,2));
            //         maximumStep = std::max(thisStepDiff, maximumStep);
            //     }
            // }

            Eigen::MatrixXf centered = matPoints.rowwise() - matPoints.colwise().mean();
            Eigen::MatrixXf cov = (centered.adjoint() * centered);
            cv::eigen2cv(cov, matCov); // copy data from eigen to cv::Mat
            cv::eigen(matCov, matEig, matVec); // find eigenvalues and eigenvectors for the covariance matrix

            float slopeAngle = std::acos(std::abs(matVec.at<float>(2, 2))) / M_PI * 180;
            // float occupancy = 1.0f / (1.0f + exp(-(slopeAngle - _slopeAngleThreshold)));

            float occupancy = 0.5 * (slopeAngle / _slopeAngleThreshold)
                            + 0.5 * (maxDifference / _stepHeightThreshold);

            if (slopeAngle > _slopeAngleThreshold || maxDifference > _stepHeightThreshold)
                occupancy = 1;

            thisCell->updateTraversabilityAlphaBeta(occupancy);
        }
    }

    vector<float> findNeighborElevations(mapCell_t *centerCell){

        vector<float> xyzVector;

        grid_t centerGrid = centerCell->grid;
        grid_t thisGrid;

        for (int k = -footprintRadiusLength; k <= footprintRadiusLength; ++k){
            for (int l = -footprintRadiusLength; l <= footprintRadiusLength; ++l){
                // skip grids too far
                if (std::sqrt(float(k*k + l*l)) * mapResolution > robotRadius)
                    continue;
                // the neighbor grid
                thisGrid.cubeX = centerGrid.cubeX;
                thisGrid.cubeY = centerGrid.cubeY;
                thisGrid.gridX = centerGrid.gridX + k;
                thisGrid.gridY = centerGrid.gridY + l;
                // If the checked grid is in another sub-map, update it's indexes
                if(thisGrid.gridX < 0){ --thisGrid.cubeX; thisGrid.gridX = thisGrid.gridX + mapCubeArrayLength;
                }else if(thisGrid.gridX >= mapCubeArrayLength){ ++thisGrid.cubeX; thisGrid.gridX = thisGrid.gridX - mapCubeArrayLength; }
                if(thisGrid.gridY < 0){ --thisGrid.cubeY; thisGrid.gridY = thisGrid.gridY + mapCubeArrayLength;
                }else if(thisGrid.gridY >= mapCubeArrayLength){ ++thisGrid.cubeY; thisGrid.gridY = thisGrid.gridY - mapCubeArrayLength; }
                // If the sub-map that the checked grid belongs to is empty or not
                int mapInd = mapArrayInd[thisGrid.cubeX][thisGrid.cubeY];
                if (mapInd == -1) continue;
                // the neighbor cell
                mapCell_t *thisCell = grid2Cell(&thisGrid);
                // save neighbor cell for calculating traversability
                if (thisCell->elevation != -FLT_MAX){
                    xyzVector.push_back(thisCell->xyz->x);
                    xyzVector.push_back(thisCell->xyz->y);
                    xyzVector.push_back(thisCell->xyz->z);
                }
            }
        }

        return xyzVector;
    }

    void traversabilityMapPrediction(){

        if (_makeTraversabilityPredictionFlag == false)
            return;


        // Prediction Process
        for (int i = 0; i < mapArrayLength; ++i){
            for (int j = 0; j < mapArrayLength; ++j){
                // no need to predict
                if (predictionArrayFlag[i][j] == false)
                    continue;

                if (mapArrayInd[i][j] == -1) continue;

                // Training data
                vector<float> yTrainVec;

                for (int k = -1; k <= 1; ++k){
                    for (int l = -1; l <= 1; ++l){
                        // skip four corners
                        if (std::abs(k) + std::abs(l) == 2)
                            continue;
                        // sub-map non-existing
                        if (i+k < 0 || i+k >= mapArrayLength || j+l < 0 || j+l >= mapArrayLength) continue;
                        int mapInd = mapArrayInd[i+k][j+l];
                        if (mapInd == -1) continue;
                        mapArray[mapInd]->findTrainingData("traversability");
                        yTrainVec.insert(std::end(yTrainVec), std::begin(mapArray[mapInd]->yTrain2), std::end(mapArray[mapInd]->yTrain2));
                    }
                }

                if (yTrainVec.size() == 0)
                    continue;

                Eigen::MatrixXf yTrain = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(yTrainVec.data(), yTrainVec.size(), 1);

                // Testing data
                int mapInd = mapArrayInd[i][j];

                if (mapArray[mapInd]->xTest.size() == 0)
                    continue;
                // Predicting
                Eigen::MatrixXf Ks = mapArray[mapInd]->Ks; // covariance matrix

                Eigen::MatrixXf ybar = (Ks * yTrain).array();
                Eigen::MatrixXf kbar = mapArray[mapInd]->kbar;

                vector<mapCell_t*> testList = mapArray[mapInd]->testList; // testing data list
                // Update Occupancy
                for (int k = 0; k < testList.size(); ++k){

                    if (std::isnan(ybar(k,0)) || std::isnan(kbar(k,0)))
                        continue;

                    if (ybar(k,0) == 0 && kbar(k,0) == 0) // too far away to training points, all zero
                        continue;

                    mapCell_t* thisCell = testList[k];

                    float alphaOccu = thisCell->alphaOccu;
                    float betaOccu = thisCell->betaOccu;
                    alphaOccu += ybar(k,0);
                    betaOccu  += kbar(k,0) - ybar(k,0);
                    float occupancy = alphaOccu / (alphaOccu + betaOccu);

                    if (!std::isnan(occupancy))
                        thisCell->updateTraversabilityAlphaBeta(alphaOccu, betaOccu, occupancy);
                }
            }
        }
    }


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////// Occupancy Map (local) //////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void publishMap(){
        // Publish Occupancy Grid Map and Elevation Map
        pubCount++;
        if (pubCount >= 10){
            pubCount = 0;
            if (getRobotPosition() == false) // Get Robot Position
                return;
            publishLocalMap();
            publishTraversabilityMap();
        }
    }

    void publishLocalMap(){

        if (pubOccupancyMapLocal.getNumSubscribers() == 0 &&
            pubOccupancyMapLocalHeight.getNumSubscribers() == 0)
            return;

        // 1.3 Initialize local occupancy grid map to unknown, height to -FLT_MAX
        std::fill(occupancyMap2DHeight.occupancy.data.begin(), occupancyMap2DHeight.occupancy.data.end(), -1);
        std::fill(occupancyMap2DHeight.height.begin(), occupancyMap2DHeight.height.end(), -FLT_MAX);
        std::fill(occupancyMap2DHeight.costMap.begin(), occupancyMap2DHeight.costMap.end(), 0);
        
        // local map origin x and y
        localMapOriginPoint.x = robotPoint.x - localMapLength / 2;
        localMapOriginPoint.y = robotPoint.y - localMapLength / 2;
        localMapOriginPoint.z = robotPoint.z;
        // local map origin cube id (in global map)
        localMapOriginGrid.cubeX = int((localMapOriginPoint.x + mapCubeLength/2.0) / mapCubeLength) + rootCubeIndex;
        localMapOriginGrid.cubeY = int((localMapOriginPoint.y + mapCubeLength/2.0) / mapCubeLength) + rootCubeIndex;
        if (localMapOriginPoint.x + mapCubeLength/2.0 < 0)  --localMapOriginGrid.cubeX;
        if (localMapOriginPoint.y + mapCubeLength/2.0 < 0)  --localMapOriginGrid.cubeY;
        // local map origin grid id (in sub-map)
        float originCubeOriginX, originCubeOriginY; // the orign of submap that the local map origin belongs to (note the submap may not be created yet, cannot use originX and originY)
        originCubeOriginX = (localMapOriginGrid.cubeX - rootCubeIndex) * mapCubeLength - mapCubeLength/2.0;
        originCubeOriginY = (localMapOriginGrid.cubeY - rootCubeIndex) * mapCubeLength - mapCubeLength/2.0;
        localMapOriginGrid.gridX = int((localMapOriginPoint.x - originCubeOriginX) / mapResolution);
        localMapOriginGrid.gridY = int((localMapOriginPoint.y - originCubeOriginY) / mapResolution);

        // 2 Calculate local occupancy grid map root position
        occupancyMap2DHeight.header.stamp = ros::Time::now();
        occupancyMap2DHeight.occupancy.header.stamp = occupancyMap2DHeight.header.stamp;
        occupancyMap2DHeight.occupancy.info.origin.position.x = localMapOriginPoint.x;
        occupancyMap2DHeight.occupancy.info.origin.position.y = localMapOriginPoint.y;
        occupancyMap2DHeight.occupancy.info.origin.position.z = localMapOriginPoint.z + 10; // add 10, just for visualization

        // extract all info
        for (int i = 0; i < localMapArrayLength; ++i){
            for (int j = 0; j < localMapArrayLength; ++j){

                int indX = localMapOriginGrid.gridX + i;
                int indY = localMapOriginGrid.gridY + j;

                grid_t thisGrid;

                thisGrid.cubeX = localMapOriginGrid.cubeX + indX / mapCubeArrayLength;
                thisGrid.cubeY = localMapOriginGrid.cubeY + indY / mapCubeArrayLength;

                thisGrid.gridX = indX % mapCubeArrayLength;
                thisGrid.gridY = indY % mapCubeArrayLength;

                // if sub-map is not created yet
                if (mapArrayInd[thisGrid.cubeX][thisGrid.cubeY] == -1) {
                    continue;
                }
                
                mapCell_t *thisCell = grid2Cell(&thisGrid);

                // skip unknown grid
                if (thisCell->elevation != -FLT_MAX){
                    int index = i + j * localMapArrayLength; // index of the 1-D array 
                    occupancyMap2DHeight.height[index] = thisCell->elevation;
                    occupancyMap2DHeight.occupancy.data[index] = thisCell->occupancy > 0.5 ? 100 : 0;
                }
            }
        }

        pubOccupancyMapLocalHeight.publish(occupancyMap2DHeight);
        pubOccupancyMapLocal.publish(occupancyMap2DHeight.occupancy);
    }
    

    void initializeLocalOccupancyMap(){
        // initialization of customized map message
        occupancyMap2DHeight.header.frame_id = "map";
        occupancyMap2DHeight.occupancy.info.width = localMapArrayLength;
        occupancyMap2DHeight.occupancy.info.height = localMapArrayLength;
        occupancyMap2DHeight.occupancy.info.resolution = mapResolution;
        
        occupancyMap2DHeight.occupancy.info.origin.orientation.x = 0.0;
        occupancyMap2DHeight.occupancy.info.origin.orientation.y = 0.0;
        occupancyMap2DHeight.occupancy.info.origin.orientation.z = 0.0;
        occupancyMap2DHeight.occupancy.info.origin.orientation.w = 1.0;

        occupancyMap2DHeight.occupancy.data.resize(occupancyMap2DHeight.occupancy.info.width * occupancyMap2DHeight.occupancy.info.height);
        occupancyMap2DHeight.height.resize(occupancyMap2DHeight.occupancy.info.width * occupancyMap2DHeight.occupancy.info.height);
        occupancyMap2DHeight.costMap.resize(occupancyMap2DHeight.occupancy.info.width * occupancyMap2DHeight.occupancy.info.height);
    }    

    bool getRobotPosition(){
        try{listener.lookupTransform("map","base_link", ros::Time(0), transform); } 
        catch (tf::TransformException ex){ ROS_ERROR("Transfrom Failure."); return false; }

        robotPoint.x = transform.getOrigin().x();
        robotPoint.y = transform.getOrigin().y();
        robotPoint.z = transform.getOrigin().z();

        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////// Point Cloud /////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void publishTraversabilityMap(){

        if (pubElevationCloud.getNumSubscribers() == 0)
            return;

        // 1. Find robot current cube index
        int currentCubeX = int((robotPoint.x + mapCubeLength/2.0) / mapCubeLength) + rootCubeIndex;
        int currentCubeY = int((robotPoint.y + mapCubeLength/2.0) / mapCubeLength) + rootCubeIndex;

        if (robotPoint.x + mapCubeLength/2.0 < 0)  --currentCubeX;
        if (robotPoint.y + mapCubeLength/2.0 < 0)  --currentCubeY;

        // 2. Loop through all the sub-maps that are nearby
        for (int i = -_visualizationRadius + currentCubeX; i <= _visualizationRadius + currentCubeX; ++i){
            for (int j = -_visualizationRadius + currentCubeY; j <= _visualizationRadius + currentCubeY; ++j){

                if (i < 0 || i >= mapArrayLength ||  j < 0 || j >= mapArrayLength) continue;

                if (mapArrayInd[i][j] == -1) continue;

                *laserCloudElevation += mapArray[mapArrayInd[i][j]]->cloud;
            }
        }

        pcl::PointCloud<PointType> laserCloudDS;
        downSizeFilter.setInputCloud(laserCloudElevation);
        downSizeFilter.filter(laserCloudDS);
        *laserCloudElevation = laserCloudDS;
        // 3. Publish elevation point cloud
        sensor_msgs::PointCloud2 laserCloudTemp;
        pcl::toROSMsg(*laserCloudElevation, laserCloudTemp);
        laserCloudTemp.header.frame_id = "/map";
        laserCloudTemp.header.stamp = ros::Time::now();
        pubElevationCloud.publish(laserCloudTemp);

        laserCloudElevation->clear();
    }
};




int main(int argc, char** argv){

    ros::init(argc, argv, "traversability_mapping");
    
    TraversabilityMapping tMapping;

    std::thread predictionThread(&TraversabilityMapping::BGK, &tMapping);

    ROS_INFO("\033[1;32m---->\033[0m Traversability Mapping Started.");

    ros::spin();

    return 0;
}