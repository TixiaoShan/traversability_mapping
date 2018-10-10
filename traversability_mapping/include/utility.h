#ifndef _UTILITY_TM_H_
#define _UTILITY_TM_H_

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <interactive_markers/interactive_marker_server.h>

#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <pcl_ros/transforms.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <array> // c++11
#include <thread> // c++11
#include <mutex> // c++11

#include "marker/Marker.h"
#include "marker/MarkerArray.h"

#include "planner/kdtree.h"
#include "planner/cubic_spline_interpolator.h"

#include "elevation_msgs/OccupancyElevation.h"

using namespace std;

typedef pcl::PointXYZI  PointType;
typedef struct kdtree kdtree_t;
typedef struct kdres kdres_t;

/*
    Prediction Settings
    */
extern const bool _makeElevationPredictionFlag = true;
extern const bool _makeTraversabilityPredictionFlag = false;
extern const float _predictionKernalSize = 0.3; // predict elevation within x meters

/*
    Mapping Configuration
    */
extern const float mapResolution = 0.1; // map resolution
extern const float mapCubeLength = 1.0; // the length of a sub-map (meters)
extern const int mapCubeArrayLength = mapCubeLength / mapResolution; // the grid dimension of a sub-map (mapCubeLength / mapResolution)
extern const int mapArrayLength = 2000 / mapCubeLength; // the sub-map dimension of global map (2000m x 2000m)
extern const int rootCubeIndex = mapArrayLength / 2; // by default, robot is at the center of global map at the beginning
/*
    Mapping 2D Publish Configuration
    */
extern const int localMapLength = 20; // length of the local occupancy grid map (meter)
extern const int localMapArrayLength = localMapLength / mapResolution;
/*
    Robot Settings
    */
extern const float robotRadius = 0.2;
extern const float _sensorHeight = 0.5;
extern const int footprintRadiusLength = int(robotRadius / mapResolution);
extern const float _slopeAngleThreshold = 20; // slope angle threshold
extern const float _stepHeightThreshold = 0.1;

/*
    Filter Settings
    */
extern const float _rangeLimit = 12; // point that has range within this value to robot is sent to mapping
extern const int _filterHeightMapArrayLength = _rangeLimit*2 / mapResolution;
/*
    Visualization Settings
    */
extern const int _visualizationRadius = 100;
extern const float _visualizationDownSampleResolution = 0.2;
/*
    Planner Settings
    */
extern const int NUM_COSTS = 3;
extern const int tmp[] = {2};
extern const std::vector<int> costHierarchy(tmp, tmp+sizeof(tmp)/sizeof(int));// c++11 initialization: costHierarchy{0, 1, 2}
extern const float terrainRoughnessThreshold = 0.1;
// PRM Planner Settings
extern const float inflationRadius = 0.3;
extern const float neighborSampleRadius  = 0.5; // 7.5 or 10
extern const float neighborConnectHeight = 1.0;//mapResolution * 3;
extern const float neighborConnectRadius = 2.0; // 15 or 20
extern const float neighborSearchRadius = localMapLength / 2;





struct grid_t;
struct mapCell_t;
struct childMap_t;
struct state_t;
struct neighbor_t;

/*
    This struct is used to send map from mapping package to prm package
    */
struct grid_t{
    int mapID;
    int cubeX;
    int cubeY;
    int gridX;
    int gridY;
    int gridIndex;
};

/*
    Cell Definition:
    a cell is a member of a grid in a sub-map
    a grid can have several cells in it. 
    a cell represent one height information
    */

struct mapCell_t{

    PointType *xyz; // it's a pointer to the corresponding point in the point cloud of submap

    grid_t grid;

    int times;
    
    float occupancy;
    float elevation;

    float occupancyVar;
    float elevationVar;

    float alphaOccu, betaOccu; // hyperparameters for occupancy of this cell
    float alphaElev, betaElev; // hyperparameters for elevation of this cell

    bool observingFlag;

    bool observedNonTraversableFlag; // non-traversable decided by cloud filter

    


    mapCell_t(){

        times = 0;

        occupancy = 0; // initialized as unkown
        elevation = -FLT_MAX;

        occupancyVar = 0;
        elevationVar = 0;

        // initial priors
        alphaOccu = 0; betaOccu = 1e-6;
        alphaElev = 0; betaElev = 1e-6;
        
        observingFlag = false;
        
        observedNonTraversableFlag = false;

    }

    void updatePoint(){
        xyz->z = elevation;
        xyz->intensity = occupancy;
    }

    void updateElevation(float elevIn){
        elevation = elevIn;
        updatePoint();
    }
    void updateOccupancy(float occupIn){
        occupancy = occupIn;
        updatePoint();
    }
    // Elevation updates for incoming point cloud
    void updateElevationAlphaBeta(float height){
        alphaElev += height;
        betaElev  += 1 - height;
        height = alphaElev / (alphaElev + betaElev);
        updateElevation(height);
    }
    void updateElevationAlphaBeta(float alpha, float beta, float height){
        alphaElev = alpha;
        betaElev  = beta;
        updateElevation(height);
    }
    // Traversability updates for incoming point cloud
    void updateTraversabilityAlphaBeta(float occup){
        if (occup == -1) return;// -1 from traversability_filter
        alphaOccu += occup;
        betaOccu  += 1 - occup;
        occup = alphaOccu / (alphaOccu + betaOccu);
        updateOccupancy(occup);
    }
    void updateTraversabilityAlphaBeta(float alpha, float beta, float occup){
        alphaOccu = alpha;
        betaOccu  = beta;
        updateOccupancy(occup);
    }
};


/*
    Sub-map Definition:
    childMap_t is a small square. We call it "cellArray". 
    It composes the whole map
    */
struct childMap_t{

    vector<vector<mapCell_t*> > cellArray;
    int subInd; //sub-map's index in 1d mapArray
    int indX; // sub-map's x index in 2d array mapArrayInd
    int indY; // sub-map's y index in 2d array mapArrayInd
    float originX; // sub-map's x root coordinate
    float originY; // sub-map's y root coordinate
    pcl::PointCloud<PointType> cloud;

    Eigen::MatrixXf Ks;
    Eigen::MatrixXf kbar;

    childMap_t(int id, int indx, int indy){

        subInd = id;
        indX = indx;
        indY = indy;
        originX = (indX - rootCubeIndex) * mapCubeLength - mapCubeLength/2.0;
        originY = (indY - rootCubeIndex) * mapCubeLength - mapCubeLength/2.0;

        // allocate and initialize each cell
        cellArray.resize(mapCubeArrayLength);
        for (int i = 0; i < mapCubeArrayLength; ++i)
            cellArray[i].resize(mapCubeArrayLength);

        for (int i = 0; i < mapCubeArrayLength; ++i)
            for (int j = 0; j < mapCubeArrayLength; ++j)
                cellArray[i][j] = new mapCell_t;
        // allocate point cloud for visualization
        cloud.points.resize(mapCubeArrayLength*mapCubeArrayLength);

        // initialize cell pointer to cloud point
        for (int i = 0; i < mapCubeArrayLength; ++i)
            for (int j = 0; j < mapCubeArrayLength; ++j)
                cellArray[i][j]->xyz = &cloud.points[i + j*mapCubeArrayLength];

        // initialize each point in the point cloud, also each cell
        for (int i = 0; i < mapCubeArrayLength; ++i){
            for (int j = 0; j < mapCubeArrayLength; ++j){
                
                // point cloud initialization
                int index = i + j * mapCubeArrayLength;
                cloud.points[index].x = originX + i * mapResolution;
                cloud.points[index].y = originY + j * mapResolution;
                cloud.points[index].z = std::numeric_limits<float>::quiet_NaN();
                cloud.points[index].intensity = cellArray[i][j]->occupancy;

                // cell position in the array of submap
                cellArray[i][j]->grid.mapID = subInd;
                cellArray[i][j]->grid.cubeX = indX;
                cellArray[i][j]->grid.cubeY = indY;
                cellArray[i][j]->grid.gridX = i;
                cellArray[i][j]->grid.gridY = j;
                cellArray[i][j]->grid.gridIndex = index;
            }
        }
    }

    vector<float> xTrain; // coordinates (x,y) for training data
    vector<float> yTrain; // elevation for training data
    vector<float> yTrain2; // traversability for training data
    vector<float> xTest; // coordinates (x,y) for testing data
    vector<mapCell_t*> trainList; // pointer to the cell of training data
    vector<mapCell_t*> testList; // pointer to the cell of testing data

    void findTrainingData(std::string inferType){
        if (inferType == "elevation"){
            if (xTrain.size() != 0 || yTrain.size() != 0)
                return;
            // elevation training data
            for (int i = 0; i < mapCubeArrayLength; ++i){
                for (int j = 0; j < mapCubeArrayLength; ++j){
                    if (cellArray[i][j]->observingFlag == true){
                        xTrain.push_back(cellArray[i][j]->xyz->x);
                        xTrain.push_back(cellArray[i][j]->xyz->y);
                        yTrain.push_back(cellArray[i][j]->elevation);
                        trainList.push_back(cellArray[i][j]);
                    }
                }
            }
        }
        if (inferType == "traversability"){
            if (yTrain2.size() != 0)
                return;
            // traversability training data
            int listSize = trainList.size();
            for (int i = 0; i < listSize; ++i){
                yTrain2.push_back(trainList[i]->occupancy);
            }
        }
    }

    void findTestingData(){
        if (xTest.size() != 0 || testList.size() != 0)
            return;
        // testing data
        for (int i = 0; i < mapCubeArrayLength; ++i){
            for (int j = 0; j < mapCubeArrayLength; ++j){
                if (cellArray[i][j]->observingFlag == false){
                    xTest.push_back(cellArray[i][j]->xyz->x);
                    xTest.push_back(cellArray[i][j]->xyz->y);
                    testList.push_back(cellArray[i][j]);
                }
            }
        }
    }

    void clearInferenceData(){
        xTrain.clear();
        yTrain.clear();
        yTrain2.clear();
        xTest.clear();
        trainList.clear();
        testList.clear();
    }
};



/*
    Robot State Defination
    */


struct state_t{
    double x[3]; //  1 - x, 2 - y, 3 - z
    float theta;
    int stateId;
    // # Cost types
    // # 0. obstacle cost
    // # 1. elevation cost
    // # 2. distance cost
    float costsToRoot[NUM_COSTS];
    float costsToParent[NUM_COSTS]; // used in RRT*
    float costsToGo[NUM_COSTS];

    state_t* parentState; // parent for this state in PRM and RRT*
    vector<neighbor_t> neighborList; // PRM adjencency list with edge costs
    vector<state_t*> childList; // RRT*

    // default initialization
    state_t(){
        parentState = NULL;
        for (int i = 0; i < NUM_COSTS; ++i){
            costsToRoot[i] = FLT_MAX;
            costsToParent[i] = FLT_MAX;
            costsToGo[i] = FLT_MAX;
        }
    }
    // use a state input to initialize new state
    
    state_t(state_t* stateIn){
        // pose initialization
        for (int i = 0; i < 3; ++i)
            x[i] = stateIn->x[i];
        theta = stateIn->theta;
        // regular initialization
        parentState = NULL;
        for (int i = 0; i < NUM_COSTS; ++i){
            costsToRoot[i] = FLT_MAX;
            costsToParent[i] = stateIn->costsToParent[i];
        }
    }
};


struct neighbor_t{
    state_t* neighbor;
    float edgeCosts[NUM_COSTS]; // the cost from this state to neighbor
    neighbor_t(){
        neighbor = NULL;
        for (int i = 0; i < NUM_COSTS; ++i)
            edgeCosts[i] = FLT_MAX;
    }
};









////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////      Some Functions    ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
state_t *compareState;
bool isStateExsiting(neighbor_t neighborIn){
    return neighborIn.neighbor == compareState ? true : false;
}



#endif
