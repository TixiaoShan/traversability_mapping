#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

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

using namespace std;

typedef pcl::PointXYZI  PointType;

class Cloud2Graph{
private:

    ros::NodeHandle nh;

    tf::TransformListener listener;
    tf::StampedTransform transform;

    ros::Publisher pubPRMGraph; // publish PRM nodes and edges

    ros::Subscriber subPRMnode;
    ros::Subscriber subPRMGraph;

    pcl::PointCloud<PointType>::Ptr nodesCloud;
    pcl::PointCloud<PointType>::Ptr graphCloud;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeNodes;

    double timeNewNodesCloud;
    double timeNewGraphCloud;

    bool newNodesCloud;
    bool newGraphCloud;

    PointType robotPoint;

public:
	Cloud2Graph():
        nh("~"){

        subPRMnode = nh.subscribe<sensor_msgs::PointCloud2>("/prm_cloud_nodes", 1, &Cloud2Graph::nodesPosHandler, this);
        subPRMGraph = nh.subscribe<sensor_msgs::PointCloud2>("/prm_cloud_graph", 1, &Cloud2Graph::graphPosHandler, this);

        pubPRMGraph = nh.advertise<visualization_msgs::Marker>("/prm_graph_", 1);

        nodesCloud.reset(new pcl::PointCloud<PointType>());
        graphCloud.reset(new pcl::PointCloud<PointType>());

        kdtreeNodes.reset(new pcl::KdTreeFLANN<PointType>());

        newNodesCloud = false;
        newGraphCloud = false;

        timeNewNodesCloud = 0;
        timeNewGraphCloud = 0;
    }
    
    void nodesPosHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
    	timeNewNodesCloud = laserCloudMsg->header.stamp.toSec();
    	newNodesCloud = true;
    	pcl::fromROSMsg(*laserCloudMsg, *nodesCloud);
        cout << "new nodes ..." << endl;
    }

    void graphPosHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
    	timeNewGraphCloud = laserCloudMsg->header.stamp.toSec();
    	newGraphCloud = true;
    	pcl::fromROSMsg(*laserCloudMsg, *graphCloud);
        cout << "new graph ..." << endl;
    }

    void searchGraph(){

        cout << "Searching ..." << endl;

    	const int cloudSize = nodesCloud->size();
    	// find start node
    	kdtreeNodes->setInputCloud(nodesCloud);
    	std::vector<int> pointSearchInd;
  		std::vector<float> pointSearchSqDis;
    	kdtreeNodes->nearestKSearch(robotPoint, 1, pointSearchInd, pointSearchSqDis);
    	int startID = pointSearchInd[0];
    	// reset costs
    	float *costToRoot = new float[cloudSize];
    	for (int i = 0; i < cloudSize; ++i)
    		nodesCloud->points[i].intensity = FLT_MAX;
    	nodesCloud->points[startID].intensity = 0;
    	// 3. BFS search
        // find nodes that the robot can go to
      
    }

    // PointType* minCostStateInQueue(vector<PointType*> Queue){
    //     // loop through nodes saved in Queue
    //     int minID = 0;
    //     int sizeQueue = Queue.size();
    //     float minCost = FLT_MAX;
    //     for (int i = 0; i < sizeQueue; ++i){
    //         PointType thisPoint = *Queue[i];
    //         if (thisPoint.intensity < minCost){
    //             minCost = thisPoint.intensity;
    //             minID = i;
    //         }
    //     }
    //     return Queue[minID];
    // }

    bool getRobotPosition(){
        // Listen to the TF transform and prepare for point cloud transformation
        try{listener.lookupTransform("map","base_link", ros::Time(0), transform);}
        catch (tf::TransformException ex){ ROS_ERROR("Transfrom Failure."); return false; }

        robotPoint.x = transform.getOrigin().x();
        robotPoint.y = transform.getOrigin().y();
        robotPoint.z = transform.getOrigin().z();

        cout << "Robot position got!" << endl;

        return true;
    }

    void publishPRM(){

        cout << "Visualizing ..." << endl;
        
        visualization_msgs::Marker line_list, path_list, points, goal_point, paths_list;
        paths_list.header.frame_id = goal_point.header.frame_id = points.header.frame_id = path_list.header.frame_id = line_list.header.frame_id = "map";
        paths_list.header.stamp = goal_point.header.stamp = points.header.stamp = path_list.header.stamp = line_list.header.stamp = ros::Time::now();
        paths_list.ns = goal_point.ns = points.ns = path_list.ns = line_list.ns = "points_and_lines";
        paths_list.action = goal_point.action = points.action = path_list.action = line_list.action = visualization_msgs::Marker::ADD;
        paths_list.pose.orientation.w = goal_point.pose.orientation.w = points.pose.orientation.w = path_list.pose.orientation.w = line_list.pose.orientation.w = 1.0;
        points.id = 0; path_list.id = 1; line_list.id = 2; goal_point.id = 3; paths_list.id = 4;
        points.type    = visualization_msgs::Marker::SPHERE_LIST;
        path_list.type = visualization_msgs::Marker::LINE_STRIP;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        goal_point.type= visualization_msgs::Marker::SPHERE_LIST;
        paths_list.type = visualization_msgs::Marker::LINE_LIST;
        geometry_msgs::Point p;
        std_msgs::ColorRGBA c;
        

        // 3. PRM nodes and edges

        // 3.1 PRM nodes
        points.scale.x = 0.2;
        points.color.r = 0; points.color.g = 1.0; points.color.b = 1.0;
        points.color.a =1.0;
        int sizeList = nodesCloud->size();
        for (int i = 0; i < sizeList; ++i){
            p.x = nodesCloud->points[i].x;
            p.y = nodesCloud->points[i].y;
            p.z = nodesCloud->points[i].z+0.13;
            points.points.push_back(p);
        }
        pubPRMGraph.publish(points);

        // 3.2 PRM edges
        line_list.scale.x = 0.03;
        line_list.color.r = 0.9; line_list.color.g = 1.0; line_list.color.b = 0.0;
        line_list.color.a = 1.0;

        for (int i = 0; i < sizeList; ++i){
            
            for (int j = 0; j < sizeList; ++j){
                // 1d index in 2d array
                int index = i + j * sizeList;
                // an edge is connectable between two nodes
                if (graphCloud->points[index].intensity == 1){
                    p.x = nodesCloud->points[i].x;
                    p.y = nodesCloud->points[i].y;
                    p.z = nodesCloud->points[i].z+0.1;
                    line_list.points.push_back(p);
                    p.x = nodesCloud->points[j].x;
                    p.y = nodesCloud->points[j].y;
                    p.z = nodesCloud->points[j].z+0.1;
                    line_list.points.push_back(p);
                }
            }
        }
        pubPRMGraph.publish(line_list);
    }

    void run(){
        
        if (newNodesCloud && newGraphCloud && std::abs(timeNewNodesCloud  - timeNewGraphCloud) < 0.01){

            newNodesCloud = false;
            newGraphCloud = false;

            if (getRobotPosition() == false) return;

            searchGraph();

            publishPRM();

        }
    }

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_odometry");

    ROS_INFO("\033[1;32m---->\033[0m hahaha.");

    Cloud2Graph c2g;

    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::spinOnce();

        c2g.run();

        rate.sleep();
    }

    return 0;
}