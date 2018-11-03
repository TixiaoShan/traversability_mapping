#include <pluginlib/class_list_macros.h>
#include "planner/tm_planner.h"
#include <unistd.h>

//register this planner as a BaseTMPlanner plugin
PLUGINLIB_EXPORT_CLASS(tm_planner::TMPlanner, nav_core::BaseGlobalPlanner)
 
	
namespace tm_planner {

	TMPlanner::TMPlanner (){}

	TMPlanner::TMPlanner(const std::string name, costmap_2d::Costmap2DROS* costmap_ros){
		initialize(name, costmap_ros);
	}
	///////////////////////////////////////////////////////////////////////////////////////////////////
	// Initialize the Plugin
	void TMPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
		// initialize subscriptions
		subPath = nh.subscribe("/global_path", 5, &TMPlanner::pathHandler, this);
		pubGoal = nh.advertise<geometry_msgs::PoseStamped>("/prm_goal", 5);
		// visualize twist command
		subTwistCommand1 = nh.subscribe<nav_msgs::Path>("/move_base/TrajectoryPlannerROS/local_plan", 5, &TMPlanner::twistCommandHandler, this);
		subTwistCommand2 = nh.subscribe<nav_msgs::Path>("/move_base/DWAPlannerROS/local_plan", 5, &TMPlanner::twistCommandHandler, this);
		// Publisher
        pubTwistCommand = nh.advertise<nav_msgs::Path>("/twist_command", 5);
	}

	// visualize twist command
	void TMPlanner::twistCommandHandler(const nav_msgs::Path::ConstPtr& pathMsg){

		try{ listener.lookupTransform("map","base_link", ros::Time(0), transform); } 
        catch (tf::TransformException ex){ return; }

        nav_msgs::Path outTwist = *pathMsg;

        for (int i = 0; i < outTwist.poses.size(); ++i)
            outTwist.poses[i].pose.position.z = transform.getOrigin().z() + 1.0;

        pubTwistCommand.publish(outTwist);
    }

    // receive path from prm global planner
	void TMPlanner::pathHandler(const nav_msgs::Path::ConstPtr& pathMsg){
		// std::lock_guard<std::mutex> lock(mtx);
		// if the planner couldn't find a feasible path, pose size should be 0
		globalPath = *pathMsg;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////
	bool TMPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan){
		
		// 1. Publish Goal to PRM Planner
		ROS_INFO("Goal Received at: [%lf, %lf, %lf]", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
		pubGoal.publish(goal);

		// 2. if the planner couldn't find a feasible path, pose size should be 0
		if (globalPath.poses.size() == 0){
			pubGoal.publish(goal);
			return false;
		}
		ROS_INFO("A Valid Path Received!");

		// 3. Extract Path
		geometry_msgs::PoseStamped this_pos = goal;
		for (int i = 0; i < globalPath.poses.size(); ++i){
			this_pos = globalPath.poses[i];
			plan.push_back(this_pos);
		}

		plan.back().pose.orientation = goal.pose.orientation;

		globalPath.poses.clear();

		return true; 
	}

};