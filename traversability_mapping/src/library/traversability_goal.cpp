#include "utility.h"

class TraversabilityGoal{

private:
    ros::NodeHandle nh;

    ros::Publisher pubGoal;
    ros::Subscriber subCancelGoal;

    interactive_markers::InteractiveMarkerServer *server;
    visualization_msgs::InteractiveMarker int_marker;
    visualization_msgs::Marker displayed_marker;
    visualization_msgs::InteractiveMarkerControl marker_control;
    visualization_msgs::InteractiveMarkerControl rotate_control;

public:
    TraversabilityGoal():
    nh("~")
    {
        pubGoal = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
        subCancelGoal = nh.subscribe<geometry_msgs::PointStamped>("/clicked_point", 1, &TraversabilityGoal::cancelGoal, this); 

        initializeGoalInteractiveMarker();
    }

    void initializeGoalInteractiveMarker(){
        // create an interactive marker for our server
        server = new interactive_markers::InteractiveMarkerServer("traversability_goal");
        
        int_marker.header.frame_id = "map";
        int_marker.header.stamp=ros::Time::now();
        int_marker.scale = 5;
        int_marker.name = "goal_flag";
        int_marker.description = "Goal";

        // create a mesh flag marker
        // displayed_marker.type = visualization_msgs::Marker::ARROW;
        displayed_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        displayed_marker.mesh_resource = "package://traversability_mapping/launch/media/flag.dae";
        displayed_marker.scale.x = 1.5;
        displayed_marker.scale.y = 1.5;
        displayed_marker.scale.z = 3.0;
        displayed_marker.pose.orientation.x = 0;
        displayed_marker.pose.orientation.y = 0;
        displayed_marker.pose.orientation.z = 0;
        displayed_marker.pose.orientation.w = 1;
        displayed_marker.color.r = 0.0;
        displayed_marker.color.g = 1.0;
        displayed_marker.color.b = 0.0;
        displayed_marker.color.a = 1.0;

        // create a non-interactive control
        marker_control.always_visible = true;
        marker_control.markers.push_back( displayed_marker );
        int_marker.controls.push_back( marker_control );

        // create a control
        rotate_control.orientation.w = 1;
        rotate_control.orientation.x = 0;
        rotate_control.orientation.y = 1;
        rotate_control.orientation.z = 0;
        rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
        rotate_control.name = "move_plane";
        int_marker.controls.push_back(rotate_control);
        rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        rotate_control.name = "move_z";
        int_marker.controls.push_back(rotate_control);

        // server 
        server->insert(int_marker);
        server->setCallback(int_marker.name, boost::bind(&TraversabilityGoal::goalHandler, this, _1));
        server->applyChanges();
    }

    // Publish Goal
    void goalHandler(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = ros::Time::now();
        goal.pose.position.x = feedback->pose.position.x;
        goal.pose.position.y = feedback->pose.position.y;
        goal.pose.position.z = feedback->pose.position.z;
        goal.pose.orientation.x = 0;
        goal.pose.orientation.y = 0;
        goal.pose.orientation.z = 0;
        goal.pose.orientation.w = 1;
        pubGoal.publish(goal);
    }
    // Publish an invalid goal to stop
    void cancelGoal(const geometry_msgs::PointStampedConstPtr &msg){
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = ros::Time::now();
        goal.pose.position.x = -FLT_MAX;
        goal.pose.position.y = -FLT_MAX;
        goal.pose.position.z = -FLT_MAX;
        goal.pose.orientation.x = 0;
        goal.pose.orientation.y = 0;
        goal.pose.orientation.z = 0;
        goal.pose.orientation.w = 1;
        pubGoal.publish(goal);
    }


};






int main(int argc, char** argv){

    ros::init(argc, argv, "traversability_mapping");
    
    TraversabilityGoal TG;

    ROS_INFO("\033[1;32m---->\033[0m Traversability Goal Handler Started.");

    ros::spin();

    return 0;
}