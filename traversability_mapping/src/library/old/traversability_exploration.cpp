#include "utility.h"

class TraversabilityExploration{

private:

    ros::NodeHandle nh;

    ros::Subscriber subGlobalOccuMap;
    ros::Subscriber subGoal;

    ros::Subscriber subLaserOdometry; // get robot position
    
    ros::Publisher pubGlobalPath;
    ros::Publisher pubLocalGoal;
    ros::Publisher pubPRMGraph; // publish PRM nodes and edges

    nav_msgs::OccupancyGrid globalOccuMap;
    nav_msgs::Path globalPath;

    bool planningFlag;

    state_t *robotState;
    state_t *goalState;
    state_t *randomState;
    

    ///////////// Planner ////////////
    vector<state_t*> nodeList;
    vector<state_t*> pathList;

    vector<state_t*> closedSet;
    vector<state_t*> openSet;

    float map_min[3]; // 0 - x, 1 - y, 2 - z
    float map_max[3];

    kdtree_t *kdtree;

public:

    TraversabilityExploration():
    nh("~"),
    planningFlag(false)
    {
        if (explorationPlanningFlag == false)
            return;

        subGlobalOccuMap = nh.subscribe<nav_msgs::OccupancyGrid>("/exploration_costmap/costmap/costmap", 1, &TraversabilityExploration::globalOccuMapHandler, this);
        subGoal          = nh.subscribe<geometry_msgs::PointStamped>("/clicked_point", 1, &TraversabilityExploration::goalHandler, this);
        subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/integrated_to_init", 1, &TraversabilityExploration::robotPosHandler, this);

        pubGlobalPath    = nh.advertise<nav_msgs::Path>("/astar_path", 1);
        pubPRMGraph      = nh.advertise<visualization_msgs::Marker>("/global_prm_graph", 2);
        pubLocalGoal     = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

        robotState = new state_t;
        goalState = new state_t;
        randomState = new state_t;

        kdtree = kd_create(3);
    }

    ~TraversabilityExploration(){}


    void globalOccuMapHandler(const nav_msgs::OccupancyGrid::ConstPtr& mapMsg){

        globalOccuMap = *mapMsg;

        map_min[0] = globalOccuMap.info.origin.position.x; 
        map_min[1] = globalOccuMap.info.origin.position.y;
        map_min[2] = 10;
        map_max[0] = globalOccuMap.info.origin.position.x + globalOccuMap.info.resolution * globalOccuMap.info.width; 
        map_max[1] = globalOccuMap.info.origin.position.y + globalOccuMap.info.resolution * globalOccuMap.info.height; 
        map_max[2] = 10;

        buildRoadMap();
    }

    void goalHandler(const geometry_msgs::PointStamped::ConstPtr& goalMsg){
        
        goalState->x[0] = goalMsg->point.x;
        goalState->x[1] = goalMsg->point.y;
        goalState->x[2] = goalMsg->point.z;

        planningFlag = true;
    }

    void robotPosHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry){
        robotState->x[0] = laserOdometry->pose.pose.position.z; // change coordinates from LOAM
        robotState->x[1] = laserOdometry->pose.pose.position.x;
        robotState->x[2] = laserOdometry->pose.pose.position.y - sensorHeight;
        robotState->theta = laserOdometry->pose.pose.orientation.y * M_PI;
    }


    void buildRoadMap(){
        // 1. generate samples
        generateSamples();
        // 2. Add edges and update state height if map is changed
        updateStatesAndEdges();  
        // 3. Planning
        AstarSearch();
        // 4. Visualize Roadmap
        publishPRM();
        // 5. Publish path to move_base and stop planning
        publishPathStop();

        pubLishLocalGoal();
    }



    void generateSamples(){

        double sampling_start_time = ros::Time::now().toSec();
        while (ros::Time::now().toSec() - sampling_start_time < 0.01 && ros::ok()){

            state_t* newState = new state_t;

            if (sampleState(newState)){
                // 1.1 Too close discard
                if (nodeList.size() != 0 && stateTooClose(newState) == true){
                    delete newState;
                    continue;
                }
                // 1.2 Save new state and insert to KD-tree
                nodeList.push_back(newState);
                insertIntoKdtree(newState);
            }
            else
                delete newState;
        }
    }

    bool stateTooClose(state_t *stateIn){
        // restrict the nearest sample that can be inserted to the roadmap
        state_t *nearestState = getNearestState(stateIn);
        if (nearestState == NULL)
            return false;

        if (distance(stateIn->x, nearestState->x) > neighborSampleRadius)
            return false;
            
        return true;
    }

    bool sampleState(state_t *stateCurr){
        // random x and y
        for (int i = 0; i < 3; ++i)
            stateCurr->x[i] = (double)rand()/(RAND_MAX + 1.0)*(map_max[i] - map_min[i]) 
                - (map_max[i] - map_min[i])/2.0 + (map_max[i] + map_min[i])/2.0;
        // random heading
        stateCurr->theta = (double)rand()/(RAND_MAX + 1.0) * 2 * M_PI - M_PI;
        // collision checking before getting height info
        if (isIncollision(stateCurr))
            return false;
        return true;
    }

    // Collision check (using state for input)
    bool isIncollision(state_t* stateIn){
        // if the state is outside the map, discard this state
        if (stateIn->x[0] <= map_min[0] || stateIn->x[0] >= map_max[0] 
            || stateIn->x[1] <= map_min[1] || stateIn->x[1] >= map_max[1])
            return true;
        // if the distance to the nearest obstacle is less than xxx, in collision
        int rounded_x = (int)((stateIn->x[0] - map_min[0]) / mapResolution);
        int rounded_y = (int)((stateIn->x[1] - map_min[1]) / mapResolution);
        int index = rounded_x + rounded_y * globalOccuMap.info.width;
        if (globalOccuMap.data[index] > 0) // == 100 or != 0
            return true;
        
        return false;
    }

    void insertIntoKdtree(state_t *stateCurr){
        kd_insert(kdtree, stateCurr->x, stateCurr);
    }

    state_t* getNearestState(state_t *stateIn){
        kdres_t *kdres = kd_nearest(kdtree, stateIn->x);
        if (kd_res_end (kdres)){
            kd_res_free (kdres);
            return NULL;
        }
        state_t* nearestState = (state_t*) kd_res_item_data(kdres);
        kd_res_free (kdres);
        return nearestState;
    }

    void getNearStates(state_t *stateIn, vector<state_t*>& vectorNearStatesOut, double radius){
        kdres_t *kdres = kd_nearest_range (kdtree, stateIn->x, radius);
        vectorNearStatesOut.clear();
        // Create the vector data structure for storing the results
        int numNearVertices = kd_res_size (kdres);
        if (numNearVertices == 0) {
            kd_res_free (kdres);
            return;
        }
        // Place pointers to the near vertices into the vector 
        kd_res_rewind (kdres);
        while (!kd_res_end(kdres)) {
            state_t *stateCurr = (state_t *) kd_res_item_data (kdres);
            vectorNearStatesOut.push_back(stateCurr);
            kd_res_next(kdres);
        }
        // Free temporary memory
        kd_res_free (kdres);
    }

    // Calculate the Euclidean distance between two samples (2D)
    float distance(double state_from[3], double state_to[3]){
        return sqrt((state_to[0]-state_from[0])*(state_to[0]-state_from[0]) + 
                    (state_to[1]-state_from[1])*(state_to[1]-state_from[1]) +
                    (state_to[2]-state_from[2])*(state_to[2]-state_from[2]));
    }


    void updateStatesAndEdges(){
        bool firstPropagation = true;
        double edge_propagation_start_time = ros::Time::now().toSec();
        while (ros::Time::now().toSec() - edge_propagation_start_time < 0.05 && ros::ok()){
            // 0. find a random state
            for (int i = 0; i < 3; ++i)
                randomState->x[i] = (double)rand()/(RAND_MAX + 1.0)*(map_max[i] - map_min[i]) 
                    - (map_max[i] - map_min[i])/2.0 + (map_max[i] + map_min[i])/2.0;
            if (firstPropagation == true){
                for (int i = 0; i < 2; ++i)
                    randomState->x[i] = robotState->x[i];
                firstPropagation = false;
            }
            // 1. add edges for the nodes that are within a certain radius of mapCenter
            vector<state_t*> nearStates;
            getNearStates(randomState, nearStates, 10);
            if (nearStates.size() == 0)
                continue;
            // 4. loop through all neighbors
            float edgeCosts[NUM_COSTS];
            neighbor_t thisNeighbor;
            for (int i = 0; i < nearStates.size(); ++i){
                for (int j = i+1; j < nearStates.size(); ++j){
                    // 4.3 distance larger than x, too far to connect
                    float distanceBetween = distance(nearStates[i]->x, nearStates[j]->x);
                    if (distanceBetween > neighborConnectRadius || distanceBetween < 0.3){
                        deleteEdge(nearStates[i], nearStates[j]);
                        continue;
                    }
                    // 4.4 this edge is connectable
                    if(edgePropagation(nearStates[i], nearStates[j], edgeCosts) == true){
                        // even if edge already exists, we still need to update costs
                        deleteEdge(nearStates[i], nearStates[j]);
                        for (int k = 0; k < NUM_COSTS; ++k)
                            thisNeighbor.edgeCosts[k] = edgeCosts[k];

                        thisNeighbor.neighbor = nearStates[j];
                        nearStates[i]->neighborList.push_back(thisNeighbor);
                        thisNeighbor.neighbor = nearStates[i];
                        nearStates[j]->neighborList.push_back(thisNeighbor);
                    }else{ // edge is not connectable, delete old edge if it exists
                        deleteEdge(nearStates[i], nearStates[j]);
                    }
                } 
            }
        }
    }

    void deleteEdge(state_t* stateA, state_t* stateB){
        // "remove" compacts the elements that differ from the value to be removed (state_in) in the beginning of the vector 
        // and returns the iterator to the first element after that range. Then "erase" removes these elements (who's value is unspecified).
        compareState = stateB;
        stateA->neighborList.erase(std::remove_if(stateA->neighborList.begin(), stateA->neighborList.end(), isStateExsiting), stateA->neighborList.end());
        compareState = stateA;
        stateB->neighborList.erase(std::remove_if(stateB->neighborList.begin(), stateB->neighborList.end(), isStateExsiting), stateB->neighborList.end());
    }

    

    bool edgePropagation(state_t *state_from, state_t *state_to, float edgeCosts[NUM_COSTS]){
        // 0. initialize edgeCosts
        for (int i = 0; i < NUM_COSTS; ++i)
            edgeCosts[i] = 0;
        // 1. segment the edge for collision checking
        int steps = floor(distance(state_from->x, state_to->x) / (mapResolution));
        float stepX = (state_to->x[0]-state_from->x[0]) / steps;
        float stepY = (state_to->x[1]-state_from->x[1]) / steps;
        float stepZ = (state_to->x[2]-state_from->x[2]) / steps;
        // 2. allocate memory for a state, this state must be deleted after collision checking
        state_t *stateCurr = new state_t;;
        stateCurr->x[0] = state_from->x[0];
        stateCurr->x[1] = state_from->x[1];
        stateCurr->x[2] = state_from->x[2];

        // 3. collision checking loop
        for (int stepCount = 0; stepCount < steps; ++stepCount){
            stateCurr->x[0] += stepX;
            stateCurr->x[1] += stepY;
            stateCurr->x[2] += stepZ;

            if (isIncollision(stateCurr)){
                delete stateCurr;
                return false;
            }

            edgeCosts[2] = edgeCosts[2] + mapResolution; // distance cost
        }

        delete stateCurr;
        return true;
    }







    void publishPRM(){
        
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
        if (pubPRMGraph.getNumSubscribers() != 0){
            // 3.1 PRM nodes
            points.scale.x = 0.25;
            points.color.r = 0; points.color.g = 1.0; points.color.b = 1.0;
            points.color.a = 1.0;
            for (int i = 0; i < nodeList.size(); ++i){
                p.x = nodeList[i]->x[0];
                p.y = nodeList[i]->x[1];
                p.z = globalOccuMap.info.origin.position.z;//nodeList[i]->x[2];
                points.points.push_back(p);
            }
            pubPRMGraph.publish(points);

            // 3.2 PRM edges
            line_list.scale.x = 0.05;
            line_list.color.r = 0.9; line_list.color.g = 1.0; line_list.color.b = 0.0;
            line_list.color.a = 0.5;

            for (int i = 0; i < nodeList.size(); ++i){
                int numNeighbors = nodeList[i]->neighborList.size();
                for (int j = 0; j < numNeighbors; ++j){
                    p.x = nodeList[i]->x[0]; p.y = nodeList[i]->x[1]; p.z = globalOccuMap.info.origin.position.z;//nodeList[i]->x[2];
                    line_list.points.push_back(p);
                    p.x = nodeList[i]->neighborList[j].neighbor->x[0]; p.y = nodeList[i]->neighborList[j].neighbor->x[1]; p.z = globalOccuMap.info.origin.position.z;//nodeList[i]->neighborList[j].neighbor->x[2]+0.1;
                    line_list.points.push_back(p);
                }
            }
            pubPRMGraph.publish(line_list);
        }
        
    }










    void AstarSearch(){
        double start_time = ros::Time::now().toSec();

        pathList.clear();
        globalPath.poses.clear();   

        if (planningFlag == false)
            return;

        // Initialization
        closedSet.clear();
        openSet.clear();
        for (int i = 0; i < nodeList.size(); ++i){
            nodeList[i]->parentState = NULL;
            nodeList[i]->costsToRoot[2] = FLT_MAX;
            nodeList[i]->costsToGo[2] = FLT_MAX;
        }
        // 2. find the state that is the closest to the robot
        state_t *startState = getNearestState(robotState);
        if (startState == NULL || startState->neighborList.size() == 0)
            return;
        for (int i = 0; i < NUM_COSTS; ++i){
            startState->costsToRoot[2] = 0;
            startState->costsToGo[2] = distance(startState->x, goalState->x);
        }
        openSet.push_back(startState);
        //
        state_t* nearestGoalState = getNearestState(goalState);
        // 3. A* search
        while(openSet.size() > 0){
            // Find minimal fScore node that is in openSet
            state_t *fromState = minCostIndexAstar();

            if (fromState == nearestGoalState)
                break;
            // remove from openSet to closedSet
            openSet.erase(remove(openSet.begin(), openSet.end(), fromState), openSet.end());
            closedSet.push_back(fromState);
            // loop through 8 neighbor grids

            // lopp through all neighbors of this state
            for (int i = 0; i < fromState->neighborList.size(); ++i){
                state_t *toState = fromState->neighborList[i].neighbor;
                if (std::find(closedSet.begin(), closedSet.end(), toState) != closedSet.end())
                    continue;
                // The distance from start to a neighbor
                float tentative_gScore = fromState->costsToRoot[2] + distance(fromState->x, toState->x);

                if (std::find(openSet.begin(), openSet.end(), toState) == openSet.end()) 
                    openSet.push_back(toState); // Discover a new node
                else if (tentative_gScore >= toState->costsToRoot[2])
                    continue; // This is not a better path.

                // This path is the best until now. Record it!
                toState->parentState = fromState;
                toState->costsToRoot[2] = tentative_gScore;
                toState->costsToGo[2] = toState->costsToRoot[2] + distance(toState->x, goalState->x);
            }
        }
        //

        if (nearestGoalState->parentState == NULL) // no path to the nearestGoalState is found
            return;

        state_t *thisState = nearestGoalState;
        while (thisState->parentState != NULL){
            pathList.insert(pathList.begin(), thisState);
            thisState = thisState->parentState;
        }
        pathList.insert(pathList.begin(), robotState); // add current robot state

        smoothPath();
        // free memory
        // cout << ros::Time::now().toSec() - start_time << endl;
    }



    state_t* minCostIndexAstar(){

        state_t* minState;
        float minCost = FLT_MAX;
        for (int i = 0; i < openSet.size(); ++i){
            if (openSet[i]->costsToGo[2] < minCost){
                minCost = openSet[i]->costsToGo[2];
                minState = openSet[i];
            }
        }
        return minState;
    }

    void smoothPath(){

        if (pathList.size() <= 1)
            return;
        // Cubic Spline
        nav_msgs::Path originPath;
        nav_msgs::Path splinePath;
        
        originPath.header.frame_id = "map";
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";

        originPath.poses.clear();

        for (int i = 0; i < pathList.size(); i++){
            pose.pose.position.x = pathList[i]->x[0];
            pose.pose.position.y = pathList[i]->x[1];
            pose.pose.position.z = pathList[i]->x[2];
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
            originPath.poses.push_back(pose);
        }

        path_smoothing::CubicSplineInterpolator csi("smooth");
        csi.interpolatePath(originPath, splinePath);

        globalPath = splinePath;
    }    

    void publishPathStop(){
        // even no feasible path is found, publish an empty path
        
        globalPath.header.frame_id = "map";
        globalPath.header.stamp = ros::Time::now();

        for (int i = 0; i < globalPath.poses.size(); ++i){
            globalPath.poses[i].pose.position.z = robotState->x[2] + 1.5;
        }
        
        // publish path
        pubGlobalPath.publish(globalPath);
        // stop planning
        if (globalPath.poses.size() > 0 &&
            sqrt((goalState->x[0]-robotState->x[0])*(goalState->x[0]-robotState->x[0]) + (goalState->x[1]-robotState->x[1])*(goalState->x[1]-robotState->x[1])) <= 2.0)
            planningFlag = false;
    }

    void pubLishLocalGoal(){

        if (globalPath.poses.size() < 20)
            return;

        geometry_msgs::PoseStamped localGoal;
        localGoal.header.frame_id = "map";
        localGoal.header.stamp = ros::Time::now();
        localGoal = globalPath.poses[19];
        pubLocalGoal.publish(localGoal);
    }





};






int main(int argc, char** argv){

    ros::init(argc, argv, "traversability_exploration");
    
    TraversabilityExploration TE;
   
    ROS_INFO("\033[1;32m---->\033[0m Traversability Exploration Planner %s.", 
            explorationPlanningFlag == true ? "\033[1;31mEnabled\033[0m" : "\033[1;31mDisabled\033[0m");

    ros::spin();

    return 0;
}
