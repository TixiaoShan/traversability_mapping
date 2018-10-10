#include "utility.h"

// RRT* Planner Settings
extern const double turningRadius = 0.2; // turning radius of Dubins vehicle
extern const double runTime = 0.1; // run time of rrt* planner (seconds)
extern const double goalBias = 0.05; // pobability of sampling goal state 0~1
extern const double goalRegionRadius = 1.0;
extern const bool switchRootFlag = true;

class TraversabilityRRTstar {

private:
	ros::NodeHandle nh;
	ros::Subscriber subElevationMap; // 2d local height map from mapping package
	ros::Subscriber subCostMap; // cost map from move_base
	ros::Subscriber subLaserOdometry; // get robot position
	ros::Subscriber subGlobalPath; // path from prm package
	ros::Publisher pubRRTtree;
	ros::Publisher pubRRTpath;
	ros::Publisher pubLocalPath;

	

	kdtree_t *kdtree;

    state_t *robotState;

    state_t* root;
	state_t* goal;

    elevation_msgs::OccupancyElevation elevationMap; // this is received from mapping package. it is a 2d local map that includes height info
    nav_msgs::OccupancyGrid costMap; // this map is received from move_base. the obstacles are inflated

    double start_time;
    double finish_time;

	vector<state_t*> nodeList;
	vector<state_t*> nodeListBackup; // for switching root
	vector<state_t*> pathList;
	vector<state_t> dubinsPath;

	float map_min[3]; // 0 - x, 1 - y, 2 - z
	float map_max[3];

	double gamma;
	double ballRadius;

	bool mapReceivedFlag;

public:

	TraversabilityRRTstar():
	nh("~"),
	mapReceivedFlag(false){

		kdtree = NULL;
		
		robotState = new state_t;
		goal = new state_t;

		subElevationMap = nh.subscribe<elevation_msgs::OccupancyElevation>("/occupancy_map_local_height", 1, &TraversabilityRRTstar::elevationMapHandler, this);
		subCostMap = nh.subscribe<nav_msgs::OccupancyGrid>("/move_base/global_costmap/costmap", 1, &TraversabilityRRTstar::costMapHandler, this);

		subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/integrated_to_init", 5, &TraversabilityRRTstar::robotPosHandler, this);
		subGlobalPath = nh.subscribe<geometry_msgs::PoseArray>("/global_path", 5, &TraversabilityRRTstar::localGoalHandler, this);

		pubRRTtree = nh.advertise<visualization_msgs::Marker>("rrt_star_tree", 1);
		pubRRTpath = nh.advertise<visualization_msgs::Marker>("rrt_star_path", 1);

		pubLocalPath = nh.advertise<geometry_msgs::PoseArray>("/local_path", 1);

	}

	~TraversabilityRRTstar(){}


	void localGoalHandler(const geometry_msgs::PoseArray::ConstPtr& pathMsg){
		// 0. No valid path is received from global PRM planner
		if (pathMsg->poses.size() == 0 || mapReceivedFlag == false)
			return;
		// 1. free all memory
		freeMemory();
		// 2. update start and goal position before planning
		setStartandGoal(pathMsg);
		// 3. add useful states from previous planning
		addUsefulStates();
		// 4. rrt* iteration
		start_time = ros::Time::now().toSec();
		while (ros::Time::now().toSec() - start_time < runTime && ros::ok())
			rrtIteration();
		// 5. retrive path from rrt* tree
		retrivePath();
		// 6. Save useful states for next planning
		saveUsefulStates();
		// 7. visualize extracted path and rrt* tree
		publishRRT();
		// 8. publish path to move_base in pose array format
		publishPath();
		

		// finish_time = ros::Time::now().toSec();
		// cout << finish_time - start_time << "s." << endl;
	}
	




	void setStartandGoal(const geometry_msgs::PoseArray::ConstPtr& pathMsg){

		root = new state_t;
		
		// 0. Cauculation of ball radius parameter
		double map_squre = (map_max[1]-map_min[1]) * (map_max[0]-map_min[0]);
		gamma = pow(2*(1+1/2.0), 1/2.0) * pow(map_squre/4.18667, 1/2.0) / 2;
	    
		// 1. Initialize and push root into nodeList
		for (int i = 0; i < 2; ++i)
			root->x[i] = robotState->x[i];
		root->x[2] = getStateHeight(root->x);
		root->theta = robotState->theta;
		// 2. Set start cost to zero
		for (int i = 0; i < NUM_COSTS; ++i){
			root->costsToRoot[i] = 0;
			root->costsToParent[i] = 0;
		}
		// 3. Save start state
		root->childList.clear();
		nodeList.push_back(root);
		// 4. Insert root into KD-tree
	    insertIntoKdtree(nodeList.back());
	    // 5. Find goal state in the local map
	    int index = 0;
	    for (int i = 0; i < pathMsg->poses.size(); ++i){
			if (pathMsg->poses[i].position.x <= map_min[0] || pathMsg->poses[i].position.x >= map_max[0] ||
				pathMsg->poses[i].position.y <= map_min[1] || pathMsg->poses[i].position.y >= map_max[1])
				break;
			index = i;
	    }
	    // cout << pathMsg->poses[index].position.x << " " << pathMsg->poses[index].position.y << " " << pathMsg->poses[index].position.z << endl;
	    goal->x[0] = pathMsg->poses[index].position.x;
		goal->x[1] = pathMsg->poses[index].position.y;
		goal->x[2] = pathMsg->poses[index].position.z - sensorHeight;
		goal->theta = root->theta;
	}


	void elevationMapHandler(const elevation_msgs::OccupancyElevation::ConstPtr& mapMsg){
		elevationMap = *mapMsg;

		map_min[0] = elevationMap.occupancy.info.origin.position.x; 
		map_min[1] = elevationMap.occupancy.info.origin.position.y;
		map_min[2] = elevationMap.occupancy.info.origin.position.z;
		map_max[0] = elevationMap.occupancy.info.origin.position.x + elevationMap.occupancy.info.resolution * elevationMap.occupancy.info.width; 
		map_max[1] = elevationMap.occupancy.info.origin.position.y + elevationMap.occupancy.info.resolution * elevationMap.occupancy.info.height; 
		map_max[2] = elevationMap.occupancy.info.origin.position.z;
	}

	void costMapHandler(const nav_msgs::OccupancyGrid::ConstPtr& mapMsg){
		costMap = *mapMsg;
		mapReceivedFlag = true;
	}



	void freeMemory(){
		// 1. Free memory
		// delete every node in the tree
		for (int i = 0; i < nodeList.size(); ++i)
			delete nodeList[i];
		nodeList.clear();
		vector<state_t*>().swap(nodeList);
		pathList.clear();
		vector<state_t*>().swap(pathList);
		dubinsPath.clear();
		vector<state_t>().swap(dubinsPath);

		// 2. Free and create a new kdtree
		if (kdtree){
			kd_clear(kdtree);
		    kd_free(kdtree);
		    kdtree = NULL;
		}
		kdtree = kd_create(3);
	}


	void robotPosHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry){
        robotState->x[0] = laserOdometry->pose.pose.position.z; // change coordinates from LOAM
        robotState->x[1] = laserOdometry->pose.pose.position.x;
        robotState->x[2] = laserOdometry->pose.pose.position.y;
        robotState->theta = laserOdometry->pose.pose.orientation.y * M_PI;
    }

    // Calculate the Euclidean distance between two samples (2D)
	double distance(double state_from[3], double state_to[3]){
		return sqrt((state_to[0]-state_from[0])*(state_to[0]-state_from[0]) + 
			(state_to[1]-state_from[1])*(state_to[1]-state_from[1]));
	}


    void insertIntoKdtree(state_t *stateCurr){
	  	kd_insert(kdtree, stateCurr->x, stateCurr);
	}

	// Collision check
	bool isIncollision(double state[3]){
	    // if the distance to the nearest obstacle is less than xxx, in collision
	    int rounded_x = (int)((state[0] - map_min[0]) / mapResolution);
	    int rounded_y = (int)((state[1] - map_min[1]) / mapResolution);
	    int index = rounded_x + rounded_y * elevationMap.occupancy.info.width;
	    if (costMap.data[index] != 0)
	        return true;
	    // if the state is outside the map, discard this state
	    if (state[0] < map_min[0] || state[0] > map_max[0] 
			|| state[1] < map_min[1] || state[1] > map_max[1])
			return true;
	    return false;
	}
	// Sample a new state
	int sampleState(state_t *stateCurr){

		// random x and y
		for (int i = 0; i < 2; ++i)
			stateCurr->x[i] = (double)rand()/(RAND_MAX + 1.0)*(map_max[i] - map_min[i]) 
	        	- (map_max[i] - map_min[i])/2.0 + (map_max[i] + map_min[i])/2.0;
		// random heading
		stateCurr->theta = (double)rand()/(RAND_MAX + 1.0) * 2 * M_PI - M_PI;
		// collision checking before getting height info
		if (isIncollision(stateCurr->x))
	        return 0;
	    // z is not random since the robot is constrained to move on the ground
		stateCurr->x[2] = getStateHeight(stateCurr->x);
		if (stateCurr->x[2] == -FLT_MAX)
			return 0;

		return 1;
	}

	// Sample a new state
	int sampleGoalState(state_t *stateCurr){
		for (int i = 0; i < 2; ++i)
			stateCurr->x[i] = (double)rand()/(RAND_MAX + 1.0)* goalRegionRadius*2 
	        	- goalRegionRadius + goal->x[i];
    	// collision checking before getting height info
	    if (isIncollision(stateCurr->x))
	        return 0;
		stateCurr->x[2] = getStateHeight(stateCurr->x);
		if (stateCurr->x[2] == -FLT_MAX)
			return 0;
		// stateCurr->theta = goal->theta;		
		stateCurr->theta = (double)rand()/(RAND_MAX + 1.0) * 2 * M_PI - M_PI;
		return 1;
	}

	double getStateHeight(double state[3]){
		int rounded_x = (int)((state[0] - map_min[0]) / mapResolution);
	    int rounded_y = (int)((state[1] - map_min[1]) / mapResolution);
	    return elevationMap.height[rounded_x + rounded_y * elevationMap.occupancy.info.width];
	}

	//
	state_t* getNearestStateIdx(state_t *stateIn){
	  	kdres_t *kdres = kd_nearest(kdtree, stateIn->x);
	  	if (kd_res_end (kdres))  {
	  		kd_res_free (kdres);
	    	return NULL;
	  	}
	  	state_t* nearestState = (state_t*) kd_res_item_data(kdres);
	  	kd_res_free (kdres);
	  	return nearestState;
	}
	//
	void getNearStates(state_t *stateIn, vector<state_t*>& vectorNearStatesOut, double radius){
		kdres_t *kdres = kd_nearest_range (kdtree, stateIn->x, radius);
		// Create the vector data structure for storing the results
		int numNearVertices = kd_res_size (kdres);
		if (numNearVertices == 0) {
			vectorNearStatesOut.clear();
			kd_res_free (kdres);
			return;
		}

		vectorNearStatesOut.clear();
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

	bool edgePropagation(state_t *from, state_t *to, float edgeCosts[NUM_COSTS]){
		// initialize edgeCosts
		for (int i = 0; i < NUM_COSTS; ++i)
			edgeCosts[i] = 0;

		// Creat this Dubins edge
		double q_from[3] = {from->x[0], from->x[1], from->theta};
		double q_to[3] = {to->x[0], to->x[1], to->theta};

		DubinsPath path;
		dubins_init(q_from, q_to, turningRadius, &path);
		double x = mapResolution;
		double length = dubins_path_length(&path);
		double q[3];
		// A loop for dubins curve propagation
		while( x <= length ) {
			// Get current state
	        dubins_path_sample(&path, x, q);

	        if (isIncollision(q))
				return false;
			// costs progagation
	        // edgeCosts[0] = 0; //
	        // edgeCosts[1] = 0; //
	        edgeCosts[2] = edgeCosts[2] + mapResolution; // distance cost
	        // edgeCosts[3] = 0; //

			x += mapResolution;
	    }
	    return true;
	}
	// find best parent for new state from it's neighbors
	state_t* findBestParent(state_t *stateRandom, vector<state_t*> vectorNearStatesIn, float edgeCosts[NUM_COSTS]){
		double bestCosts[NUM_COSTS];
		for (int i = 0; i < NUM_COSTS; ++i)
			bestCosts[i] = DBL_MAX;

		state_t *bestParent = NULL;
		float tempEdgeCosts[NUM_COSTS];

		// Loop through cost hierarchy
		for (vector<int>::const_iterator iter1 = costHierarchy.begin(); iter1 != costHierarchy.end(); iter1++){

			int costIndex = *iter1;
			vector<state_t*> tempXmin;
			// loop through all near states
			for (vector<state_t*>::iterator iter = vectorNearStatesIn.begin(); iter != vectorNearStatesIn.end(); iter++){
				state_t* thisState = *iter;
				
				if (edgePropagation(thisState, stateRandom, tempEdgeCosts) == false)
					continue;
				// potential connection can offer lower cost, create a tempXmin to save it
				if (tempEdgeCosts[costIndex] + thisState->costsToRoot[costIndex] < bestCosts[costIndex]){
					for (int i = 0; i < NUM_COSTS; ++i){
						bestCosts[i] = tempEdgeCosts[i] + thisState->costsToRoot[costIndex];
						edgeCosts[i] = tempEdgeCosts[i];
					}

					bestParent = thisState;
					tempXmin.clear();
					tempXmin.push_back(thisState);
				}
				// connection has same cost, push this state into a vector 
				// and then use lower cost hierarchy to break ties till best parent is found
				else if (tempEdgeCosts[costIndex] + thisState->costsToRoot[costIndex] == bestCosts[costIndex])
					tempXmin.push_back(thisState);
			}
			// 
			// vectorNearStatesIn is used again for next loop
			vectorNearStatesIn.clear();
			vectorNearStatesIn = tempXmin;
		}

		return bestParent;
	}
	void rewireNearStates(state_t *stateFrom, vector<state_t*> vectorNearStatesIn){

		for (vector<state_t*>::iterator iter = vectorNearStatesIn.begin(); iter != vectorNearStatesIn.end(); iter++){
			state_t *stateTo = *iter;
			// if this state that is going to be rewired is new state's parent or belong to the path to new state, skip
			if (stateFrom->parentState == stateTo
				|| belongToPreviousPath(stateFrom, stateTo) == 1)
				continue;
			// calculate costs for this potential connection
			float tempEdgeCosts[NUM_COSTS];
			if (edgePropagation(stateFrom, stateTo, tempEdgeCosts) == false)
				continue;

			// Loop through cost hierarchy
			for (vector<int>::const_iterator iter1 = costHierarchy.begin(); iter1 != costHierarchy.end(); iter1++){
				int costIndex = *iter1;
				// cost can be lowered, update costs use this new connection
				if (tempEdgeCosts[costIndex] + stateFrom->costsToRoot[costIndex] < stateTo->costsToRoot[costIndex]){
					updateCost(stateFrom, stateTo, tempEdgeCosts);
					break;
				}
				// if same cost got, go to next cost hierarchy to break ties
				else if (tempEdgeCosts[costIndex] + stateFrom->costsToRoot[costIndex]  == stateTo->costsToRoot[costIndex])
					continue;
				// cost is increased, not optimal connection, break
				else
					break;
			}
		}
	}

	int belongToPreviousPath(state_t *stateFrom, state_t *stateTo){
		// while(fromIdx != 0){
		// 	int temp_p = nodeList[fromIdx]->parent;
		// 	if (temp_p == toIdx)
		// 		return 1;
		// 	fromIdx = temp_p;
		// }
		return 0;
	}

	void updateCost(state_t *stateFrom, state_t *stateTo, float edgeCosts[NUM_COSTS]){
		// Attention, should update states saved in nodeList!!!
		// Update costs
		for (int i = 0; i < NUM_COSTS; ++i){
			stateTo->costsToRoot[i] = stateFrom->costsToRoot[i] + edgeCosts[i];
			stateTo->costsToParent[i] = edgeCosts[i];
		}
		// delete this stateTo from its old parent
		state_t *oldParent = stateTo->parentState;
		oldParent->childList.erase(
			std::remove(oldParent->childList.begin(), oldParent->childList.end(), stateTo), 
			oldParent->childList.end()	);
		// Change parent from old parent to new parent
		stateTo->parentState = stateFrom;
		// add this near state to the child list of new state
		stateFrom->childList.push_back(stateTo);
		// Update costs for this near state's all children
		updateChildrenCosts(stateTo);
	}

	void updateChildrenCosts(state_t *stateParent){
		// loop through all children of stateParent
		for (vector<state_t*>::iterator iter = stateParent->childList.begin(); iter != stateParent->childList.end(); iter++){
			state_t *stateChild = *iter;
			// update this child's costs
			for (int i = 0; i < NUM_COSTS; ++i)
				stateChild->costsToRoot[i] = stateParent->costsToRoot[i] + stateChild->costsToParent[i];
			// update all children's costs for this child (recursive)
			updateChildrenCosts(stateChild);
		}
	}
	//
	state_t* insertTree(state_t *stateNew, state_t *parentState, float edgeCosts[NUM_COSTS]){
		// update costs for this new state in tree
		for (int i = 0; i < NUM_COSTS; ++i){
			stateNew->costsToRoot[i] = parentState->costsToRoot[i] + edgeCosts[i];
			stateNew->costsToParent[i] = edgeCosts[i];
		}
		// Update parent and children index
		stateNew->parentState = parentState;
		parentState->childList.push_back(stateNew);

	    // add new state to the tree
		nodeList.push_back(stateNew); // only push when everything is updated
		// insert new state into KD-tree
		insertIntoKdtree(nodeList.back());

		return stateNew;
	}

	void retrivePath(){
		// no valid nodes in tree
		if (nodeList.size() <= 1)
			return;
		// find all states in goal region
		vector<state_t*> goalNearStates;
		getNearStates(goal, goalNearStates, goalRegionRadius);

		state_t* bestState;

		if (goalNearStates.size() == 0)
			// If no near states within radius, choose the nearest state in the tree
			bestState = getNearestStateIdx(goal);
		else{
			// find the state in goal region that can offer the lowest distance cost
			float shortestDist = DBL_MAX;
			for (vector<state_t*>::iterator iter = goalNearStates.begin(); iter != goalNearStates.end(); iter++){
				state_t* thisState = *iter;
				if (thisState->costsToRoot[2] < shortestDist){
					shortestDist = thisState->costsToRoot[2];
					bestState = thisState;
				}
			}
		}
		// creat path
		pathList.clear();
		vector<state_t*>().swap(pathList);

		while (bestState->parentState != NULL){
			pathList.insert(pathList.begin(), bestState);
			bestState = bestState->parentState;
		}

		pathList.insert(pathList.begin(), root);
	}

	void publishRRT(){
		// Publish Rviz Tree
		visualization_msgs::Marker line_list, path_list, points;
		points.header.frame_id = path_list.header.frame_id = line_list.header.frame_id = "/map";
		points.header.stamp = path_list.header.stamp = line_list.header.stamp = ros::Time::now();
		points.ns = path_list.ns = line_list.ns = "points_and_lines";
		points.action = path_list.action = line_list.action = visualization_msgs::Marker::ADD;
		points.pose.orientation.w = path_list.pose.orientation.w = line_list.pose.orientation.w = 1.0;
		points.id = 0; path_list.id = 1; line_list.id = 2;
		points.type = visualization_msgs::Marker::SPHERE_LIST;
		path_list.type = visualization_msgs::Marker::LINE_STRIP;
		line_list.type = visualization_msgs::Marker::LINE_LIST;
		geometry_msgs::Point p;
		std_msgs::ColorRGBA c;

		///////////////////////////Dispaly single-source Dubins path////////////////////////////
		if (pubRRTtree.getNumSubscribers() != 0){
			line_list.scale.x = 0.05;
			line_list.color.r = 0; line_list.color.g = 1.0; line_list.color.b = 1.0;
			line_list.color.a = 1.0;

			int numNodes = nodeList.size();
			double heightDiff;
			for (int i = 0; i < numNodes; ++i){
				state_t* to = nodeList[i];
				state_t* from = nodeList[i]->parentState;

				if (from == NULL)
					continue;

				double q_from[3] = {from->x[0], from->x[1], from->theta};
				double q_to[3] = {to->x[0], to->x[1], to->theta};
				heightDiff = to->x[2] - from->x[2];

				DubinsPath path;
				dubins_init(q_from, q_to, turningRadius, &path);
				double q[3];
				double length = dubins_path_length(&path);

				p.x = from->x[0]; p.y = from->x[1]; p.z = from->x[2]+0.1;
				for (double x = 0; x < length; x += mapResolution){
					line_list.points.push_back(p);
					dubins_path_sample(&path, x, q);
					p.x = q[0]; p.y = q[1]; p.z = (from->x[2]+x/length*heightDiff) + 0.15;//getStateHeight(q) + 0.1;
					line_list.points.push_back(p);
				}
			}
			pubRRTtree.publish(line_list);
		}

		//////////////////////////////Extract Dubins Path to dubinsPath ////////////////////////////////
		dubinsPath.clear();
		vector<state_t>().swap(dubinsPath);
		path_list.scale.x = 0.2;
		path_list.color.r = 1.0; path_list.color.g = 0; path_list.color.b = 0;
		path_list.color.a = 1.0;

		int pathListLength = pathList.size();
		if (pathListLength == 0)
			return;
		for (int i = 0; i < pathListLength - 1; ++i){
			double q_from[3] = {pathList[i]->x[0], pathList[i]->x[1], pathList[i]->theta};
			double q_to[3] = {pathList[i+1]->x[0], pathList[i+1]->x[1], pathList[i+1]->theta};

			DubinsPath path;
			dubins_init(q_from, q_to, turningRadius, &path);
			double x = 0; double q[3];
			double length = dubins_path_length(&path);
			double heightDiff = pathList[i+1]->x[2] - pathList[i]->x[2];

			while( x < length ) {
				dubins_path_sample(&path, x, q);
				state_t this_state;
				this_state.x[0] = q[0];	this_state.x[1] = q[1];	this_state.x[2] = (pathList[i]->x[2]+x/length*heightDiff) + 0.15;//getStateHeight(this_state.x);
				this_state.theta = q[2];

				dubinsPath.push_back(this_state);

				p.x = this_state.x[0];	p.y = this_state.x[1];	p.z = this_state.x[2]+0.15;
				path_list.points.push_back(p);

				x += mapResolution;
			}
		}
		pubRRTpath.publish(path_list);
	}

	void publishPath(){
        geometry_msgs::PoseArray localPath;
        localPath.header.frame_id = "/map";
        localPath.header.stamp = ros::Time::now();

        int numPathStates = dubinsPath.size();
        if (numPathStates <= 1)
        	return;
        geometry_msgs::Pose thisPose;
        for (int i = numPathStates - 1; i >= 0; --i){
            thisPose.position.x = dubinsPath[i].x[0];
            thisPose.position.y = dubinsPath[i].x[1];
            thisPose.position.z = dubinsPath[i].x[2]+0.15;
            localPath.poses.push_back(thisPose);
        }
        pubLocalPath.publish(localPath);
    }

    //////////////////////////////////////// switch root ////////////////////////////////////////////////////
	void addUsefulStates(){
		if (switchRootFlag == false)
			return;
		if (nodeListBackup.size() == 0)
			return;
		// add states in nodeListBackup to nodeList
		state_t* stateAfterRoot = insertUsefulStates(nodeListBackup.front(), nodeList);
		if (stateAfterRoot == NULL){
			for (int i = 1; i < nodeList.size(); ++i)
				delete nodeList[i];
			nodeList.resize(1);
			return;
		}
		// update costs for the newly inserted states
		float edgeCosts[NUM_COSTS];
		if (edgePropagation(root, stateAfterRoot, edgeCosts) == false){
			for (int i = 1; i < nodeList.size(); ++i)
				delete nodeList[i];
			nodeList.resize(1);
			return;
		}
		for (int i = 0; i < NUM_COSTS; ++i){
			stateAfterRoot->costsToRoot[i] = root->costsToRoot[i] + edgeCosts[i];
			stateAfterRoot->costsToParent[i] = edgeCosts[i];
		}
		// link states
		root->childList.push_back(stateAfterRoot);
		stateAfterRoot->parentState = root;
		// update all children costs
		updateChildrenCosts(stateAfterRoot);
		// add useful states into KD-tree
		for (int i = 1; i < nodeList.size(); ++i) // i = 1, root is added already
			insertIntoKdtree(nodeList[i]);
	}
	// mark useful states
	void saveUsefulStates(){
		if (switchRootFlag == false)
			return;
		// no need to save states since the robot is next to goal
		if (pathList.size() <= 2)
			return;
		// free memory
		for (int i = 0; i < nodeListBackup.size(); ++i)
			delete nodeListBackup[i];
		nodeListBackup.clear();
		vector<state_t*>().swap(nodeListBackup);
		// save useful states recursively
		insertUsefulStates(pathList[1], nodeListBackup);
	}
	// recursive insert useful states into backup list
	state_t* insertUsefulStates(state_t* stateIn, vector<state_t*>& listType){

		if (stateIn->x[0] < map_min[0] || stateIn->x[0] > map_max[0] 
			|| stateIn->x[1] < map_min[1] || stateIn->x[1] > map_max[1])
			return NULL;
		state_t *thisState = new state_t(stateIn);
		listType.push_back(thisState);

		for (int i = 0; i < stateIn->childList.size(); ++i){
			
			state_t* pointerToNewChild = insertUsefulStates(stateIn->childList[i], listType);
			if (pointerToNewChild == NULL)
				continue;
			pointerToNewChild->parentState = thisState;
			thisState->childList.push_back(pointerToNewChild);
		}
		return thisState;
	}
	//////////////////////////////////////// switch root ////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////    ///    ///////////////////////////////////////////////////////////////////////////////////////////
	/////    ///    ///////                                                ////////////////////////////////////
	//       ///       ////               RRT Iteration Start              ////////////////////////////////////
	// ///   ///   ///  ///                                                ////////////////////////////////////
	///  //  ///  ///  ////////////////////////////////////////////////////////////////////////////////////////
	////     ///     //////////////////////////////////////////////////////////////////////////////////////////
	// RRT iteration (each iteration tries to add a new sample state)
	void rrtIteration(){
		// 1. Sample a new state
		state_t* stateRandom = new state_t;

		if (((double)rand())/(RAND_MAX + 1.0) < goalBias) {
			 // 1.1 sample in goal region
			if (sampleGoalState(stateRandom) == 0)
				{delete stateRandom; return;}
		}
		else{// 1.2 regular random sample
			if (sampleState(stateRandom) == 0)
				{delete stateRandom; return;}
		}
		// 2. Find near nodes
		ballRadius = gamma * pow( log((double)(nodeList.size() + 1.0))/((double)(nodeList.size() + 1.0)), 1.0/3.0 );
		vector<state_t*> vectorNearStates;
		getNearStates(stateRandom, vectorNearStates, ballRadius);
		// 3. Find the best parent and extend from that parent
		state_t *bestState;
		float edgeCosts[NUM_COSTS];

		if (vectorNearStates.size() == 0) {
			// 3.1 Find the nearest state parent if there are no near states
			bestState = getNearestStateIdx(stateRandom);

			if (bestState == NULL)
				{delete stateRandom; return;}

			if (edgePropagation(bestState, stateRandom, edgeCosts) == false)
				{delete stateRandom; return;}

		}
		else{
			// 3.2 Find the best parent among states
			bestState = findBestParent(stateRandom, vectorNearStates, edgeCosts);
			if (bestState == NULL)
				{delete stateRandom; return;}
		}
		
		// 3.3 insert new state into tree and KD-tree
		state_t *stateNew =  insertTree(stateRandom, bestState, edgeCosts);

		// 4. Rewire neighbors
		if (vectorNearStates.size() > 0)
			rewireNearStates(stateNew, vectorNearStates);
	}

	///////   //   ////////////////////////////////////////////////////////////////////////////////////////////
	/////   //////   //////////////////////////////////////////////////////////////////////////////////////////
	///  //  ///  //  /////                                                ////////////////////////////////////
	//       //       /////                RRT Iteration End               ////////////////////////////////////
	///////  //  //////////                                                ////////////////////////////////////
	///////  //  //////////////////////////////////////////////////////////////////////////////////////////////
	///////  //  //////////////////////////////////////////////////////////////////////////////////////////////
};







int main(int argc, char** argv){

	ros::init(argc, argv, "traversability_map");

	TraversabilityRRTstar TRRTstar;

	ROS_INFO("\033[1;32m---->\033[0m Traversability RRT* Local Planner Started.");

    ros::spin();

	return 0;
}