#include "utility.h"

#include "elevation_msgs/OccupancyElevation.h"


class TMPRM{
private:

    ros::NodeHandle nh;

    tf::TransformListener listener;
    tf::StampedTransform transform;

    ros::Subscriber subGoal;
    
    ros::Publisher pubPRMGraph;
    ros::Publisher pubPRMPath;
    ros::Publisher pubGlobalPath;
    ros::Publisher pubSingleSourcePaths;

    ros::Publisher pubCloudPRMNodes;
    ros::Publisher pubCloudPRMGraph;

    ros::Subscriber subElevationMap;

    elevation_msgs::OccupancyElevation elevationMap;

    float map_min[3];
    float map_max[3];
 
    vector<state_t*> nodeList;
    vector<state_t*> pathList;
    
    nav_msgs::Path globalPath;
    nav_msgs::Path displayGlobalPath;

    double start_time;
    double finish_time;

    bool planningFlag;

    state_t *robotState;
    state_t *goalState;
    state_t *nearestGoalState;
    state_t *mapCenter;

    kdtree_t *kdtree;

    bool costUpdateFlag[NUM_COSTS];

    std::mutex mtx;

public:
    TMPRM():
        nh("~"),
        planningFlag(false){

        robotState = new state_t;
        goalState = new state_t;
        mapCenter = new state_t;

        subGoal = nh.subscribe<geometry_msgs::PoseStamped>("/prm_goal", 1, &TMPRM::goalPosHandler, this);
        subElevationMap = nh.subscribe<elevation_msgs::OccupancyElevation>("/occupancy_map_local_height", 1, &TMPRM::elevationMapHandler, this);     

        pubPRMGraph = nh.advertise<visualization_msgs::MarkerArray>("/prm_graph", 1);
        pubPRMPath = nh.advertise<visualization_msgs::MarkerArray>("/prm_path", 1);
        pubSingleSourcePaths = nh.advertise<visualization_msgs::MarkerArray>("/prm_single_source_paths", 1);

        pubCloudPRMNodes = nh.advertise<sensor_msgs::PointCloud2>("/prm_cloud_nodes", 1);
        pubCloudPRMGraph = nh.advertise<sensor_msgs::PointCloud2>("/prm_cloud_graph", 1);

        pubGlobalPath = nh.advertise<nav_msgs::Path>("/global_path", 1);

        allocateMemory(); 
    }

    ~TMPRM(){}

    void allocateMemory(){

        kdtree = kd_create(3);

        for (int i = 0; i < NUM_COSTS; ++i)
            costUpdateFlag[i] = false;
        for (int i = 0; i < costHierarchy.size(); ++i)
            costUpdateFlag[costHierarchy[i]] = true;
    }

    void elevationMapHandler(const elevation_msgs::OccupancyElevation::ConstPtr& mapMsg){

        std::lock_guard<std::mutex> lock(mtx);

        elevationMap = *mapMsg;

        updateMapBoundary();

        updateCostMap();

        buildRoadMap();
    }

    void updateMapBoundary(){
        map_min[0] = elevationMap.occupancy.info.origin.position.x; 
        map_min[1] = elevationMap.occupancy.info.origin.position.y;
        map_min[2] = elevationMap.occupancy.info.origin.position.z;
        map_max[0] = elevationMap.occupancy.info.origin.position.x + elevationMap.occupancy.info.resolution * elevationMap.occupancy.info.width; 
        map_max[1] = elevationMap.occupancy.info.origin.position.y + elevationMap.occupancy.info.resolution * elevationMap.occupancy.info.height; 
        map_max[2] = elevationMap.occupancy.info.origin.position.z;
    }

    void updateCostMap(){
        int sizeMap = elevationMap.occupancy.data.size();
        int inflationSize = int(costmapInflationRadius / elevationMap.occupancy.info.resolution);
        for (int i = 0; i < sizeMap; ++i) {
            int idX = int(i % elevationMap.occupancy.info.width);
            int idY = int(i / elevationMap.occupancy.info.width);
            if (elevationMap.occupancy.data[i] > 0){
                for (int m = -inflationSize; m <= inflationSize; ++m) {
                    for (int n = -inflationSize; n <= inflationSize; ++n) {
                        int newIdX = idX + m;
                        int newIdY = idY + n;
                        if (newIdX < 0 || newIdX >= elevationMap.occupancy.info.width || newIdY < 0 || newIdY >= elevationMap.occupancy.info.height)
                            continue;
                        int index = newIdX + newIdY * elevationMap.occupancy.info.width;
                        elevationMap.costMap[index] = std::max(elevationMap.costMap[index], std::sqrt(float(m*m+n*n)));
                    }
                }
            }
        }
    }

    void buildRoadMap(){

        generateSamples();

        updateStatesAndEdges();   

        bfsSearch();

        publishPRM();

        publishPathStop();

        publishRoadmap2Cloud();
    }

    void goalPosHandler(const geometry_msgs::PoseStampedConstPtr& goal){
    	
        goalState->x[0] = goal->pose.position.x;
        goalState->x[1] = goal->pose.position.y;
        goalState->x[2] = goal->pose.position.z;

        if (goalState->x[0] == -FLT_MAX){
            displayGlobalPath.poses.clear();
            return;
        }

        nearestGoalState = getNearestState(goalState);

        if (nearestGoalState == NULL || nearestGoalState->neighborList.size() == 0)
            return;

        planningFlag = true;
    }

    
    
    bool bfsSearch(){

        pathList.clear();
        globalPath.poses.clear();

        if (planningFlag == false)
            return true;

        for (int i = 0; i < nodeList.size(); ++i){
            for (int j = 0; j < NUM_COSTS; ++j)
                nodeList[i]->costsToRoot[j] = FLT_MAX;
            nodeList[i]->parentState = NULL;
        }

        state_t *startState;

        vector<state_t*> nearRobotStates;
        getNearStates(robotState, nearRobotStates, 2);
        if (nearRobotStates.size() == 0)
            return false;

        float nearRobotDist = FLT_MAX;
        for (int i = 0; i < nearRobotStates.size(); ++i){
            if (distance(nearRobotStates[i]->x, robotState->x) < nearRobotDist 
                && nearRobotStates[i]->neighborList.size() != 0){
                nearRobotDist = distance(nearRobotStates[i]->x, robotState->x);
                startState = nearRobotStates[i];
            }
        }

        for (int i = 0; i < NUM_COSTS; ++i)
            startState->costsToRoot[i] = 0;

        float thisCost;
        vector<state_t*> Queue;
        Queue.push_back(startState);

        while(Queue.size() > 0 && ros::ok()){

            state_t *fromState = minCostStateInQueue(Queue);
            Queue.erase(remove(Queue.begin(), Queue.end(), fromState), Queue.end());

            if (fromState == nearestGoalState)
                break;

            for (int i = 0; i < fromState->neighborList.size(); ++i){
                state_t *toState = fromState->neighborList[i].neighbor;

                for (vector<int>::const_iterator iter = costHierarchy.begin(); iter != costHierarchy.end(); iter++){
                    int costIndex = *iter;

                    thisCost = fromState->costsToRoot[costIndex] + fromState->neighborList[i].edgeCosts[costIndex];

                    if (thisCost < toState->costsToRoot[costIndex]){
                        updateCosts(fromState, toState, i);
                        toState->parentState = fromState;
                        Queue.push_back(toState);
                    }
                    else if (thisCost == toState->costsToRoot[costIndex]){
                        continue;
                    }
                    else
                        break;
                }
            }
        }

        if (nearestGoalState->parentState == NULL)
            return false;

        state_t *thisState = nearestGoalState;
        while (thisState->parentState != NULL){
            pathList.insert(pathList.begin(), thisState);
            thisState = thisState->parentState;
        }
        pathList.insert(pathList.begin(), robotState);

        smoothPath();

        return true;
    }

    state_t* minCostStateInQueue(vector<state_t*> Queue){

        for (vector<int>::const_iterator iter1 = costHierarchy.begin(); iter1 != costHierarchy.end(); ++iter1){
            int costIndex = *iter1;
            vector<state_t*> tempQueue;
            float minCost = FLT_MAX;

            for (vector<state_t*>::const_iterator iter2 = Queue.begin(); iter2 != Queue.end(); ++iter2){
                state_t* thisState = *iter2;

                if (thisState->costsToRoot[costIndex] < minCost){
                    minCost = thisState->costsToRoot[costIndex];
                    tempQueue.clear();
                    tempQueue.push_back(thisState);
                }

                else if (thisState->costsToRoot[costIndex] == minCost)
                    tempQueue.push_back(thisState);
            }

            Queue.clear();
            Queue = tempQueue;
        }

        return Queue[0];
    }

    void updateCosts(state_t* fromState, state_t* toState, int neighborInd){
        for (int i = 0; i < NUM_COSTS; ++i)
            toState->costsToRoot[i] = fromState->costsToRoot[i] + fromState->neighborList[neighborInd].edgeCosts[i];
    }

    void smoothPath(){

        if (pathList.size() <= 1)
            return;

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
        displayGlobalPath = globalPath;
    }    

    void generateSamples(){

        double sampling_start_time = ros::WallTime::now().toSec();
        while (ros::WallTime::now().toSec() - sampling_start_time < 0.01 && ros::ok()){

            state_t* newState = new state_t;

            if (sampleState(newState)){

                if (nodeList.size() != 0 && stateTooClose(newState) == true){
                    delete newState;
                    continue;
                }

                newState->stateId = nodeList.size();
                nodeList.push_back(newState);
                insertIntoKdtree(newState);
            }
            else
                delete newState;
        }
    }

    bool stateTooClose(state_t *stateIn){

        state_t *nearestState = getNearestState(stateIn);
        if (nearestState == NULL)
            return false;

        if (distance(stateIn->x, nearestState->x) > neighborSampleRadius
            && abs(stateIn->x[2] - nearestState->x[2]) <= neighborConnectHeight)
            return false;
            
        return true;
    }

    void updateStatesAndEdges(){
        getRobotState();

        mapCenter->x[0] = (map_min[0] + map_max[0]) / 2;
        mapCenter->x[1] = (map_min[1] + map_max[1]) / 2;
        mapCenter->x[2] = robotState->x[2];

        vector<state_t*> nearStates;
        getNearStates(mapCenter, nearStates, neighborSearchRadius);
        if (nearStates.size() == 0)
            return;

        for (int i = 0; i < nearStates.size(); ++i){
            float thisHeight = getStateHeight(nearStates[i]);
            if (thisHeight != -FLT_MAX)
                nearStates[i]->x[2]  = thisHeight;
        }

        float edgeCosts[NUM_COSTS];
        neighbor_t thisNeighbor;
        for (int i = 0; i < nearStates.size(); ++i){
            for (int j = i+1; j < nearStates.size(); ++j){

                if (abs(nearStates[i]->x[2] - nearStates[j]->x[2]) > neighborConnectHeight){
                    deleteEdge(nearStates[i], nearStates[j]);
                    continue;
                }

                float distanceBetween = distance(nearStates[i]->x, nearStates[j]->x);
                if (distanceBetween > neighborConnectRadius || distanceBetween < 0.3){
                    deleteEdge(nearStates[i], nearStates[j]);
                    continue;
                }

                if(edgePropagation(nearStates[i], nearStates[j], edgeCosts) == true){

                    deleteEdge(nearStates[i], nearStates[j]);
                    for (int k = 0; k < NUM_COSTS; ++k)
                        thisNeighbor.edgeCosts[k] = edgeCosts[k];

                    thisNeighbor.neighbor = nearStates[j];
                    nearStates[i]->neighborList.push_back(thisNeighbor);
                    thisNeighbor.neighbor = nearStates[i];
                    nearStates[j]->neighborList.push_back(thisNeighbor);
                }else{
                    deleteEdge(nearStates[i], nearStates[j]);
                }
            } 
        }
    }

    void deleteEdge(state_t* stateA, state_t* stateB){
        compareState = stateB;
        stateA->neighborList.erase(std::remove_if(stateA->neighborList.begin(), stateA->neighborList.end(), isStateExsiting), stateA->neighborList.end());
        compareState = stateA;
        stateB->neighborList.erase(std::remove_if(stateB->neighborList.begin(), stateB->neighborList.end(), isStateExsiting), stateB->neighborList.end());
    }

    

    bool edgePropagation(state_t *state_from, state_t *state_to, float edgeCosts[NUM_COSTS]){

        for (int i = 0; i < NUM_COSTS; ++i)
            edgeCosts[i] = 0;

        int steps = floor(distance(state_from->x, state_to->x) / (mapResolution));
        float stepX = (state_to->x[0]-state_from->x[0]) / steps;
        float stepY = (state_to->x[1]-state_from->x[1]) / steps;
        float stepZ = (state_to->x[2]-state_from->x[2]) / steps;

        state_t *stateCurr = new state_t;;
        stateCurr->x[0] = state_from->x[0];
        stateCurr->x[1] = state_from->x[1];
        stateCurr->x[2] = state_from->x[2];

        int rounded_x, rounded_y, indexInLocalMap;

        for (int stepCount = 0; stepCount < steps; ++stepCount){
            stateCurr->x[0] += stepX;
            stateCurr->x[1] += stepY;
            stateCurr->x[2] += stepZ;

            rounded_x = (int)((stateCurr->x[0] - map_min[0]) / mapResolution);
            rounded_y = (int)((stateCurr->x[1] - map_min[1]) / mapResolution);
            indexInLocalMap = rounded_x + rounded_y * elevationMap.occupancy.info.width;

            if (isIncollision(rounded_x, rounded_y, indexInLocalMap)){
                delete stateCurr;
                return false;
            }

            if (costUpdateFlag[2])
                edgeCosts[2] = edgeCosts[2] + mapResolution;
        }
        delete stateCurr;
        return true;
    }


    bool isIncollision(int rounded_x, int rounded_y, int index){
        if (rounded_x < 0 || rounded_x >= localMapArrayLength ||
            rounded_y < 0 || rounded_y >= localMapArrayLength )
            return true;
        if (elevationMap.costMap[index] != 0)
            return true;
        if (elevationMap.height[index] == -FLT_MAX)
            return true;
        return false;
    }

    void getNearStates(state_t *stateIn, vector<state_t*>& vectorNearStatesOut, double radius){
        kdres_t *kdres = kd_nearest_range (kdtree, stateIn->x, radius);
        vectorNearStatesOut.clear();

        int numNearVertices = kd_res_size (kdres);
        if (numNearVertices == 0) {
            kd_res_free (kdres);
            return;
        }

        kd_res_rewind (kdres);
        while (!kd_res_end(kdres)) {
            state_t *stateCurr = (state_t *) kd_res_item_data (kdres);
            vectorNearStatesOut.push_back(stateCurr);
            kd_res_next(kdres);
        }

        kd_res_free (kdres);
    }


    bool sampleState(state_t *stateCurr){
        for (int i = 0; i < 2; ++i)
            stateCurr->x[i] = (double)rand()/(RAND_MAX + 1.0)*(map_max[i] - map_min[i]) 
                - (map_max[i] - map_min[i])/2.0 + (map_max[i] + map_min[i])/2.0;

        stateCurr->theta = (double)rand()/(RAND_MAX + 1.0) * 2 * M_PI - M_PI;

        if (isIncollision(stateCurr))
            return false;

        stateCurr->x[2] = getStateHeight(stateCurr);
        if (stateCurr->x[2] == -FLT_MAX)
            return false;

        return true;
    }

    double getStateHeight(state_t* stateIn){
        int rounded_x = (int)((stateIn->x[0] - map_min[0]) / mapResolution);
        int rounded_y = (int)((stateIn->x[1] - map_min[1]) / mapResolution);
        return elevationMap.height[rounded_x + rounded_y * elevationMap.occupancy.info.width];
    }

    bool isIncollision(state_t* stateIn){

        if (stateIn->x[0] <= map_min[0] || stateIn->x[0] >= map_max[0] 
            || stateIn->x[1] <= map_min[1] || stateIn->x[1] >= map_max[1])
            return true;

        int rounded_x = (int)((stateIn->x[0] - map_min[0]) / mapResolution);
        int rounded_y = (int)((stateIn->x[1] - map_min[1]) / mapResolution);
        int index = rounded_x + rounded_y * elevationMap.occupancy.info.width;
        if (elevationMap.costMap[index] != 0)
            return true;

        if (elevationMap.height[index] == -FLT_MAX)
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

    float distance(double state_from[3], double state_to[3]){
        return sqrt((state_to[0]-state_from[0])*(state_to[0]-state_from[0]) + 
                    (state_to[1]-state_from[1])*(state_to[1]-state_from[1]) +
                    (state_to[2]-state_from[2])*(state_to[2]-state_from[2]));
    }


    void publishPRM(){        

        if (pubPRMPath.getNumSubscribers() != 0){

            visualization_msgs::MarkerArray markerArray;
            geometry_msgs::Point p;

            visualization_msgs::Marker markerPath;
            markerPath.header.frame_id = "map";
            markerPath.header.stamp = ros::Time::now();
            markerPath.action = visualization_msgs::Marker::ADD;
            markerPath.type = visualization_msgs::Marker::LINE_STRIP;
            markerPath.ns = "path";
            markerPath.id = 0;
            markerPath.scale.x = 0.2;
            markerPath.color.r = 0.0; markerPath.color.g = 0; markerPath.color.b = 1.0;
            markerPath.color.a = 1.0;

            for (int i = 0; i < displayGlobalPath.poses.size(); ++i){
                p.x = displayGlobalPath.poses[i].pose.position.x;
                p.y = displayGlobalPath.poses[i].pose.position.y;
                p.z = displayGlobalPath.poses[i].pose.position.z + 0.3;
                markerPath.points.push_back(p);
            }
            
            visualization_msgs::Marker markerGoal;
            markerGoal.header.frame_id = "map";
            markerGoal.header.stamp = ros::Time::now();
            markerGoal.action = visualization_msgs::Marker::ADD;
            markerGoal.type= visualization_msgs::Marker::SPHERE_LIST;
            markerGoal.ns = "goal";
            markerGoal.id = 1;
            markerGoal.scale.x = 0.5;
            markerGoal.color.r = 0.0; markerGoal.color.g = 0.0; markerGoal.color.b = 1.0;
            markerGoal.color.a = 1.0;

            if (displayGlobalPath.poses.size() != 0){
                p.x = displayGlobalPath.poses.back().pose.position.x;
                p.y = displayGlobalPath.poses.back().pose.position.y;
                p.z = displayGlobalPath.poses.back().pose.position.z + 0.3;
                markerGoal.points.push_back(p);
            }

            markerArray.markers.push_back(markerPath);
            markerArray.markers.push_back(markerGoal);
            pubPRMPath.publish(markerArray);
        }


        if (pubPRMGraph.getNumSubscribers() != 0){

            visualization_msgs::MarkerArray markerArray;
            geometry_msgs::Point p;

            visualization_msgs::Marker markerNode;
            markerNode.header.frame_id = "map";
            markerNode.header.stamp = ros::Time::now();
            markerNode.action = visualization_msgs::Marker::ADD;
            markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
            markerNode.ns = "nodes";
            markerNode.id = 2;
            markerNode.scale.x = 0.2;
            markerNode.color.r = 0; markerNode.color.g = 1; markerNode.color.b = 1;
            markerNode.color.a = 1;

            for (int i = 0; i < nodeList.size(); ++i){
                if (distance(nodeList[i]->x, robotState->x) >= visualizationRadius)
                    continue;
                p.x = nodeList[i]->x[0];
                p.y = nodeList[i]->x[1];
                p.z = nodeList[i]->x[2] + 0.13;
                markerNode.points.push_back(p);
            }

            visualization_msgs::Marker markerEdge;
            markerEdge.header.frame_id = "map";
            markerEdge.header.stamp = ros::Time::now();
            markerEdge.action = visualization_msgs::Marker::ADD;
            markerEdge.type = visualization_msgs::Marker::LINE_LIST;
            markerEdge.ns = "edges";
            markerEdge.id = 3;
            markerEdge.scale.x = 0.05;
            markerEdge.color.r = 0.9; markerEdge.color.g = 1; markerEdge.color.b = 0;
            markerEdge.color.a = 1;

            for (int i = 0; i < nodeList.size(); ++i){
                if (distance(nodeList[i]->x, robotState->x) >= visualizationRadius)
                    continue;
                int numNeighbors = nodeList[i]->neighborList.size();
                for (int j = 0; j < numNeighbors; ++j){
                    p.x = nodeList[i]->x[0];
                    p.y = nodeList[i]->x[1];
                    p.z = nodeList[i]->x[2] + 0.1;
                    markerEdge.points.push_back(p);
                    p.x = nodeList[i]->neighborList[j].neighbor->x[0];
                    p.y = nodeList[i]->neighborList[j].neighbor->x[1];
                    p.z = nodeList[i]->neighborList[j].neighbor->x[2] + 0.1;
                    markerEdge.points.push_back(p);
                }
            }

            markerArray.markers.push_back(markerNode);
            markerArray.markers.push_back(markerEdge);
            pubPRMGraph.publish(markerArray);

        }

        if (pubSingleSourcePaths.getNumSubscribers() != 0){

            visualization_msgs::MarkerArray markerArray;
            geometry_msgs::Point p;

            if (planningFlag == false){
                pubSingleSourcePaths.publish(markerArray);
                return;
            }

            visualization_msgs::Marker markersPath;
            markersPath.header.frame_id = "map";
            markersPath.header.stamp = ros::Time::now();
            markersPath.action = visualization_msgs::Marker::ADD;
            markersPath.type = visualization_msgs::Marker::LINE_LIST;
            markersPath.ns = "path";
            markersPath.id = 4;
            markersPath.scale.x = 0.05;
            markersPath.color.r = 0.3; markersPath.color.g = 0; markersPath.color.b = 1.0;
            markersPath.color.a = 1.0;

            for (int i = 0; i < nodeList.size(); ++i){
                if (nodeList[i]->parentState == NULL)
                    continue;
                p.x = nodeList[i]->x[0];
                p.y = nodeList[i]->x[1];
                p.z = nodeList[i]->x[2] + 0.2;
                markersPath.points.push_back(p);
                p.x = nodeList[i]->parentState->x[0];
                p.y = nodeList[i]->parentState->x[1];
                p.z = nodeList[i]->parentState->x[2]+0.2;
                markersPath.points.push_back(p);
            }

            markerArray.markers.push_back(markersPath);
            pubSingleSourcePaths.publish(markerArray);
        }
    }

    void publishPathStop(){

        globalPath.header.frame_id = "map";
        globalPath.header.stamp = ros::Time::now();

        pubGlobalPath.publish(globalPath);

        planningFlag = false;
    }

    void publishRoadmap2Cloud(){
        if (pubCloudPRMNodes.getNumSubscribers() == 0 && pubCloudPRMGraph.getNumSubscribers() == 0)
            return;

        int sizeCloud = nodeList.size();

        PointType thisPoint;
        pcl::PointCloud<PointType> nodeCloud; nodeCloud.resize(sizeCloud);
        pcl::PointCloud<PointType> adjacencyCloud; adjacencyCloud.resize(sizeCloud*sizeCloud);
        for (int i = 0; i < sizeCloud; ++i){

            thisPoint.x = nodeList[i]->x[0];
            thisPoint.y = nodeList[i]->x[1];
            thisPoint.z = nodeList[i]->x[2]+0.15;
            thisPoint.intensity = i;
            nodeCloud.points[i] = thisPoint;

            int numNeighbors = nodeList[i]->neighborList.size();
            for (int j = 0; j < numNeighbors; ++j){
                int index = nodeList[i]->stateId + sizeCloud * nodeList[i]->neighborList[j].neighbor->stateId;
                adjacencyCloud.points[index].intensity = 1;
            }
        }

        sensor_msgs::PointCloud2 laserCloudTemp;
        pcl::toROSMsg(nodeCloud, laserCloudTemp);
        laserCloudTemp.header.frame_id = "map";
        laserCloudTemp.header.stamp = ros::Time::now();
        pubCloudPRMNodes.publish(laserCloudTemp);
        pcl::toROSMsg(adjacencyCloud, laserCloudTemp);
        laserCloudTemp.header.frame_id = "map";
        laserCloudTemp.header.stamp = ros::Time::now();
        pubCloudPRMGraph.publish(laserCloudTemp);
    }

    void getRobotState(){
        try{listener.lookupTransform("map","base_link", ros::Time(0), transform); } 
        catch (tf::TransformException ex){ /*ROS_ERROR("Transfrom Failure.");*/ return; }
        robotState->x[0] = transform.getOrigin().x();
        robotState->x[1] = transform.getOrigin().y();
        robotState->x[2] = transform.getOrigin().z();

        double roll, pitch, yaw;
        tf::Matrix3x3 m(transform.getRotation());
        m.getRPY(roll, pitch, yaw);
        robotState->theta = yaw + M_PI;
    }

};


int main(int argc, char** argv){

    ros::init(argc, argv, "tm_mapping");

    TMPRM TPRM;

    ROS_INFO("\033[1;32m---->\033[0m Planner Started.");

    ros::spin();
    return 0;
}
