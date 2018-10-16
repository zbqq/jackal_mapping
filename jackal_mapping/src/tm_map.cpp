#include "utility.h"

class TMMapping{

private:


    ros::NodeHandle nh;

    std::mutex mtx;

    tf::TransformListener listener;
    tf::StampedTransform transform;

    ros::Subscriber subFilteredGroundCloud;

    ros::Publisher pubOccupancyMapLocal;
    ros::Publisher pubOccupancyMapLocalHeight;
    ros::Publisher pubElevationCloud;

    pcl::PointCloud<PointType>::Ptr laserCloud;
    pcl::PointCloud<PointType>::Ptr laserCloudElevation;

    nav_msgs::OccupancyGrid occupancyMap2D;
    elevation_msgs::OccupancyElevation occupancyMap2DHeight;

    int pubCount;
    
    int mapArrayCount;
    int **mapArrayInd;
    int **predictionArrayFlag;
    vector<childMap_t*> mapArray;

    PointType robotPoint;
    PointType localMapOriginPoint;
    grid_t localMapOriginGrid;

    cv::Mat matCov, matEig, matVec;

    vector<mapCell_t*> observingList1;
    vector<mapCell_t*> observingList2;

public:
    TMMapping():
        nh("~"),
        pubCount(1),
        mapArrayCount(0){
        subFilteredGroundCloud = nh.subscribe<sensor_msgs::PointCloud2>("/filtered_pointcloud", 1, &TMMapping::cloudHandler, this);
        pubOccupancyMapLocal = nh.advertise<nav_msgs::OccupancyGrid> ("/occupancy_map_local", 1);
        pubOccupancyMapLocalHeight = nh.advertise<elevation_msgs::OccupancyElevation> ("/occupancy_map_local_height", 1);
        pubElevationCloud = nh.advertise<sensor_msgs::PointCloud2> ("/elevation_pointcloud", 1);

        allocateMemory(); 
    }

    ~TMMapping(){}

    

    void allocateMemory(){
        laserCloud.reset(new pcl::PointCloud<PointType>());
        laserCloudElevation.reset(new pcl::PointCloud<PointType>());
        
        mapArrayInd = new int*[mapArrayLength];
        for (int i = 0; i < mapArrayLength; ++i)
            mapArrayInd[i] = new int[mapArrayLength];

        for (int i = 0; i < mapArrayLength; ++i)
            for (int j = 0; j < mapArrayLength; ++j)
                mapArrayInd[i][j] = -1;

        predictionArrayFlag = new int*[mapArrayLength];
        for (int i = 0; i < mapArrayLength; ++i)
            predictionArrayFlag[i] = new int[mapArrayLength];

        for (int i = 0; i < mapArrayLength; ++i)
            for (int j = 0; j < mapArrayLength; ++j)
                predictionArrayFlag[i][j] = false;

        matCov = cv::Mat (3, 3, CV_32F, cv::Scalar::all(0));
        matEig = cv::Mat (1, 3, CV_32F, cv::Scalar::all(0));
        matVec = cv::Mat (3, 3, CV_32F, cv::Scalar::all(0));

        initializeLocalOccupancyMap();
    }


    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

        std::lock_guard<std::mutex> lock(mtx);

        if (getRobotPosition() == false) 
            return;

        pcl::fromROSMsg(*laserCloudMsg, *laserCloud);

        updateElevationMap();

        publishMap();
    }

    void updateElevationMap(){
        int cloudSize = laserCloud->points.size();
        for (int i = 0; i < cloudSize; ++i){
            laserCloud->points[i].z -= 0.2;
            updateElevationMap(&laserCloud->points[i]);
        }
    }

    void updateElevationMap(PointType *point){
        grid_t thisGrid;
        if (findPointGridInMap(&thisGrid, point) == false) return;

        mapCell_t *thisCell = grid2Cell(&thisGrid);

        updateCellElevation(thisCell, point);

        updateCellOccupancy(thisCell, point);

        updateCellObservationTime(thisCell);
    }

    void updateCellObservationTime(mapCell_t *thisCell){
        ++thisCell->observeTimes;
        if (thisCell->observeTimes >= traversabilityObserveTimeTh)
            observingList1.push_back(thisCell);
    }

    void updateCellOccupancy(mapCell_t *thisCell, PointType *point){

        float p;
        if (point->intensity == 100)
            p = p_occupied_when_laser;
        else
            p = p_occupied_when_no_laser;
        thisCell->log_odds += std::log(p / (1 - p));

        if (thisCell->log_odds < -large_log_odds)
            thisCell->log_odds = -large_log_odds;
        else if (thisCell->log_odds > large_log_odds)
            thisCell->log_odds = large_log_odds;

        float occupancy;
        if (thisCell->log_odds < -max_log_odds_for_belief)
            occupancy = 0;
        else if (thisCell->log_odds > max_log_odds_for_belief)
            occupancy = 100;
        else
            occupancy = (int)(lround((1 - 1 / (1 + std::exp(thisCell->log_odds))) * 100));

        thisCell->updateOccupancy(occupancy);
    }

    void updateCellElevation(mapCell_t *thisCell, PointType *point){
        if (thisCell->elevation == -FLT_MAX){
            thisCell->elevation = point->z;
            thisCell->elevationVar = pointDistance(robotPoint, *point);
            return;
        }

        float x_pred = thisCell->elevation;
        float P_pred = thisCell->elevationVar + 0.01;

        float R_factor = (thisCell->observeTimes > 20) ? 10 : 1;
        float R = pointDistance(robotPoint, *point) * R_factor;
        float K = P_pred / (P_pred + R);
        float y = point->z;
        float x_final = x_pred + K * (y - x_pred);
        float P_final = (1 - K) * P_pred;

        thisCell->updateElevation(x_final, P_final);
    }

    mapCell_t* grid2Cell(grid_t *thisGrid){
        return mapArray[mapArrayInd[thisGrid->cubeX][thisGrid->cubeY]]->cellArray[thisGrid->gridX][thisGrid->gridY];
    }

    bool findPointGridInMap(grid_t *gridOut, PointType *point){

        grid_t thisGrid;
        getPointCubeIndex(&thisGrid.cubeX, &thisGrid.cubeY, point);

        if (thisGrid.cubeX >= 0 && thisGrid.cubeX < mapArrayLength && 
            thisGrid.cubeY >= 0 && thisGrid.cubeY < mapArrayLength){

            if (mapArrayInd[thisGrid.cubeX][thisGrid.cubeY] == -1){
                childMap_t *thisChildMap = new childMap_t(mapArrayCount, thisGrid.cubeX, thisGrid.cubeY);
                mapArray.push_back(thisChildMap);
                mapArrayInd[thisGrid.cubeX][thisGrid.cubeY] = mapArrayCount;
                ++mapArrayCount;
            }
        }else{
            return false;
        }

        thisGrid.mapID = mapArrayInd[thisGrid.cubeX][thisGrid.cubeY];

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


    void TMThread(){

        ros::Rate rate(10);

        while (ros::ok()){

            tmMapCalculation();

            rate.sleep();
        }
    }

    void tmMapCalculation(){

        if (observingList1.size() == 0)
            return;

        observingList2 = observingList1;
        observingList1.clear();

        int listSize = observingList2.size();

        for (int i = 0; i < listSize; ++i){

            mapCell_t *thisCell = observingList2[i];

            PointType thisPoint;
            thisPoint.x = thisCell->xyz->x;
            thisPoint.y = thisCell->xyz->y;
            thisPoint.z = thisCell->xyz->z;

            if (pointDistance(thisPoint, robotPoint) >= traversabilityCalculatingDistance)
                continue;

            vector<float> xyzVector = findNeighborElevations(thisCell);

            if (xyzVector.size() <= 2)
                continue;

            Eigen::MatrixXf matPoints = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(xyzVector.data(), xyzVector.size() / 3, 3);


            float minElevation = matPoints.col(2).minCoeff();
            float maxElevation = matPoints.col(2).maxCoeff();
            float maxDifference = maxElevation - minElevation;

            if (maxDifference > filterHeightLimit){
                thisPoint.intensity = 100;
                updateCellOccupancy(thisCell, &thisPoint);
                continue;
            }

            updateCellOccupancy(thisCell, &thisPoint);
        }
    }

    vector<float> findNeighborElevations(mapCell_t *centerCell){

        vector<float> xyzVector;

        grid_t centerGrid = centerCell->grid;
        grid_t thisGrid;

        for (int k = -footprintRadiusLength; k <= footprintRadiusLength; ++k){
            for (int l = -footprintRadiusLength; l <= footprintRadiusLength; ++l){

                if (std::sqrt(float(k*k + l*l)) * mapResolution > robotRadius)
                    continue;

                thisGrid.cubeX = centerGrid.cubeX;
                thisGrid.cubeY = centerGrid.cubeY;
                thisGrid.gridX = centerGrid.gridX + k;
                thisGrid.gridY = centerGrid.gridY + l;

                if(thisGrid.gridX < 0){ --thisGrid.cubeX; thisGrid.gridX = thisGrid.gridX + mapCubeArrayLength;
                }else if(thisGrid.gridX >= mapCubeArrayLength){ ++thisGrid.cubeX; thisGrid.gridX = thisGrid.gridX - mapCubeArrayLength; }
                if(thisGrid.gridY < 0){ --thisGrid.cubeY; thisGrid.gridY = thisGrid.gridY + mapCubeArrayLength;
                }else if(thisGrid.gridY >= mapCubeArrayLength){ ++thisGrid.cubeY; thisGrid.gridY = thisGrid.gridY - mapCubeArrayLength; }

                int mapInd = mapArrayInd[thisGrid.cubeX][thisGrid.cubeY];
                if (mapInd == -1) continue;

                mapCell_t *thisCell = grid2Cell(&thisGrid);

                if (thisCell->elevation != -FLT_MAX){
                    xyzVector.push_back(thisCell->xyz->x);
                    xyzVector.push_back(thisCell->xyz->y);
                    xyzVector.push_back(thisCell->xyz->z);
                }
            }
        }

        return xyzVector;
    }


    void publishMap(){

        pubCount++;
        if (pubCount > visualizationFrequency){
            pubCount = 1;
            publishLocalMap();
            publishTraversabilityMap();
        }
    }

    void publishLocalMap(){

        if (pubOccupancyMapLocal.getNumSubscribers() == 0 &&
            pubOccupancyMapLocalHeight.getNumSubscribers() == 0)
            return;


        std::fill(occupancyMap2DHeight.occupancy.data.begin(), occupancyMap2DHeight.occupancy.data.end(), -1);
        std::fill(occupancyMap2DHeight.height.begin(), occupancyMap2DHeight.height.end(), -FLT_MAX);
        std::fill(occupancyMap2DHeight.costMap.begin(), occupancyMap2DHeight.costMap.end(), 0);
        

        localMapOriginPoint.x = robotPoint.x - localMapLength / 2;
        localMapOriginPoint.y = robotPoint.y - localMapLength / 2;
        localMapOriginPoint.z = robotPoint.z;

        localMapOriginGrid.cubeX = int((localMapOriginPoint.x + mapCubeLength/2.0) / mapCubeLength) + rootCubeIndex;
        localMapOriginGrid.cubeY = int((localMapOriginPoint.y + mapCubeLength/2.0) / mapCubeLength) + rootCubeIndex;
        if (localMapOriginPoint.x + mapCubeLength/2.0 < 0)  --localMapOriginGrid.cubeX;
        if (localMapOriginPoint.y + mapCubeLength/2.0 < 0)  --localMapOriginGrid.cubeY;

        float originCubeOriginX, originCubeOriginY;
        originCubeOriginX = (localMapOriginGrid.cubeX - rootCubeIndex) * mapCubeLength - mapCubeLength/2.0;
        originCubeOriginY = (localMapOriginGrid.cubeY - rootCubeIndex) * mapCubeLength - mapCubeLength/2.0;
        localMapOriginGrid.gridX = int((localMapOriginPoint.x - originCubeOriginX) / mapResolution);
        localMapOriginGrid.gridY = int((localMapOriginPoint.y - originCubeOriginY) / mapResolution);


        occupancyMap2DHeight.header.stamp = ros::Time::now();
        occupancyMap2DHeight.occupancy.header.stamp = occupancyMap2DHeight.header.stamp;
        occupancyMap2DHeight.occupancy.info.origin.position.x = localMapOriginPoint.x;
        occupancyMap2DHeight.occupancy.info.origin.position.y = localMapOriginPoint.y;
        occupancyMap2DHeight.occupancy.info.origin.position.z = localMapOriginPoint.z + 10;


        for (int i = 0; i < localMapArrayLength; ++i){
            for (int j = 0; j < localMapArrayLength; ++j){

                int indX = localMapOriginGrid.gridX + i;
                int indY = localMapOriginGrid.gridY + j;

                grid_t thisGrid;

                thisGrid.cubeX = localMapOriginGrid.cubeX + indX / mapCubeArrayLength;
                thisGrid.cubeY = localMapOriginGrid.cubeY + indY / mapCubeArrayLength;

                thisGrid.gridX = indX % mapCubeArrayLength;
                thisGrid.gridY = indY % mapCubeArrayLength;

                if (mapArrayInd[thisGrid.cubeX][thisGrid.cubeY] == -1) {
                    continue;
                }
                
                mapCell_t *thisCell = grid2Cell(&thisGrid);

                if (thisCell->elevation != -FLT_MAX){
                    int index = i + j * localMapArrayLength;
                    occupancyMap2DHeight.height[index] = thisCell->elevation;
                    occupancyMap2DHeight.occupancy.data[index] = thisCell->occupancy > 80 ? 100 : 0;
                }
            }
        }

        pubOccupancyMapLocalHeight.publish(occupancyMap2DHeight);
        pubOccupancyMapLocal.publish(occupancyMap2DHeight.occupancy);
    }
    

    void initializeLocalOccupancyMap(){

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
        catch (tf::TransformException ex){ /*ROS_ERROR("Transfrom Failure.");*/ return false; }

        robotPoint.x = transform.getOrigin().x();
        robotPoint.y = transform.getOrigin().y();
        robotPoint.z = transform.getOrigin().z();

        return true;
    }


    void publishTraversabilityMap(){

        if (pubElevationCloud.getNumSubscribers() == 0)
            return;

        int currentCubeX, currentCubeY;
        getPointCubeIndex(&currentCubeX, &currentCubeY, &robotPoint);

        int visualLength = int(visualizationRadius / mapCubeLength);
        for (int i = -visualLength; i <= visualLength; ++i){
            for (int j = -visualLength; j <= visualLength; ++j){

                if (sqrt(float(i*i+j*j)) >= visualLength) continue;

                int idx = i + currentCubeX;
                int idy = j + currentCubeY;

                if (idx < 0 || idx >= mapArrayLength ||  idy < 0 || idy >= mapArrayLength) continue;

                if (mapArrayInd[idx][idy] == -1) continue;

                *laserCloudElevation += mapArray[mapArrayInd[idx][idy]]->cloud;
            }
        }

        sensor_msgs::PointCloud2 laserCloudTemp;
        pcl::toROSMsg(*laserCloudElevation, laserCloudTemp);
        laserCloudTemp.header.frame_id = "/map";
        laserCloudTemp.header.stamp = ros::Time::now();
        pubElevationCloud.publish(laserCloudTemp);

        laserCloudElevation->clear();
    }
};




int main(int argc, char** argv){

    ros::init(argc, argv, "tm_mapping");
    
    TMMapping tMapping;

    std::thread predictionThread(&TMMapping::TMThread, &tMapping);

    ROS_INFO("\033[1;32m---->\033[0m Mapping Started.");
    ROS_INFO("\033[1;32m---->\033[0m Mapping Scenario: %s.", 
        urbanMapping == true ? "\033[1;31mUrban\033[0m" : "\033[1;31mTerrain\033[0m");

    ros::spin();

    return 0;
}