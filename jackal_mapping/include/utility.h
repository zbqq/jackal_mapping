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
#include <array>
#include <thread>
#include <mutex>

#include "marker/Marker.h"
#include "marker/MarkerArray.h"

#include "planner/kdtree.h"
#include "planner/cubic_spline_interpolator.h"

#include "elevation_msgs/OccupancyElevation.h"

using namespace std;

typedef pcl::PointXYZI  PointType;
typedef struct kdtree kdtree_t;
typedef struct kdres kdres_t;

extern const bool urbanMapping = true;

extern const int N_SCAN = 16;
extern const int Horizon_SCAN = 1800;

extern const float mapResolution = 0.1;
extern const float mapCubeLength = 1.0;
extern const int mapCubeArrayLength = mapCubeLength / mapResolution;
extern const int mapArrayLength = 2000 / mapCubeLength;
extern const int rootCubeIndex = mapArrayLength / 2;

extern const int scanNumCurbFilter = 8;
extern const int scanNumSlopeFilter = 10;
extern const int scanNumMax = std::max(scanNumCurbFilter, scanNumSlopeFilter);

extern const float sensorRangeLimit = 12;
extern const float filterHeightLimit = (urbanMapping == true) ? 0.06 : 0.1;       
extern const float filterAngleLimit = 20;       
extern const int filterHeightMapArrayLength = sensorRangeLimit*2 / mapResolution;

extern const bool predictionEnableFlag = true;
extern const float predictionKernalSize = 0.2;

extern const float p_occupied_when_laser = 0.9;
extern const float p_occupied_when_no_laser = 0.2;
extern const float large_log_odds = 100;
extern const float max_log_odds_for_belief = 20;

extern const int localMapLength = 20;
extern const int localMapArrayLength = localMapLength / mapResolution;

extern const float visualizationRadius = 100;
extern const float visualizationFrequency = 5;

extern const float robotRadius = 0.2;
extern const float sensorHeight = 0.5;
extern const int footprintRadiusLength = int(robotRadius / mapResolution);

extern const int traversabilityObserveTimeTh = 10;
extern const float traversabilityCalculatingDistance = 8.0;

extern const int NUM_COSTS = 3;
extern const int tmp[] = {2};
extern const std::vector<int> costHierarchy(tmp, tmp+sizeof(tmp)/sizeof(int));

extern const float costmapInflationRadius = 0.3;
extern const float neighborSampleRadius  = 0.5;
extern const float neighborConnectHeight = 1.0;
extern const float neighborConnectRadius = 2.0;
extern const float neighborSearchRadius = localMapLength / 2;

struct grid_t;
struct mapCell_t;
struct childMap_t;
struct state_t;
struct neighbor_t;

struct grid_t{
    int mapID;
    int cubeX;
    int cubeY;
    int gridX;
    int gridY;
    int gridIndex;
};

struct mapCell_t{

    PointType *xyz;

    grid_t grid;

    float log_odds;

    int observeTimes;
    
    float occupancy, occupancyVar;
    float elevation, elevationVar;

    mapCell_t(){

        log_odds = 0.5;
        observeTimes = 0;

        elevation = -FLT_MAX;
        elevationVar = 1e3;

        occupancy = 0;
        occupancyVar = 1e3;
    }

    void updatePoint(){
        xyz->z = elevation;
        xyz->intensity = occupancy;
    }
    void updateElevation(float elevIn, float varIn){
        elevation = elevIn;
        elevationVar = varIn;
        updatePoint();
    }
    void updateOccupancy(float occupIn){
        occupancy = occupIn;
        updatePoint();
    }
};

struct childMap_t{

    vector<vector<mapCell_t*> > cellArray;
    int subInd;
    int indX;
    int indY;
    float originX;
    float originY;
    pcl::PointCloud<PointType> cloud;

    childMap_t(int id, int indx, int indy){

        subInd = id;
        indX = indx;
        indY = indy;
        originX = (indX - rootCubeIndex) * mapCubeLength - mapCubeLength/2.0;
        originY = (indY - rootCubeIndex) * mapCubeLength - mapCubeLength/2.0;

        cellArray.resize(mapCubeArrayLength);
        for (int i = 0; i < mapCubeArrayLength; ++i)
            cellArray[i].resize(mapCubeArrayLength);

        for (int i = 0; i < mapCubeArrayLength; ++i)
            for (int j = 0; j < mapCubeArrayLength; ++j)
                cellArray[i][j] = new mapCell_t;

        cloud.points.resize(mapCubeArrayLength*mapCubeArrayLength);

        for (int i = 0; i < mapCubeArrayLength; ++i)
            for (int j = 0; j < mapCubeArrayLength; ++j)
                cellArray[i][j]->xyz = &cloud.points[i + j*mapCubeArrayLength];

        for (int i = 0; i < mapCubeArrayLength; ++i){
            for (int j = 0; j < mapCubeArrayLength; ++j){
                
                int index = i + j * mapCubeArrayLength;
                cloud.points[index].x = originX + i * mapResolution;
                cloud.points[index].y = originY + j * mapResolution;
                cloud.points[index].z = std::numeric_limits<float>::quiet_NaN();
                cloud.points[index].intensity = cellArray[i][j]->occupancy;

                cellArray[i][j]->grid.mapID = subInd;
                cellArray[i][j]->grid.cubeX = indX;
                cellArray[i][j]->grid.cubeY = indY;
                cellArray[i][j]->grid.gridX = i;
                cellArray[i][j]->grid.gridY = j;
                cellArray[i][j]->grid.gridIndex = index;
            }
        }
    }
};


struct state_t{
    double x[3];
    float theta;
    int stateId;

    float costsToRoot[NUM_COSTS];
    float costsToParent[NUM_COSTS];
    float costsToGo[NUM_COSTS];

    state_t* parentState;
    vector<neighbor_t> neighborList;
    vector<state_t*> childList;

    state_t(){
        parentState = NULL;
        for (int i = 0; i < NUM_COSTS; ++i){
            costsToRoot[i] = FLT_MAX;
            costsToParent[i] = FLT_MAX;
            costsToGo[i] = FLT_MAX;
        }
    }
    
    state_t(state_t* stateIn){

        for (int i = 0; i < 3; ++i)
            x[i] = stateIn->x[i];
        theta = stateIn->theta;

        parentState = NULL;
        for (int i = 0; i < NUM_COSTS; ++i){
            costsToRoot[i] = FLT_MAX;
            costsToParent[i] = stateIn->costsToParent[i];
        }
    }
};


struct neighbor_t{
    state_t* neighbor;
    float edgeCosts[NUM_COSTS];
    neighbor_t(){
        neighbor = NULL;
        for (int i = 0; i < NUM_COSTS; ++i)
            edgeCosts[i] = FLT_MAX;
    }
};

state_t *compareState;
bool isStateExsiting(neighbor_t neighborIn){
    return neighborIn.neighbor == compareState ? true : false;
}

float pointDistance(PointType p1, PointType p2){
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

#endif
