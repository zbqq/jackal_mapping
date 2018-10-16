#include "utility.h"

class TMFilter{
    
private:

    ros::NodeHandle nh;

    ros::Subscriber subCloud;
    ros::Publisher pubCloud;
    ros::Publisher pubCloudVisual;

    ros::Publisher pubLaserScan;

    pcl::PointCloud<PointType>::Ptr laserCloudIn;
    pcl::PointCloud<PointType>::Ptr laserCloudOut; 
    pcl::PointCloud<PointType>::Ptr laserCloudObstacles;

    tf::TransformListener listener;
    tf::StampedTransform transform;

    PointType robotPoint;
    PointType localMapOrigin;

    vector<vector<PointType>> laserCloudMatrix;

    cv::Mat obstacleMatrix;
    cv::Mat rangeMatrix;

    sensor_msgs::LaserScan laserScan;

    float **minHeight;
    float **maxHeight;
    bool **obstFlag;
    bool **initFlag;

public:
    TMFilter():
        nh("~"){

        subCloud = nh.subscribe<sensor_msgs::PointCloud2>("/full_cloud_info", 1, &TMFilter::cloudHandler, this);
        pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/filtered_pointcloud", 1);
        pubCloudVisual = nh.advertise<sensor_msgs::PointCloud2> ("/filtered_pointcloud_visual", 1);
        pubLaserScan = nh.advertise<sensor_msgs::LaserScan> ("/pointcloud_2_laserscan", 1);  
        allocateMemory();
        pointcloud2laserscanInitialization();
    }

    void allocateMemory(){

        laserCloudIn.reset(new pcl::PointCloud<PointType>());
        laserCloudOut.reset(new pcl::PointCloud<PointType>());
        laserCloudObstacles.reset(new pcl::PointCloud<PointType>());

        obstacleMatrix = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(-1));
        rangeMatrix =  cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(-1));

        laserCloudMatrix.resize(N_SCAN);
        for (int i = 0; i < N_SCAN; ++i)
            laserCloudMatrix[i].resize(Horizon_SCAN);

        initFlag = new bool*[filterHeightMapArrayLength];
        for (int i = 0; i < filterHeightMapArrayLength; ++i)
            initFlag[i] = new bool[filterHeightMapArrayLength];

        obstFlag = new bool*[filterHeightMapArrayLength];
        for (int i = 0; i < filterHeightMapArrayLength; ++i)
            obstFlag[i] = new bool[filterHeightMapArrayLength];

        minHeight = new float*[filterHeightMapArrayLength];
        for (int i = 0; i < filterHeightMapArrayLength; ++i)
            minHeight[i] = new float[filterHeightMapArrayLength];

        maxHeight = new float*[filterHeightMapArrayLength];
        for (int i = 0; i < filterHeightMapArrayLength; ++i)
            maxHeight[i] = new float[filterHeightMapArrayLength];

        resetParameters();
    }

    void resetParameters(){

        laserCloudIn->clear();
        laserCloudOut->clear();
        laserCloudObstacles->clear();

        obstacleMatrix = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(-1));
        rangeMatrix =  cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(-1));

        for (int i = 0; i < filterHeightMapArrayLength; ++i){
            for (int j = 0; j < filterHeightMapArrayLength; ++j){
                initFlag[i][j] = false;
                obstFlag[i][j] = false;
            }
        }
    }

    ~TMFilter(){}


    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
        
        extractRawCloud(laserCloudMsg);

        if (transformCloud() == false) return;

        cloud2Matrix();

        applyFilter();

        extractFilteredCloud();

        downsampleCloud();

        predictCloudBGK();

        publishCloud();

        publishLaserScan();

        resetParameters();
    }

    void extractRawCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);

        for (int i = 0; i < N_SCAN; ++i){
            for (int j = 0; j < Horizon_SCAN; ++j){
                int index = j  + i * Horizon_SCAN;
                if (laserCloudIn->points[index].intensity == std::numeric_limits<float>::quiet_NaN()) continue;
                rangeMatrix.at<float>(i, j) = laserCloudIn->points[index].intensity;
                obstacleMatrix.at<int>(i, j) = 0;
            }
        }
    }

    bool transformCloud(){
        try{listener.lookupTransform("map","base_link", ros::Time(0), transform); }
        catch (tf::TransformException ex){ /*ROS_ERROR("Transfrom Failure.");*/ return false; }

        robotPoint.x = transform.getOrigin().x();
        robotPoint.y = transform.getOrigin().y();
        robotPoint.z = transform.getOrigin().z();

        laserCloudIn->header.frame_id = "base_link";
        laserCloudIn->header.stamp = 0;

        pcl::PointCloud<PointType> laserCloudTemp;
        pcl_ros::transformPointCloud("map", *laserCloudIn, laserCloudTemp, listener);
        *laserCloudIn = laserCloudTemp;

        return true;
    }

    void cloud2Matrix(){

        for (int i = 0; i < N_SCAN; ++i){
            for (int j = 0; j < Horizon_SCAN; ++j){
                int index = j  + i * Horizon_SCAN;
                PointType p = laserCloudIn->points[index];
                laserCloudMatrix[i][j] = p;
            }
        }
    }

    void applyFilter(){

        if (urbanMapping == true){
            positiveCurbFilter();
            negativeCurbFilter();
        }

        slopeFilter();
    }

    void positiveCurbFilter(){
        int rangeCompareNeighborNum = 3;
        float diff[Horizon_SCAN - 1];

        for (int i = 0; i < scanNumCurbFilter; ++i){
            for (int j = 0; j < Horizon_SCAN - 1; ++j)
                diff[j] = rangeMatrix.at<float>(i, j) - rangeMatrix.at<float>(i, j+1);

            for (int j = rangeCompareNeighborNum; j < Horizon_SCAN - rangeCompareNeighborNum; ++j){

                if (obstacleMatrix.at<int>(i, j) == 1)
                    continue;

                bool breakFlag = false;
                if (rangeMatrix.at<float>(i, j) > sensorRangeLimit)
                    continue;
                for (int k = -rangeCompareNeighborNum; k <= rangeCompareNeighborNum; ++k)
                    if (rangeMatrix.at<float>(i, j+k) == -1){
                        breakFlag = true;
                        break;
                    }
                if (breakFlag == true) continue;
                for (int k = -rangeCompareNeighborNum; k < rangeCompareNeighborNum-1; ++k)
                    if (diff[j+k] * diff[j+k+1] <= 0){
                        breakFlag = true;
                        break;
                    }
                if (breakFlag == true) continue;
                if (abs(rangeMatrix.at<float>(i, j-rangeCompareNeighborNum) - rangeMatrix.at<float>(i, j+rangeCompareNeighborNum)) /rangeMatrix.at<float>(i, j) < 0.03)
                    continue;
                obstacleMatrix.at<int>(i, j) = 1;
            }
        }
    }

    void negativeCurbFilter(){
        int rangeCompareNeighborNum = 3;

        for (int i = 0; i < scanNumCurbFilter; ++i){
            for (int j = 0; j < Horizon_SCAN; ++j){
                if (obstacleMatrix.at<int>(i, j) == 1)
                    continue;
                if (rangeMatrix.at<float>(i, j) == -1)
                    continue;
                for (int m = -rangeCompareNeighborNum; m <= rangeCompareNeighborNum; ++m){
                    int k = j + m;
                    if (k < 0 || k >= Horizon_SCAN)
                        continue;
                    if (rangeMatrix.at<float>(i, k) == -1)
                        continue;
                    if (laserCloudMatrix[i][j].z - laserCloudMatrix[i][k].z > 0.1
                        && pointDistance(laserCloudMatrix[i][j], laserCloudMatrix[i][k]) <= 1.0){
                        obstacleMatrix.at<int>(i, j) = 1;
                        break;
                    }
                }
            }
        }
    }

    void slopeFilter(){
        
        for (int i = 0; i < scanNumSlopeFilter; ++i){
            for (int j = 0; j < Horizon_SCAN; ++j){
                if (obstacleMatrix.at<int>(i, j) == 1)
                    continue;
                if (rangeMatrix.at<float>(i, j) == -1 || rangeMatrix.at<float>(i+1, j) == -1)
                    continue;
                float diffX = laserCloudMatrix[i+1][j].x - laserCloudMatrix[i][j].x;
                float diffY = laserCloudMatrix[i+1][j].y - laserCloudMatrix[i][j].y;
                float diffZ = laserCloudMatrix[i+1][j].z - laserCloudMatrix[i][j].z;
                float angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY)) * 180 / M_PI;
                if (angle < -filterAngleLimit || angle > filterAngleLimit){
                    obstacleMatrix.at<int>(i, j) = 1;
                    continue;
                }
            }
        }
    }

    

    void extractFilteredCloud(){
        for (int i = 0; i < scanNumMax; ++i){
            for (int j = 0; j < Horizon_SCAN; ++j){
                if (rangeMatrix.at<float>(i, j) >= sensorRangeLimit ||
                    rangeMatrix.at<float>(i, j) == -1)
                    continue;
                PointType p = laserCloudMatrix[i][j];
                p.intensity = obstacleMatrix.at<int>(i,j) == 1 ? 100 : 0;
                laserCloudOut->push_back(p);
                if (p.intensity == 100)
                    laserCloudObstacles->push_back(p);
            }
        }
        if (pubCloudVisual.getNumSubscribers() != 0){
            sensor_msgs::PointCloud2 laserCloudTemp;
            pcl::toROSMsg(*laserCloudOut, laserCloudTemp);
            laserCloudTemp.header.stamp = ros::Time::now();
            laserCloudTemp.header.frame_id = "map";
            pubCloudVisual.publish(laserCloudTemp);
        }
    }

    void downsampleCloud(){

        float roundedX = float(int(robotPoint.x * 10.0f)) / 10.0f;
        float roundedY = float(int(robotPoint.y * 10.0f)) / 10.0f;
        localMapOrigin.x = roundedX - sensorRangeLimit;
        localMapOrigin.y = roundedY - sensorRangeLimit;
        
        int cloudSize = laserCloudOut->points.size();
        for (int i = 0; i < cloudSize; ++i){

            int idx = (laserCloudOut->points[i].x - localMapOrigin.x) / mapResolution;
            int idy = (laserCloudOut->points[i].y - localMapOrigin.y) / mapResolution;

            if (idx < 0 || idy < 0 || idx >= filterHeightMapArrayLength || idy >= filterHeightMapArrayLength)
                continue;

            if (laserCloudOut->points[i].intensity == 100)
                obstFlag[idx][idy] = true;

            if (initFlag[idx][idy] == false){
                minHeight[idx][idy] = laserCloudOut->points[i].z;
                maxHeight[idx][idy] = laserCloudOut->points[i].z;
                initFlag[idx][idy] = true;
            } else {
                minHeight[idx][idy] = std::min(minHeight[idx][idy], laserCloudOut->points[i].z);
                maxHeight[idx][idy] = std::max(maxHeight[idx][idy], laserCloudOut->points[i].z);
            }
        }

        pcl::PointCloud<PointType>::Ptr laserCloudTemp(new pcl::PointCloud<PointType>());

        for (int i = 0; i < filterHeightMapArrayLength; ++i){
            for (int j = 0; j < filterHeightMapArrayLength; ++j){
                if (initFlag[i][j] == false)
                    continue;
                PointType thisPoint;
                thisPoint.x = localMapOrigin.x + i * mapResolution + mapResolution / 2.0;
                thisPoint.y = localMapOrigin.y + j * mapResolution + mapResolution / 2.0;
                thisPoint.z = maxHeight[i][j];

                if (obstFlag[i][j] == true || maxHeight[i][j] - minHeight[i][j] > filterHeightLimit){
                    obstFlag[i][j] = true;
                    thisPoint.intensity = 100;
                    laserCloudTemp->push_back(thisPoint);
                }else{
                    thisPoint.intensity = 0;
                    laserCloudTemp->push_back(thisPoint);
                }
            }
        }

        *laserCloudOut = *laserCloudTemp;
    }

    void predictCloudBGK(){

        if (predictionEnableFlag == false)
            return;

        int kernelGridLength = int(predictionKernalSize / mapResolution);

        for (int i = 0; i < filterHeightMapArrayLength; ++i){
            for (int j = 0; j < filterHeightMapArrayLength; ++j){

                if (initFlag[i][j] == true)
                    continue;
                PointType testPoint;
                testPoint.x = localMapOrigin.x + i * mapResolution + mapResolution / 2.0;
                testPoint.y = localMapOrigin.y + j * mapResolution + mapResolution / 2.0;
                testPoint.z = robotPoint.z; 
                if (pointDistance(testPoint, robotPoint) > sensorRangeLimit)
                    continue;
                vector<float> xTrainVec;
                vector<float> yTrainVecElev;
                vector<float> yTrainVecOccu;
                for (int m = -kernelGridLength; m <= kernelGridLength; ++m){
                    for (int n = -kernelGridLength; n <= kernelGridLength; ++n){

                        if (std::sqrt(float(m*m + n*n)) * mapResolution > predictionKernalSize)
                            continue;
                        int idx = i + m;
                        int idy = j + n;

                        if (idx < 0 || idy < 0 || idx >= filterHeightMapArrayLength || idy >= filterHeightMapArrayLength)
                            continue;

                        if (initFlag[idx][idy] == true){
                            xTrainVec.push_back(localMapOrigin.x + idx * mapResolution + mapResolution / 2.0);
                            xTrainVec.push_back(localMapOrigin.y + idy * mapResolution + mapResolution / 2.0);
                            yTrainVecElev.push_back(maxHeight[idx][idy]);
                            yTrainVecOccu.push_back(obstFlag[idx][idy] == true ? 1 : 0);
                        }
                    }
                }

                if (xTrainVec.size() == 0)
                    continue;

                Eigen::MatrixXf xTrain = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(xTrainVec.data(), xTrainVec.size() / 2, 2);
                Eigen::MatrixXf yTrainElev = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(yTrainVecElev.data(), yTrainVecElev.size(), 1);
                Eigen::MatrixXf yTrainOccu = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(yTrainVecOccu.data(), yTrainVecOccu.size(), 1);

                vector<float> xTestVec;
                xTestVec.push_back(testPoint.x);
                xTestVec.push_back(testPoint.y);
                Eigen::MatrixXf xTest = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(xTestVec.data(), xTestVec.size() / 2, 2);

                Eigen::MatrixXf Ks;
                covSparse(xTest, xTrain, Ks);

                Eigen::MatrixXf ybarElev = (Ks * yTrainElev).array();
                Eigen::MatrixXf ybarOccu = (Ks * yTrainOccu).array();
                Eigen::MatrixXf kbar = Ks.rowwise().sum().array();

                if (std::isnan(ybarElev(0,0)) || std::isnan(ybarOccu(0,0)) || std::isnan(kbar(0,0)))
                    continue;

                if (kbar(0,0) == 0)
                    continue;

                float elevation = ybarElev(0,0) / kbar(0,0);
                float occupancy = ybarOccu(0,0) / kbar(0,0);

                PointType p;
                p.x = xTestVec[0];
                p.y = xTestVec[1];
                p.z = elevation;
                p.intensity = (occupancy > 0.5) ? 100 : 0;

                laserCloudOut->push_back(p);
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
        dist(xStar/(predictionKernalSize+0.1), xTrain/(predictionKernalSize+0.1), Kxz);
        Kxz = (((2.0f + (Kxz * 2.0f * 3.1415926f).array().cos()) * (1.0f - Kxz.array()) / 3.0f) +
              (Kxz * 2.0f * 3.1415926f).array().sin() / (2.0f * 3.1415926f)).matrix() * 1.0f;
        for (int i = 0; i < Kxz.rows(); ++i)
            for (int j = 0; j < Kxz.cols(); ++j)
                if (Kxz(i,j) < 0) Kxz(i,j) = 0;
    }

    void publishCloud(){
        sensor_msgs::PointCloud2 laserCloudTemp;
        pcl::toROSMsg(*laserCloudOut, laserCloudTemp);
        laserCloudTemp.header.stamp = ros::Time::now();
        laserCloudTemp.header.frame_id = "map";
        pubCloud.publish(laserCloudTemp);
    }

    void publishLaserScan(){

        updateLaserScan();

        laserScan.header.stamp = ros::Time::now();
        pubLaserScan.publish(laserScan);
        std::fill(laserScan.ranges.begin(), laserScan.ranges.end(), laserScan.range_max + 1.0);
    }

    void updateLaserScan(){

        try{listener.lookupTransform("base_link","map", ros::Time(0), transform);}
        catch (tf::TransformException ex){ /*ROS_ERROR("Transfrom Failure.");*/ return; }

        laserCloudObstacles->header.frame_id = "map";
        laserCloudObstacles->header.stamp = 0;
        pcl::PointCloud<PointType> laserCloudTemp;
        pcl_ros::transformPointCloud("base_link", *laserCloudObstacles, laserCloudTemp, listener);

        int cloudSize = laserCloudTemp.points.size();
        for (int i = 0; i < cloudSize; ++i){
            PointType *point = &laserCloudTemp.points[i];
            float x = point->x;
            float y = point->y;
            float range = std::sqrt(x*x + y*y);
            float angle = std::atan2(y, x);
            int index = (angle - laserScan.angle_min) / laserScan.angle_increment;
            laserScan.ranges[index] = std::min(laserScan.ranges[index], range);
        } 
    }

    void pointcloud2laserscanInitialization(){

        laserScan.header.frame_id = "base_link";

        laserScan.angle_min = -M_PI;
        laserScan.angle_max =  M_PI;
        laserScan.angle_increment = 1.0f / 180 * M_PI;
        laserScan.time_increment = 0;

        laserScan.scan_time = 0.1;
        laserScan.range_min = 0.3;
        laserScan.range_max = 100;

        int range_size = std::ceil((laserScan.angle_max - laserScan.angle_min) / laserScan.angle_increment);
        laserScan.ranges.assign(range_size, laserScan.range_max + 1.0);
    }
};






int main(int argc, char** argv){

    ros::init(argc, argv, "tm_mapping");
    
    TMFilter TFilter;

    ROS_INFO("\033[1;32m---->\033[0m Filter Started.");

    ros::spin();

    return 0;
}