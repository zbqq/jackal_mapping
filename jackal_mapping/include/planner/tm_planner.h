#include "utility.h"

#ifndef tm_planner_CPP
#define tm_planner_CPP

namespace tm_planner {


    class TMPlanner : public nav_core::BaseGlobalPlanner {

    public:
        ros::NodeHandle nh;

        tf::TransformListener listener;
        tf::StampedTransform transform;

        ros::Subscriber subPath;
        ros::Publisher pubGoal;

        ros::Subscriber subTwistCommand;
        ros::Publisher pubTwistCommand;

        nav_msgs::Path globalPath;

        std::mutex mtx;

        TMPlanner(); 
        TMPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        void pathHandler(const nav_msgs::Path::ConstPtr& pathMsg);
        
        void twistCommandHandler(const nav_msgs::Path::ConstPtr& pathMsg);

        bool makePlan(const geometry_msgs::PoseStamped& start, 
                        const geometry_msgs::PoseStamped& goal, 
                        std::vector<geometry_msgs::PoseStamped>& plan);  

    };


};


#endif