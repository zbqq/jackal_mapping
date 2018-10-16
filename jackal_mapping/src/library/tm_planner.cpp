#include <pluginlib/class_list_macros.h>
#include "planner/tm_planner.h"
#include <unistd.h>

PLUGINLIB_EXPORT_CLASS(tm_planner::TMPlanner, nav_core::BaseGlobalPlanner)
 
	
namespace tm_planner {

	TMPlanner::TMPlanner (){}

	TMPlanner::TMPlanner(const std::string name, costmap_2d::Costmap2DROS* costmap_ros){
		initialize(name, costmap_ros);
	}

	void TMPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){

		subPath = nh.subscribe("/global_path", 1, &TMPlanner::pathHandler, this);
		pubGoal = nh.advertise<geometry_msgs::PoseStamped>("/prm_goal", 2);
		subTwistCommand = nh.subscribe<nav_msgs::Path>("/move_base/TrajectoryPlannerROS/local_plan", 1, &TMPlanner::twistCommandHandler, this);
        pubTwistCommand = nh.advertise<nav_msgs::Path>("/twist_command", 1);

		ROS_INFO("----> Traversability Plugin Started.");
	}

	void TMPlanner::twistCommandHandler(const nav_msgs::Path::ConstPtr& pathMsg){

		try{ listener.lookupTransform("map","base_link", ros::Time(0), transform); } 
        catch (tf::TransformException ex){ /*ROS_ERROR("Transfrom Failure.");*/ return; }

        nav_msgs::Path outTwist = *pathMsg;

        for (int i = 0; i < outTwist.poses.size(); ++i)
            outTwist.poses[i].pose.position.z = transform.getOrigin().z() + 1.5;

        pubTwistCommand.publish(outTwist);
    }

	void TMPlanner::pathHandler(const nav_msgs::Path::ConstPtr& pathMsg){
		globalPath = *pathMsg;
	}

	bool TMPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan){
		
		ROS_INFO("Goal Received at: [%lf, %lf, %lf]", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
		pubGoal.publish(goal);

		if (globalPath.poses.size() == 0){
			pubGoal.publish(goal);
			return false;
		}
		ROS_INFO("A Valid Path Received!");

		geometry_msgs::PoseStamped this_pos = goal;
		for (int i = 0; i < globalPath.poses.size(); ++i){
			this_pos.pose.position.x = globalPath.poses[i].pose.position.x;
			this_pos.pose.position.y = globalPath.poses[i].pose.position.y;
			this_pos.pose.position.z = globalPath.poses[i].pose.position.z;
			this_pos.pose.orientation.x = globalPath.poses[i].pose.orientation.x;
			this_pos.pose.orientation.y = globalPath.poses[i].pose.orientation.y;
			this_pos.pose.orientation.z = globalPath.poses[i].pose.orientation.z;
			this_pos.pose.orientation.w = globalPath.poses[i].pose.orientation.w;

			plan.push_back(this_pos);
		}

		plan.back().pose.orientation = goal.pose.orientation;

		globalPath.poses.clear();

		return true; 
	}

};