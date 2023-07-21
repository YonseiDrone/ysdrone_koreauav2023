#ifndef BUILDING_SEARCH_H
#define BUILDING_SEARCH_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <math.h>

#include "ysdrone_msgs/DroneCommand.h" 

class BuildingSearch
{
public:
	BuildingSearch(const ros::NodeHandle& nh_private);
	void turn_to_target_yaw(double x, double y, double z);
	void move_to_target(double x, double y, double z);
	void command(const ros::TimerEvent& event);
/// utils
	double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);
	double distance(const visualization_msgs::Marker& p1, const geometry_msgs::PoseStamped& p2);
	double orientationGap(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);
/// ROS callbacks
	void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);
	// bool srv_cb(ysdrone_msgs::DroneCommand::Request &req, ysdrone_msgs::DroneCommand::Response &res);
	// bool call_drone_command(const double& data);
	void mission_cb(const std_msgs::Float32::ConstPtr &msg);

private:
	double calc_xy_err(const geometry_msgs::Point &pos, double last_goal_x, double last_goal_y);
	double calc_z_err(float z, double last_goal_z);
// paramters
	bool srv_mode;
	double last_goal_x;
	double last_goal_y;
	double last_goal_z;
	double building_search_mission, marker_mission;

/// ROS
// publisher & subscriber
	ros::NodeHandle nh_;
	ros::Rate rate;
	ros::Publisher goal_pos_pub, goal_yaw_pub;
	ros::Subscriber cloud_sub, pos_sub, mission_sub;
// services
	ros::ServiceClient client;
	ros::ServiceServer server;
// msgs
	geometry_msgs::PoseStamped current_pose;
	visualization_msgs::MarkerArray marker_array;
	geometry_msgs::PoseStamped current_target_position;
	visualization_msgs::Marker goal_pos;
	// ysdrone_msgs::DroneCommand srv;
	int mission = -1;
	int searching_status;
	bool last_goal_reached, init_mission_sub;



};

#endif //BUILDING_SEARCH_H