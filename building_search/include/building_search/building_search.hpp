#ifndef BUILDING_SEARCH_H
#define BUILDING_SEARCH_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
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
	BuildingSearch(const ros::NodeHandle& n_private);
	void turn_to_target_yaw(double x, double y, double z);

	double last_goal_x;
	double last_goal_y;
	double last_goal_z;
	bool is_search_done, is_cargo_launched;
	double marker_mission_num;

private:
/// utils
	double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);
	double distance(const visualization_msgs::Marker& p1, const geometry_msgs::PoseStamped& p2);
	double orientationGap(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);
/// ROS callbacks
	void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);
	void cargo_bool_cb(const std_msgs::Bool::ConstPtr &msg);
	bool call_drone_command(const double& data);
/// ROS params
	ros::NodeHandle nh_;
	ros::Rate rate;
	ros::Publisher goal_pos_pub, pub_markers, goal_yaw_pub;
	ros::Subscriber cloud_sub, state_sub, cargo_bool_sub;
	ros::ServiceClient client;
	geometry_msgs::PoseStamped current_pose;
	visualization_msgs::MarkerArray marker_array;
	geometry_msgs::PoseStamped current_target_position;
	visualization_msgs::Marker goal_pos;



};

#endif //BUILDING_SEARCH_H