#include "building_search.hpp"

BuildingSearch::BuildingSearch(const ros::NodeHandle& n_private) : nh_(n_private), rate(30)
{
    // ROS params
    nh_.param("/marker_mission_num", marker_mission_num, 3);
    //TODO: 처음 넣어주는 GPS 마지막 좌표로 설정되어야 함.
    nh_.param("/last_goal_x", last_goal_x, 0.0);
    nh_.param("/last_goal_y", last_goal_y, 135.0);
    nh_.param("/last_goal_z", last_goal_z, 0.0);

    // ROS Publisher & Subscriber
	cloud_sub = nh_.subscribe<sensor_msgs::PointCloud2>("/local_pointcloud", 1, boost::bind(&BuildingSearch::cloud_cb, this, _1));
	state_sub = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, boost::bind(&BuildingSearch::pose_cb, this, _1));
	cargo_bool_sub = nh_.subscribe<std_msgs::Bool>("/cargo_mission", 1, boost::bind(&BuildingSearch::cargo_bool_cb, this, _1));
	goal_pos_pub = nh_.advertise<visualization_msgs::MarkerArray>("input/goal_position", 1);
	pub_markers = nh_.advertise<visualization_msgs::MarkerArray>("cluster_markers", 1);
	goal_yaw_pub = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

    // ROS msgs
	current_target_position = geometry_msgs::PoseStamped();
	current_pose = geometry_msgs::PoseStamped();
	
	is_search_done = false;

    while (ros::ok())
    {
        // If object is detected, publish the estimated yaw to the clustered object
        if (is_search_done)        
            turn_to_target_yaw(goal_pos.pose.position.x, goal_pos.pose.position.y, goal_pos.pose.position.z);
    }
}

/// utils
double BuildingSearch::distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
{
	double x2 = pow(p1.pose.position.x - p2.pose.position.x, 2);
	double y2 = pow(p1.pose.position.y - p2.pose.position.y, 2);
	double z2 = pow(p1.pose.position.z - p2.pose.position.z, 2);
	return sqrt(x2 + y2 + z2);
}

double BuildingSearch::distance(const visualization_msgs::Marker& p1, const geometry_msgs::PoseStamped& p2)
{
	double x2 = pow(p1.pose.position.x - p2.pose.position.x, 2);
	double y2 = pow(p1.pose.position.y - p2.pose.position.y, 2);
	double z2 = pow(p1.pose.position.z - p2.pose.position.z, 2);
	return sqrt(x2 + y2 + z2);
}

double BuildingSearch::orientationGap(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
{
    // Get the current orientation and target orientation as tf::Quaternion
    tf::Quaternion current_orientation;
    tf::quaternionMsgToTF(p1.pose.orientation, current_orientation);

    tf::Quaternion target_orientation;
    tf::quaternionMsgToTF(p2.pose.orientation, target_orientation);

    // Calculate the angle difference between the two orientations
    double angle_diff = current_orientation.angle(target_orientation);
    double angle_diff_deg = angle_diff * 180.0 / M_PI;
    return angle_diff_deg;
}

// ROS callback functions
void BuildingSearch::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose = *msg;
}

void BuildingSearch::cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);
    ROS_INFO_STREAM(" Cloud inputs: #" << cloud->size() << " Points");

    if(cloud->size() > 0)
    {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(4.0); // 2m
        ec.setMinClusterSize(10);
        ec.setMaxClusterSize(250);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        ROS_INFO_STREAM(" # of Clusters: " << cluster_indices.size());

        for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
                cloud_cluster->points.push_back(cloud->points[*pit]); //*
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            ROS_INFO_STREAM("Cluster " << (it - cluster_indices.begin()) << ": " << cloud_cluster->points.size() << " points");

            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cloud_cluster, centroid);
            printf("centroid: %f %f %f\n", centroid[0], centroid[1], centroid[2]);

            // Calculate the dimensions of the bounding box
            pcl::PointXYZ min_pt, max_pt;
            pcl::getMinMax3D(*cloud_cluster, min_pt, max_pt);

            // Now you can create and publish a MarkerArray message
            goal_pos.header.stamp = ros::Time::now();
            goal_pos.ns = "cluster_goal";
            goal_pos.id = it - cluster_indices.begin();
            goal_pos.type = visualization_msgs::Marker::CUBE;
            goal_pos.action = visualization_msgs::Marker::ADD;

            /* Make goal_pos to the center of the centroid of clustered object and last goal position */
            goal_pos.pose.position.x = (centroid[0] + last_goal_x) / 2;
            goal_pos.pose.position.y = (centroid[1] + last_goal_y) / 2;
            goal_pos.pose.position.z = (centroid[2] + last_goal_z) / 2;
            goal_pos.scale.x = max_pt.x - min_pt.x;
            goal_pos.scale.y = max_pt.y - min_pt.y;
            goal_pos.scale.z = max_pt.z - min_pt.z;
            goal_pos.color.r = 1.0;
            goal_pos.color.g = 0.0;
            goal_pos.color.b = 0.0;
            goal_pos.color.a = 1.0;
            goal_pos.lifetime = ros::Duration();            

            marker_array.markers.push_back(goal_pos);
        
            // Create a Marker message for this cluster
            visualization_msgs::Marker marker;
            marker = goal_pos;
            marker.header.frame_id = "/local_origin";  // Replace with your frame_id
            marker.ns = "clusters";

            // Append the Marker to the MarkerArray
            marker_array.markers.push_back(marker);
        }
        if (ros::ok() && distance(goal_pos, current_pose) > 0.5)
        {
            goal_pos_pub.publish(marker_array);
            pub_markers.publish(marker_array);
            if (distance(goal_pos, current_pose) < 5.0)
				printf("Distance     : %.4f\n", distance(goal_pos, current_pose));
        }
        else if((distance(goal_pos, current_pose) < 0.5) && !is_search_done)
        {
            is_search_done = true;
            printf("Goal reached to (Centroid + LastGoal)/2\n");
            printf("Goal %f %f %f reached!\n", goal_pos.pose.position.x, goal_pos.pose.position.y, goal_pos.pose.position.z);
        }
    }
    else
    {
        ROS_WARN("Empty input cloud!");
    }
}

// Turn the heading angle of the quadrotor to the target
// Since the local planner(PX4 Avoidance) shows the unexpected heading angle in the last goal position,
// Publish the orientation which is calculated by relative vector between last goal position and the clustered object.
void BuildingSearch::turn_to_target_yaw(double x, double y, double z)
{
	// Flight in Position Mode
	current_target_position.pose.position.x = x;
	current_target_position.pose.position.y = y;
	current_target_position.pose.position.z = z;

	double target_yaw = atan2(
    current_target_position.pose.position.y - last_goal_y,
    current_target_position.pose.position.x - last_goal_x);

	// http://wiki.ros.org/tf2/Tutorials/Quaternions
	tf2::Quaternion q;
	q.setRPY(0.0, 0.0, target_yaw);

	geometry_msgs::Quaternion q_msg;
	tf2::convert(q, q_msg);

	current_target_position.pose.orientation = q_msg;

    while(ros::ok() && !is_cargo_launched)
    {
		//TODO: 마지막 골 지점으로 Local Planner가 계속 이동하려고 하므로, 미션을 마칠 때까지 계속 publish해주어야 함.
			goal_yaw_pub.publish(current_target_position);
			if (orientationGap(current_target_position, current_pose) > 5.0)
					printf("Orientation gap     : %.4f\n", orientationGap(current_target_position, current_pose));
			else
			{
					printf("Target yaw reached!\n");
					ROS_INFO("HOLDING");
                    // Move on to the next mission
                    call_drone_command(marker_mission_num);
			}
    }
}

bool BuildingSearch::call_drone_command(const std::string& data) {
    client = nh_.serviceClient<ysdrone_msgs::DroneCommand>("/drone_command");
    ysdrone_msgs::DroneCommand srv;

    srv.request.command = data;

    if (client.call(srv)) {
        return true;    // The service call was successful
    } else {
        ROS_ERROR("Failed to call service drone_command");
        return false;   // The service call failed
    }
}


void BuildingSearch::cargo_bool_cb(const std_msgs::Bool::ConstPtr& msg)
{
	is_cargo_launched = msg->data;
	if(is_cargo_launched)
	{
		for(int i = 0; i< 5; ++i)
		{
			ROS_INFO("Cargo Launched!");
		}
	}
}

