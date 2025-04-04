#include "building_search.hpp"


BuildingSearch::BuildingSearch(const ros::NodeHandle& nh_private) : nh_(nh_private), rate(30)
{
    // ROS params
    nh_.param("/srv_mode", srv_mode, true);
    nh_.param("/bulding_search_mission", building_search_mission, 2.0);
    nh_.param("/marker_mission", marker_mission, 3.0);

    nh_.param("/destination_3_pose_x", last_goal_x, 0.0);
    nh_.param("/destination_3_pose_y", last_goal_y, 0.0);
    nh_.param("/destination_z", last_goal_z, 2.5);
    ROS_INFO("[Building Search] Initialized");
    ROS_INFO("Last Goal %f, %f, %f", last_goal_x, last_goal_y, last_goal_z);

    // ROS Publisher & Subscriber
	cloud_sub = nh_.subscribe<sensor_msgs::PointCloud2>("/local_pointcloud", 10, boost::bind(&BuildingSearch::cloud_cb, this, _1));
	pos_sub = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, boost::bind(&BuildingSearch::pose_cb, this, _1));

	goal_pos_pub = nh_.advertise<visualization_msgs::MarkerArray>("/input/goal_position", 1);
	goal_yaw_pub = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    colored_cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("/clusters", 1);
    centroid_marker_pub = nh_.advertise<visualization_msgs::Marker>("/centroid_marker", 1);

    // ROS service for DroneCommand
    // client = nh_.serviceClient<ysdrone_msgs::DroneCommand>("drone_command");
    // server = nh_.advertiseService("drone_command", &BuildingSearch::srv_cb, this);
    mission_sub = nh_.subscribe<std_msgs::Float32>("/mission", 1, boost::bind(&BuildingSearch::mission_cb, this, _1));

    // ROS msgs
	current_target_position = geometry_msgs::PoseStamped();
	current_pose = geometry_msgs::PoseStamped();
	
    searching_status = 0;
    last_goal_reached = false;
    init_mission_sub = false;
}

void BuildingSearch::command(const ros::TimerEvent& event)
{
    if(last_goal_reached)
    {
        if(!init_mission_sub) {
	        ROS_INFO("Last goal reached! current mission #%d", srv_mode);
            ROS_INFO("Building search Ready", srv_mode, mission);
            init_mission_sub = true;
        }
        // Auto Mode    or
        // Service Mode => Is request.command same as buidling search mission number?
        if(!srv_mode || mission == building_search_mission)
        {
            if(searching_status == 0)
            {
                ROS_INFO("# marker: %ld", marker_array.markers.size());
                if(marker_array.markers.size() > 0)
                {  
                    ROS_INFO("Turn to Target yaw");
                    turn_to_target_yaw(marker_array.markers[0].pose.position.x, marker_array.markers[0].pose.position.y, marker_array.markers[0].pose.position.z);
                }
            }
            else if(searching_status == 1)
            {
                ROS_INFO("searching status: 1");
                ROS_INFO("# marker: %ld", marker_array.markers.size());
                ROS_INFO("Goal x: %f, y: %f, z: %f", marker_array.markers[0].pose.position.x, marker_array.markers[0].pose.position.y, marker_array.markers[0].pose.position.z);
                goal_pos_pub.publish(marker_array);
                if ((distance(marker_array.markers[0], current_pose) < 3.0) && (distance(marker_array.markers[0], current_pose) > 0.5))
                    ROS_INFO("Distance     : %.4f\n", distance(marker_array.markers[0], current_pose));
                else if(distance(marker_array.markers[0], current_pose) < 0.5)
                {
                    ROS_INFO("Goal %f %f %f reached!\n", marker_array.markers[0].pose.position.x, marker_array.markers[0].pose.position.y, marker_array.markers[0].pose.position.z);
                    ros::spinOnce();
                    searching_status += 1;
                }
            }
            // If object is detected, publish the estimated yaw to the clustered object
            else if (searching_status == 2)
            {   
                ROS_INFO("searching status: 2");
                ROS_INFO("Goal x: %f, y: %f, z: %f", marker_array.markers[0].pose.position.x, marker_array.markers[0].pose.position.y, marker_array.markers[0].pose.position.z);
                move_to_target(marker_array.markers[0].pose.position.x, marker_array.markers[0].pose.position.y, marker_array.markers[0].pose.position.z);
                ROS_INFO("[Building Search] Done");
                searching_status += 1;
                last_goal_reached = false;
            }
        }
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
    double current_roll, current_pitch, current_yaw;
    tf::Matrix3x3(current_orientation).getRPY(current_roll, current_pitch, current_yaw);

    tf::Quaternion target_orientation;
    tf::quaternionMsgToTF(p2.pose.orientation, target_orientation);
    double target_roll, target_pitch, target_yaw;
    tf::Matrix3x3(target_orientation).getRPY(target_roll, target_pitch, target_yaw);

    // Calculate the angle difference between the two orientations
    double yaw_diff = abs(current_yaw - target_yaw);
    return yaw_diff;
}

double BuildingSearch::calc_xy_err(const geometry_msgs::Point &pos, double last_goal_x, double last_goal_y)
{
    double xy_err = sqrt(pow((pos.x - last_goal_x), 2) + pow((pos.y - last_goal_y), 2));
    return xy_err;
}

double BuildingSearch::calc_z_err(float z, double last_goal_z)
{
    double z_err = sqrt(pow((z - last_goal_z), 2));
    return z_err;
}

// ROS callback functions
void BuildingSearch::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose = *msg;
    if ((calc_xy_err(current_pose.pose.position, last_goal_x, last_goal_y) < 0.3) && (calc_z_err(current_pose.pose.position.z, last_goal_z) < 0.2)) {
        last_goal_reached = true;
        // ROS_WARN("[Building Search] Last WPT Reached!");
    }
}

void BuildingSearch::cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    colored_cloud->header = cloud->header;
    // ROS_INFO_STREAM(" Cloud inputs: #" << cloud->size() << " Points");

    if(cloud->size() > 0)
    {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(4.0); // 2m
        ec.setMinClusterSize(30);
        ec.setMaxClusterSize(250);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        // ROS_INFO_STREAM(" # of Clusters: " << cluster_indices.size());

        for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);

            uint8_t r = rand() % 256;
            uint8_t g = rand() % 256;
            uint8_t b = rand() % 256;

            for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            {
                cloud_cluster->points.push_back(cloud->points[*pit]); //*

                pcl::PointXYZRGB colored_point;
                colored_point.x = cloud->points[*pit].x;
                colored_point.y = cloud->points[*pit].y;
                colored_point.z = cloud->points[*pit].z;
                uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
                colored_point.rgb = *reinterpret_cast<float*>(&rgb);
                colored_cloud->points.push_back(colored_point);
            }
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            // ROS_INFO_STREAM("Cluster " << (it - cluster_indices.begin()) << ": " << cloud_cluster->points.size() << " points");
            pcl::toROSMsg(*colored_cloud, colored_cloud_msg);
            colored_cloud_pub.publish(colored_cloud_msg);

            pcl::compute3DCentroid(*cloud_cluster, centroid);
            // printf("centroid: %f %f %f\n", centroid[0], centroid[1], centroid[2]);

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
            goal_pos.pose.position.z = (centroid[2] > last_goal_z) ? last_goal_z : centroid[2];
            goal_pos.scale.x = max_pt.x - min_pt.x;
            goal_pos.scale.y = max_pt.y - min_pt.y;
            goal_pos.scale.z = max_pt.z - min_pt.z;
            goal_pos.color.r = 1.0;
            goal_pos.color.g = 0.0;
            goal_pos.color.b = 0.0;
            goal_pos.color.a = 1.0;
            goal_pos.lifetime = ros::Duration(0.2);

            centroid_marker.header.frame_id = input->header.frame_id;
            centroid_marker.header.stamp = ros::Time::now();     
            centroid_marker.ns = "centroid_marker";
            centroid_marker.id = it - cluster_indices.begin();
            centroid_marker.type = visualization_msgs::Marker::SPHERE;
            centroid_marker.action = visualization_msgs::Marker::ADD;
            centroid_marker.pose.position.x = centroid[0];
            centroid_marker.pose.position.y = centroid[1];
            centroid_marker.pose.position.z = centroid[2];
            centroid_marker.pose.orientation.x = 0.0;
            centroid_marker.pose.orientation.y = 0.0;
            centroid_marker.pose.orientation.z = 0.0;
            centroid_marker.pose.orientation.w = 1.0;
            centroid_marker.scale.x = 0.1;
            centroid_marker.scale.y = 0.1;
            centroid_marker.scale.z = 0.1;
            centroid_marker.color.r = 1.0;
            centroid_marker.color.g = 1.0;
            centroid_marker.color.b = 0.0;
            centroid_marker.color.a = 1.0;

            centroid_marker_pub.publish(centroid_marker);

            marker_array.markers.clear();
            marker_array.markers.push_back(goal_pos);
        }
    }
    else
    {
        ROS_WARN("Empty input cloud!");
    }
}

void BuildingSearch::mission_cb(const std_msgs::Float32::ConstPtr &msg)
{
    mission = msg->data;
}

// Stay in last goal position but turn yaw to target
void BuildingSearch::turn_to_target_yaw(double x, double y, double z)
{
	// Flight in Position Mode
	current_target_position.pose.position.x = current_pose.pose.position.x;
	current_target_position.pose.position.y = current_pose.pose.position.y;
	current_target_position.pose.position.z = z;

	double target_yaw = atan2(y - current_pose.pose.position.x, x - current_pose.pose.position.y);

	// http://wiki.ros.org/tf2/Tutorials/Quaternions
	tf2::Quaternion q;
	q.setRPY(0.0, 0.0, target_yaw);

	geometry_msgs::Quaternion q_msg;
	tf2::convert(q, q_msg);

	current_target_position.pose.orientation = q_msg;

    //TODO: 마지막 골 지점으로 Local Planner가 계속 이동하려고 하므로, 미션을 마칠 때까지 계속 publish해주어야 함.
    while ((distance(current_target_position, current_pose) > 0.5) && (orientationGap(current_target_position, current_pose) > 5.0))
    {
        goal_yaw_pub.publish(current_target_position);
        ROS_INFO("Distance: %.4f    Orientation gap: %.4f\n", distance(current_target_position, current_pose), orientationGap(current_target_position, current_pose));
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Target yaw reached!\n");
    searching_status += 1;
}

/* Turn the heading angle of the quadrotor to the target
* Since the local planner(PX4 Avoidance) shows the unexpected heading angle in the last goal position,
* Publish the orientation which is calculated by relative vector between last goal position and the clustered object.
*/

void BuildingSearch::move_to_target(double x, double y, double z)
{
	// Flight in Position Mode
	current_target_position.pose.position.x = x;
	current_target_position.pose.position.y = y;
	current_target_position.pose.position.z = z;

	double target_yaw = atan2(
    centroid[1] - current_pose.pose.position.y,
    centroid[0] - current_pose.pose.position.x);

	// http://wiki.ros.org/tf2/Tutorials/Quaternions
	// tf2::Quaternion q;
	// q.setRPY(0.0, 0.0, target_yaw);

	// geometry_msgs::Quaternion q_msg;
	// tf2::convert(q, q_msg);
    tf::Quaternion quaternion = tf::createQuaternionFromYaw(target_yaw);

	// current_target_position.pose.orientation = q_msg;
	current_target_position.pose.orientation.x = quaternion.x();
	current_target_position.pose.orientation.y = quaternion.y();
	current_target_position.pose.orientation.z = quaternion.z();
	current_target_position.pose.orientation.w = quaternion.w();

    while((distance(current_target_position, current_pose) > 0.5) && (orientationGap(current_target_position, current_pose)) > 0.008)
    {
        goal_yaw_pub.publish(current_target_position);
        if (orientationGap(current_target_position, current_pose) > 5.0)
            ROS_INFO("Orientation gap: %.4f\n", orientationGap(current_target_position, current_pose));
    }
    ROS_INFO("Move to target Done!");
}