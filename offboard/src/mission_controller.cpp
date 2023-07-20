#include <ros/ros.h>
#include <math.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>

#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>

#include <ysdrone_msgs/DroneCommand.h>

/// Mathematical conversion functions
double rad2deg(double radian) {
    return radian * 180 / M_PI;
}

double deg2rad(double degree) {
    return degree * M_PI /180;
}

tf2::Quaternion to_quaternion(double yaw, double pitch, double roll) {
    // double cy = cos(yaw * 0.5);
    // double sy = sin(yaw * 0.5);
    // double cp = cos(pitch * 0.5);
    // double sp = sin(pitch * 0.5);
    // double cr = cos(roll * 0.5);
    // double sr = sin(roll * 0.5);

    // double x = sr*cp*cy - cr*sp*sy;
    // double y = cr*sp*cy + sr*cp*sy;
    // double z = cr*cp*sy - sr*sp*cy;
    // double w = cr*cp*cy + sr*sp*sy;

    tf2::Quaternion q;
    q.setRPY(pitch, roll, yaw);

    return q;
}

tf::Vector3 to_euler_angles(tf::Quaternion q) {
    // roll(x-axis rotation)
    double sinr_cosp = 2 * (q.w()*q.x() + q.y()*q.z());
    double cosr_cosp = 1 - 2*(q.x()*q.x() + q.y()*q.y());
    double roll = atan2(sinr_cosp, cosr_cosp);

    // pitch(y-axis rotation)
    double sinp = 2*(q.w()*q.y() - q.z()*q.x());
    double pitch;
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);
    
    // yaw(z-axis rotation)
    double siny_cosp = 2*(q.w()*q.z() + q.x()*q.y());
    double cosy_cosp = 1 - 2*(q.y()*q.y() + q.z()*q.z());
    double yaw = atan2(siny_cosp, cosy_cosp);

    return tf::Vector3(roll, pitch, yaw);
}

/// Main Controller

class MissionControl {
public:
    MissionControl(const ros::NodeHandle nh_private) : nh_(nh_private) {


        state_sub = nh_.subscribe("/mavros/state", 1, &MissionControl::state_cb, this);
        pose_sub = nh_.subscribe("/mavros/local_position/pose", 1, &MissionControl::pose_cb, this);

        destination_command_sub = nh_.subscribe("/destination_command", 1, &MissionControl::destination_command_cb, this);
        landing_velocity_sub = nh_.subscribe("/landing_velocity", 1, &MissionControl::landing_vel_cb, this);
        isly_destination_command_sub = nh_.subscribe("/isly_destination_command", 1, &MissionControl::isly_destination_command_cb, this);
        
        mavros_cmd_pub = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
        avoidance_pos_pub = nh_.advertise<visualization_msgs::MarkerArray>("input/goal_position", 1);
        landing_velocity_pub = nh_.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 1);

        drone_command_server = nh_.advertiseService("drone_command", &MissionControl::cmdsrv_cb, this);
        drone_command_client = nh_.serviceClient<ysdrone_msgs::DroneCommand>("drone_command");
    }
    // ROS Nodehandle
    ros::NodeHandle nh_;

    void state_cb(const mavros_msgs::State::ConstPtr &msg) {
        current_state = *msg;
    }

    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
        current_pose = *msg;
    }

    void destination_command_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
        destination_marker.ns="";
        destination_marker.id = 0;
        destination_marker.type = visualization_msgs::Marker::SPHERE;
        destination_marker.action = visualization_msgs::Marker::ADD;
        destination_marker.scale.x = 1.0;
        destination_marker.scale.y = 1.0;
        destination_marker.scale.z = 1.0;
        destination_marker.color.a = 1.0;
        destination_marker.color.r = 0.0;
        destination_marker.color.g = 0.0;
        destination_marker.color.b = 1.0;
        // Assign
        destination_marker.pose.position.x = msg->pose.position.x;
        destination_marker.pose.position.y = msg->pose.position.y;
        destination_marker.pose.position.z = msg->pose.position.z;
        // Push back
        destination_marker_array.markers.clear();
        destination_marker_array.markers.push_back(destination_marker);
    }

    void landing_vel_cb(const geometry_msgs::Twist::ConstPtr &msg) {
        landing_vel = *msg;
        landing_vel.linear.z = -1;
    }

    void isly_destination_command_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
        isly_destination_marker.ns="";
        isly_destination_marker.id = 0;
        isly_destination_marker.type = visualization_msgs::Marker::SPHERE;
        isly_destination_marker.action = visualization_msgs::Marker::ADD;
        isly_destination_marker.scale.x = 1.0;
        isly_destination_marker.scale.y = 1.0;
        isly_destination_marker.scale.z = 1.0;
        isly_destination_marker.color.a = 1.0;
        isly_destination_marker.color.r = 0.0;
        isly_destination_marker.color.g = 0.0;
        isly_destination_marker.color.b = 1.0;
        // Assign
        isly_destination_marker.pose.position.x = msg->pose.position.x;
        isly_destination_marker.pose.position.y = msg->pose.position.y;
        isly_destination_marker.pose.position.z = msg->pose.position.z;
        // Push back
        isly_destination_marker_array.markers.clear();
        isly_destination_marker_array.markers.push_back(destination_marker);
    }

    bool cmdsrv_cb(ysdrone_msgs::DroneCommand::Request &req, ysdrone_msgs::DroneCommand::Response &res) {
        // Initially, cmd_state = -1
        cmd_state = req.command;
        switch(cmd_state) {
            case 0:
                resp.mode = "Offboard Mode";
                resp.res = true;
            case 1:
                resp.mode = "Avoidance Mode";
                resp.res = true;
                break;
            case 2:
                resp.mode = "Building Searching Mode";
                resp.res = true;
                break;
            case 3:
                resp.mode = "Cross Detection Mode";
                resp.res = true;
                break;
            case 4:
                resp.mode = "Cargo Launching Mode";
                resp.res = true;
                break;
            case 5:
                resp.mode = "Of course I Still Love You";
                resp.res = true;
                break;
            case 6:
                resp.mode = "Safety Landing Mode";
                resp.res = true;
                break;
            default:
                ROS_WARN("Invalid command received!");
            ROS_INFO("Received request: %d", cmd_state);
            ROS_INFO("Current Mode: %s, Enable: %s", resp.mode, resp.res?"true":"false");
        }
    }

    void main_controller(const ros::TimerEvent& event) {
        switch(cmd_state) {
            // Arming, Offboard, Take off
            case 0:
                break;
            // Local Planner with obstacle avoidance
            case 1:
                avoidance_pos_pub.publish(destination_marker_array);
                break;
            // Building searching
            case 2:
                break;
            // Cross marker detection
            case 3:
                break;
            // Cargo launching
            case 4:
                break;
            // Return to Home
            case 5:
                avoidance_pos_pub.publish(isly_destination_marker_array);
                break;
            // Safety Landing
            case 6:
                landing_velocity_pub.publish(landing_vel);
                break;
        }        
    }

    mavros_msgs::State current_state;
    ros::ServiceClient drone_command_client;

private:
    geometry_msgs::PoseStamped current_pose;
    geometry_msgs::Twist landing_vel;

    visualization_msgs::Marker destination_marker, isly_destination_marker;
    visualization_msgs::MarkerArray destination_marker_array, isly_destination_marker_array;

    ros::Subscriber destination_command_sub, state_sub, pose_sub, landing_velocity_sub, isly_destination_command_sub;
    ros::Publisher mavros_cmd_pub, avoidance_pos_pub, landing_velocity_pub;
    ros::ServiceServer drone_command_server;

    ysdrone_msgs::DroneCommandResponse resp;
    int cmd_state = -1;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "mission_controller");
    ros::NodeHandle nh("~");
    MissionControl mission_controller(nh);
    ros::Rate rate(100); // 10 Hz
    while (ros::ok() && !mission_controller.current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_WARN("Mission Controller Initialized!");
    ros::Timer timer = nh.createTimer(ros::Duration(0.05), &MissionControl::main_controller, &mission_controller);

    ros::AsyncSpinner spinner(4); // User 4 threads
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
