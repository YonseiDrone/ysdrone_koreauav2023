#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <vector>
#include <string>


int main(int argc, char** argv) {
    ros::init(argc, argv, "yaml_to_gps");
    ros::NodeHandle nh;

    XmlRpc::XmlRpcValue WPT_1, WPT_2, WPT_3;
    nh.getParam("WPT_1", WPT_1);
    nh.getParam("WPT_2", WPT_2);
    nh.getParam("WPT_3", WPT_3);

    std::vector<XmlRpc::XmlRpcValue> waypoints = {WPT_1, WPT_2, WPT_3};

    ros::Publisher pub1 = nh.advertise<sensor_msgs::NavSatFix>("WPT_1_lla", 1000);
    ros::Publisher pub2 = nh.advertise<sensor_msgs::NavSatFix>("WPT_2_lla", 1000);
    ros::Publisher pub3 = nh.advertise<sensor_msgs::NavSatFix>("WPT_3_lla", 1000);

    ros::Rate loop_rate(1);  // publish messages at 1Hz
    while (ros::ok()) {
        for (size_t i = 0; i < waypoints.size(); ++i) {
            sensor_msgs::NavSatFix msg;
            msg.latitude = static_cast<double>(waypoints[i][0]);
            msg.longitude = static_cast<double>(waypoints[i][1]);
            msg.altitude = static_cast<double>(waypoints[i][2]);

            if (i == 0) pub1.publish(msg);
            if (i == 1) pub2.publish(msg);
            if (i == 2) pub3.publish(msg);

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    return 0;
}
