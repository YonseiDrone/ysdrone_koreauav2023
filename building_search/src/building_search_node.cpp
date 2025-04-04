#include "building_search.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "building_search_node");
    ros::NodeHandle nh;
    BuildingSearch building_search(nh);

    ros::Timer timer = nh.createTimer(ros::Duration(0.05), &BuildingSearch::command, &building_search);
//    ros::AsyncSpinner spinner(4); // Use 4 threads
//    spinner.start();
    ros::spin();
    // ros::waitForShutdown();

    return 0;
}
