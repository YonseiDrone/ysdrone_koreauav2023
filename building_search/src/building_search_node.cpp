#include "building_search.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "building_search_node");
    ros::NodeHandle nh("~");
    BuildingSearch building_search(nh);
    ros::spin();
    return 0;
}
