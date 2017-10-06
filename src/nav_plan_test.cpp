#include "navigation_planner.h"

int main (int argc, char** argv) {
    ros::init (argc, argv, "cloud_sub");
    ros::NodeHandle node_handler;
    ros::Rate loop_rate(1000);
    std::string topic_octomap;
    ros::NodeHandle private_nh("~");
    private_nh.param("topic_octomap",topic_octomap, std::string("/octomap_point_cloud_centers"));
    NavigationPlanner* navigation_planner = new NavigationPlanner(node_handler,topic_octomap);
    navigation_planner->start();
    ROS_INFO("PASSED");
    ros::NodeHandle simple_nh("move_base_simple");
    ros::Subscriber goal_sub = simple_nh.subscribe("goal",1,&NavigationPlanner::startTraversal,navigation_planner);
    // ros::Subscriber goal_sub = simple_nh.subscribe("goal",1,cloud_callback);
    // ros::Subscriber subscriber_node;
    // subscriber_node = node_handler.subscribe<PointCloud>("/octomap_point_cloud_centers", 100, cloud_callback);
    // subscriber_node = node_handler.subscribe("/octomap_full", 10000, retrieveDataFromOctomap);
    // // subscriber_node = node_handler.subscribe("/odom", 1, update_odometry_data);

    ros::spin();   
 }