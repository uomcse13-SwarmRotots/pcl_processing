#include "navigation_planner.h"



int planerCoefficientApproximation(pcl::PointCloud<pcl::PointXYZ>::Ptr& plane_cloud){
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (plane_cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0){
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return (-1);
    }
    std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                        << coefficients->values[1] << " "
                                        << coefficients->values[2] << " " 
                                        << coefficients->values[3] << std::endl;

    // std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
    // for (size_t i = 0; i < inliers->indices.size (); ++i)
    //     std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
    //                                             << cloud->points[inliers->indices[i]].y << " "
    //                                             << cloud->points[inliers->indices[i]].z << std::endl;

    return (0);
}













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
    ros::Subscriber goal_sub = simple_nh.subscribe("goal",1,&NavigationPlanner::segmentBoundingCube,navigation_planner);
    // ros::Subscriber goal_sub = simple_nh.subscribe("goal",1,cloud_callback);
    // ros::Subscriber subscriber_node;
    // subscriber_node = node_handler.subscribe<PointCloud>("/octomap_point_cloud_centers", 100, cloud_callback);
    // subscriber_node = node_handler.subscribe("/octomap_full", 10000, retrieveDataFromOctomap);
    // // subscriber_node = node_handler.subscribe("/odom", 1, update_odometry_data);

    ros::spin();   
 }