#include "navigation_planner.h"
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

bool getCoefficients(const geometry_msgs::PoseStamped& pose){

    float x_cordinate = pose.pose.position.x;
    float y_cordinate = pose.pose.position.y;
    float z_cordinate = pose.pose.position.z;

    float resolution = 0.5;
    x_min = x_cordinate - resolution;
    x_max = x_cordinate + resolution;
    y_min = y_cordinate - resolution;
    y_max = y_cordinate + resolution;
    z_min = z_cordinate - resolution;
    z_max = z_cordinate + resolution;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (z_min, z_max);in
    pass.filter (*cloud_filtered);

    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (y_min, y_max);
    pass.filter (*cloud_filtered);

    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (x_min, x_max);
    pass.filter (*cloud_filtered);

    int ok_count;
    int occupied_count;


    for(pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud_filtered->iterator; it!= cloud_filtered->end(); it++){
        if(it->z > z_cordinate + 0.05){
            occupied_count++;
        }else{
            ok_count++;
        }   
        //cout << it->x << ", " << it->y << ", " << it->z << endl;
    } 
    cout << ok_count << ", " << occupied_count << endl;

  //pass.setFilterLimitsNegative (true);
    
    /*
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

    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0) {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        //return (-1);
    }
    */


    // std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
    //                                   << coefficients->values[1] << " "
    //                                   << coefficients->values[2] << " " 
    //                                   << coefficients->values[3] << std::endl;

    // std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
    // for (size_t i = 0; i < inliers->indices.size (); ++i)
    //     std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
    //                                            << cloud->points[inliers->indices[i]].y << " "
    //                                            << cloud->points[inliers->indices[i]].z << std::endl;

    //return (0);

}

void cloud_callback(const PointCloud::ConstPtr& msg){
//   printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    cloud->width  = msg->width;
    cloud->height = msg->height;
    cloud->points.resize (cloud->width * cloud->height);

    int i=0;
    BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points){
        printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
        cloud->points[i].x = pt.x;
        cloud->points[i].y = pt.y;
        cloud->points[i].z = pt.z;
        i++;
    }
}


int main (int argc, char** argv) {
    ros::init (argc, argv, "cloud_sub");
    ros::NodeHandle node_handler;
    ros::Rate loop_rate(1000);
    std::string topic_octomap;
    ros::NodeHandle private_nh("~");
    private_nh.param("topic_octomap",topic_octomap, std::string("/octomap_binary"));
    
    NavigationPlanner* navigation_planner = new NavigationPlanner(node_handler,topic_octomap);
    // navigation_planner->start();
    // ROS_INFO("PASSED");
    ros::NodeHandle simple_nh("move_base_simple");
    // ros::Subscriber goal_sub = simple_nh.subscribe("goal",1,&NavigationPlanner::neighbourhoodCallback,navigation_planner);
    ros::Subscriber goal_sub = simple_nh.subscribe("goal",1,cloud_callback);
    // ros::Subscriber subscriber_node;
    // subscriber_node = node_handler.subscribe<PointCloud>("/octomap_point_cloud_centers", 100, cloud_callback);
    // subscriber_node = node_handler.subscribe("/octomap_full", 10000, retrieveDataFromOctomap);
    // // subscriber_node = node_handler.subscribe("/odom", 1, update_odometry_data);

    ros::spin();   
 }