
#include <iostream>
#include <string>
#include <sstream>
#include <math.h>
#include <ros/ros.h>

#include <bitset>
#include <map>
#include <iostream>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>


#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <fstream>

#include <octomap_msgs/GetOctomap.h>

void handleSetOctomap(const octomap_msgs::OctomapConstPtr& msg){
// read all the message info here //
}

int main (int argc, char** argv) {
    ros::init (argc, argv, "cloud_sub");
    ros::NodeHandle node_handler;
    ros::Rate loop_rate(10);
    ros::Subscriber subscriber_node;
    subscriber_node = node_handler.subscribe("/octomap_full", 100, handleSetOctomap);
    ros::spin();   
 }
