#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "nav_msgs/OccupancyGrid.h"

void mapConvert(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  ROS_INFO("I heard: [%d]",msg->info.width);
//   map_pub.publish(msg->info.width);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_converter");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/projected_map", 1000, mapConvert);
//   ros::Publisher map_pub = n.advertise<int>("mapconverted", 1000);
  ros::Rate loop_rate(10);

 int count = 0;
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::spin();
  return 0;
}