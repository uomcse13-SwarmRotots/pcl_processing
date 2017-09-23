#ifndef NAVIGATION_PLANNER_INCLUDE
#define NAVIGATION_PLANNER_INCLUDE

#include <iostream>
#include <string>
#include <sstream>
#include <math.h>

#include <octomap/OcTree.h>

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
#include <queue>

#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <fstream>

#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>

#include <iostream>
#include <vector>
#include <ctime>
#include <pcl/kdtree/kdtree_flann.h>
#include <octomap_msgs/GetOctomap.h>
#include <algorithm>

#include <boost/lambda/lambda.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/tuple/tuple_io.hpp>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include<octomap/OcTreeBase.h>

const float  PI_F=3.14159265358979f;

/*
    namespace initialization
*/
using octomap_msgs::GetOctomap;
using namespace std;
using namespace octomap;
using namespace boost::lambda;

namespace {
  typedef float coord_t;
  typedef boost::tuple<coord_t,coord_t,coord_t> point_t;

  coord_t distance_sq(const point_t& a, const point_t& b) { // or boost::geometry::distance
    coord_t x = a.get<0>() - b.get<0>();
    coord_t y = a.get<1>() - b.get<1>();
    coord_t z = a.get<2>() - b.get<2>();
    return x*x + y*y + z*z;
  }
}

/*
    define structs
*/

struct MapIndex{
    float x, y, z;
    MapIndex()
    :x(0), y(0), z(0){
    }
    MapIndex(float x_, float y_, float z_)
    :x(x_), y(y_), z(z_){
    }
};

struct Graph_Node{
    float x_cordinate;
    float y_cordinate;
    float z_cordinate;
    Graph_Node *predecessor;
    int path_cost;
    float priority;
};

/*
    operator override
*/
bool operator<(const MapIndex &v1, const MapIndex &v2){
    if (v1.z > v2.z)
        return false;
    if (v1.z < v2.z)
        return true;
    if (v1.y > v2.y)
        return false;
    if (v1.y < v2.y)
        return true;
    if (v1.x < v2.x)
        return true;
    return false;
}

/*
    define templates
*/
template<typename Val> struct Array3D{
    typedef std::map<MapIndex, Val> Data;
    Data data;
    Val defaultValue;
    const Val& getValue(float x, float y, float z) const{
        MapIndex index(x, y, z);
        typename Data::const_iterator found = data.find(index);
        if (found == data.end())
            return defaultValue;
        return found->second;
    }
    void setValue(float x, float y, float z, const Val &val){
        data.insert(std::make_pair(MapIndex(x, y, z), val));
    }
    bool hasValue(float x, float y, float z) const{
        typename Data::const_iterator found = data.find(MapIndex(x, y, z));
        return found != data.end();
    }
    Array3D(const Val& defaultValue_ = Val())
    :defaultValue(defaultValue_){
    }
};

class NavigationPlanner{
    private:
        /*
            variables
        */
        struct Graph_Node *start_node;
        struct Graph_Node *end_node;
        struct Graph_Node *breadth_array_free_cells[1000];

        Array3D<struct Graph_Node*> found_nodes;
        Array3D<float> occupied_points;
        Array3D<float> checked_points;
        
        queue<struct Graph_Node*> node_queue;
        
        int breadth_array_free_cells_size;
        float starting_cordinate_x;
        float starting_cordinate_y;
        float starting_cordinate_z;
        float current_yaw_angle;

        ros::Subscriber subscriber_node;

        ros::NodeHandle node_handle_;
        std::string topic_;

        /*
            methods
        */
        inline float round(float val);
        void clearVariables();
        int checkSquareCondition(float x_cordinate, float y_cordinate, float z_cordinate, float box_dimension);
        void convert_odom_angle_to_radians(float yaw_in_radians);
        double get_turning_angle_between_two_points(double start_x_cordinate,double start_y_cordinate, double end_x_cordinate, double end_y_cordinate);
        double calculate_weight_to_the_point(struct Graph_Node *temp_current_node);
        void set_publish_end_point(double x_cordinate, double y_cordinate, double z_cordinate);
        void getAdjecentSquareCentroids(float x_cordinate,float y_cordinate, float z_cordinate, float box_dimension,struct Graph_Node *node);
        void initialize_first_node(float x_cordinate,float y_cordinate, float z_cordinate);
    public:
        /*
            methods
        */
        void retrieveDataFromOctomap(const octomap_msgs::OctomapConstPtr& msg);
        int **check_neighbourhood(const geometry_msgs::PoseStamped& pose, float box_dimension);
        void start();
        NavigationPlanner(ros::NodeHandle &nh, std::string topic);
        void neighbourhood_callback(const geometry_msgs::PoseStamped& pose);
        ~NavigationPlanner();
};
#endif