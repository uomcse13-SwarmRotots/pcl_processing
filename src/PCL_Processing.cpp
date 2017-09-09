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

#include <boost/lambda/lambda.hpp>  // _1
#include <boost/lambda/bind.hpp>    // bind()
#include <boost/tuple/tuple_io.hpp>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include<octomap/OcTreeBase.h>

using namespace octomap;

namespace {
  typedef double coord_t;
  typedef boost::tuple<coord_t,coord_t,coord_t> point_t;

  coord_t distance_sq(const point_t& a, const point_t& b) { // or boost::geometry::distance
    coord_t x = a.get<0>() - b.get<0>();
    coord_t y = a.get<1>() - b.get<1>();
    coord_t z = a.get<2>() - b.get<2>();
    return x*x + y*y + z*z;
  }
}

using octomap_msgs::GetOctomap;
using namespace std;
using namespace octomap;
pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

using namespace boost::lambda; // _1, _2, bind()
vector<point_t> points;

struct MapIndex{
    double x, y, z;
    MapIndex()
    :x(0), y(0), z(0){
    }
    MapIndex(double x_, double y_, double z_)
    :x(x_), y(y_), z(z_){
    }
};

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

template<typename Val> struct Array3D{
    typedef std::map<MapIndex, Val> Data;
    Data data;
    Val defaultValue;
    const Val& getValue(double x, double y, double z) const{
        MapIndex index(x, y, z);
        typename Data::const_iterator found = data.find(index);
        if (found == data.end())
            return defaultValue;
        return found->second;
    }
    void setValue(double x, double y, double z, const Val &val){
        data.insert(std::make_pair(MapIndex(x, y, z), val));
    }
    bool hasValue(double x, double y, double z) const{
        typename Data::const_iterator found = data.find(MapIndex(x, y, z));
        return found != data.end();
    }
    Array3D(const Val& defaultValue_ = Val())
    :defaultValue(defaultValue_){
    }
};

Array3D<double> occupied_points;
Array3D<double> checked_points;

struct Graph_Node{
    double x_cordinate;
    double y_cordinate;
    double z_cordinate;
    Graph_Node *predecessor;
    int path_cost;
    double priority;
};

struct Graph_Node *start_node;
struct Graph_Node *end_node;
Array3D<struct Graph_Node*> found_nodes;
queue<struct Graph_Node*> node_queue;

void clearVariables(){
    found_nodes = NULL;
    // node_queue = NULL;
}

void checkSquareCondition(double x_cordinate, double y_cordinate, double z_cordinate, double box_dimension){
    double start_x = x_cordinate - box_dimension/2;
    double end_x = x_cordinate + box_dimension/2;
    double start_y = y_cordinate - box_dimension/2;
    double end_y = y_cordinate + box_dimension/2;
    double start_z = z_cordinate - box_dimension/2;
    double end_z = z_cordinate + box_dimension/2;

    double i,j,k;
    for(i=start_x;i<=end_x;i++){
        for(j=start_y;j<=end_y;j++){
            for(k=start_z;j<=end_z;k++){
    
            }
        }
    }
}

void initialize_first_node(double x_cordinate,double y_cordinate, double z_cordinate){
    struct Graph_Node *temp_node = new Graph_Node;
    temp_node->x_cordinate = x_cordinate;
    temp_node->y_cordinate = y_cordinate;
    temp_node->z_cordinate = z_cordinate;
    temp_node->path_cost = 0;
    temp_node->predecessor = NULL;
    temp_node->priority = 0.0;
    start_node = temp_node;
}

void getAdjecentSquareCentroids(double x_cordinate,double y_cordinate, double z_cordinate, double box_dimension,struct Graph_Node *node){
    struct Graph_Node *current_node= node;

    double front_x = x_cordinate + box_dimension;
    double front_y = y_cordinate;
   
    if(!found_nodes.hasValue(front_x,front_y,z_cordinate)){
        struct Graph_Node *temp_node = new Graph_Node;
        temp_node->x_cordinate = front_x;
        temp_node->y_cordinate = front_y;
        temp_node->z_cordinate = z_cordinate;
        temp_node->path_cost = current_node->path_cost + 1;
        temp_node->predecessor = current_node;
        temp_node->priority = 0.0;
        found_nodes.setValue(front_x,front_y,z_cordinate,temp_node);
        node_queue.push(temp_node);
    }else{
        if(!occupied_points.hasValue(front_x,front_y,z_cordinate)){
            return;
        }else{
            struct Graph_Node *available_node = found_nodes.getValue(front_x,front_y,z_cordinate);
            if(available_node->path_cost>current_node->path_cost+1){
                available_node->path_cost = current_node->path_cost + 1;
                available_node->predecessor = current_node;
                available_node->priority = 0.0;
            }   
        }
    }
    
    double front_left_x = x_cordinate + box_dimension;
    double front_left_y = y_cordinate + box_dimension;
    
    if(!found_nodes.hasValue(front_left_x,front_left_y,z_cordinate)){
        struct Graph_Node *temp_node = new Graph_Node;
        temp_node->x_cordinate = front_left_x;
        temp_node->y_cordinate = front_left_y;
        temp_node->z_cordinate = z_cordinate;
        temp_node->path_cost = current_node->path_cost + 1;
        temp_node->predecessor = current_node;
        temp_node->priority = 0.0;
        found_nodes.setValue(front_left_x,front_left_y,z_cordinate,temp_node);
        node_queue.push(temp_node);
    }else{
        if(!occupied_points.hasValue(front_left_x,front_left_y,z_cordinate)){
            return;
        }else{
            struct Graph_Node *available_node = found_nodes.getValue(front_left_x,front_left_y,z_cordinate);
            if(available_node->path_cost>current_node->path_cost+1){
                available_node->path_cost = current_node->path_cost + 1;
                available_node->predecessor = current_node;
                available_node->priority = 0.0;
            }
        }
    }

    double left_x = x_cordinate;
    double left_y = y_cordinate + box_dimension;
    
    if(!found_nodes.hasValue(left_x,left_y,z_cordinate)){
        struct Graph_Node *temp_node = new Graph_Node;
        temp_node->x_cordinate = left_x;
        temp_node->y_cordinate = left_y;
        temp_node->z_cordinate = z_cordinate;
        temp_node->path_cost = current_node->path_cost + 1;
        temp_node->predecessor = current_node;
        temp_node->priority = 0.0;
        found_nodes.setValue(left_x,left_y,z_cordinate,temp_node);
        node_queue.push(temp_node);
    }else{
        if(!occupied_points.hasValue(left_x,left_y,z_cordinate)){
            return;
        }else{
            struct Graph_Node *available_node = found_nodes.getValue(left_x,left_y,z_cordinate);
            if(available_node->path_cost>current_node->path_cost+1){
                available_node->path_cost = current_node->path_cost + 1;
                available_node->predecessor = current_node;
                available_node->priority = 0.0;
            }
        }
    }

    double back_left_x = x_cordinate - box_dimension;
    double back_left_y = y_cordinate + box_dimension;
    
    if(!found_nodes.hasValue(back_left_x,back_left_y,z_cordinate)){
        struct Graph_Node *temp_node = new Graph_Node;
        temp_node->x_cordinate = back_left_x;
        temp_node->y_cordinate = back_left_y;
        temp_node->z_cordinate = z_cordinate;
        temp_node->path_cost = current_node->path_cost + 1;
        temp_node->predecessor = current_node;
        temp_node->priority = 0.0;
        found_nodes.setValue(back_left_x,back_left_y,z_cordinate,temp_node);
        node_queue.push(temp_node);
    }else{
        if(!occupied_points.hasValue(back_left_x,back_left_y,z_cordinate)){
            return;
        }else{
           struct Graph_Node *available_node = found_nodes.getValue(back_left_x,back_left_y,z_cordinate);
            if(available_node->path_cost>current_node->path_cost+1){
                available_node->path_cost = current_node->path_cost + 1;
                available_node->predecessor = current_node;
                available_node->priority = 0.0;
            } 
        }
    }

    double back_x = x_cordinate - box_dimension;
    double back_y = y_cordinate;
    
    if(!found_nodes.hasValue(back_x,back_y,z_cordinate)){
        struct Graph_Node *temp_node = new Graph_Node;
        temp_node->x_cordinate = back_x;
        temp_node->y_cordinate = back_y;
        temp_node->z_cordinate = z_cordinate;
        temp_node->path_cost = current_node->path_cost + 1;
        temp_node->predecessor = current_node;
        temp_node->priority = 0.0;
        found_nodes.setValue(back_x,back_y,z_cordinate,temp_node);
        node_queue.push(temp_node);
    }else{
        if(!occupied_points.hasValue(back_x,back_y,z_cordinate)){
            return;
        }else{
            struct Graph_Node *available_node = found_nodes.getValue(back_x,back_y,z_cordinate);
            if(available_node->path_cost>current_node->path_cost+1){
                available_node->path_cost = current_node->path_cost + 1;
                available_node->predecessor = current_node;
                available_node->priority = 0.0;
            }
        }
    }

    double back_right_x = x_cordinate - box_dimension;
    double back_right_y = y_cordinate - box_dimension;
    
    if(!found_nodes.hasValue(back_right_x,back_right_y,z_cordinate)){
        struct Graph_Node *temp_node = new Graph_Node;
        temp_node->x_cordinate = back_right_x;
        temp_node->y_cordinate = back_right_y;
        temp_node->z_cordinate = z_cordinate;
        temp_node->path_cost = current_node->path_cost + 1;
        temp_node->predecessor = current_node;
        temp_node->priority = 0.0;
        found_nodes.setValue(back_right_x,back_right_y,z_cordinate,temp_node);
        node_queue.push(temp_node);
    }else{
        if(!occupied_points.hasValue(back_right_x,back_right_y,z_cordinate)){
            return;
        }else{
            struct Graph_Node *available_node = found_nodes.getValue(back_right_x,back_right_y,z_cordinate);
            if(available_node->path_cost>current_node->path_cost+1){
                available_node->path_cost = current_node->path_cost + 1;
                available_node->predecessor = current_node;
                available_node->priority = 0.0;
            }
        }
    }

    double right_x = x_cordinate;
    double right_y = y_cordinate - box_dimension;
    
    if(!found_nodes.hasValue(right_x,right_y,z_cordinate)){
        struct Graph_Node *temp_node = new Graph_Node;
        temp_node->x_cordinate = right_x;
        temp_node->y_cordinate = right_y;
        temp_node->z_cordinate = z_cordinate;
        temp_node->path_cost = current_node->path_cost + 1;
        temp_node->predecessor = current_node;
        temp_node->priority = 0.0;
        found_nodes.setValue(right_x,right_y,z_cordinate,temp_node);
        node_queue.push(temp_node);
    }else{
        if(!occupied_points.hasValue(right_x,right_y,z_cordinate)){
            return;
        }else{
            struct Graph_Node *available_node = found_nodes.getValue(right_x,right_y,z_cordinate);
            if(available_node->path_cost>current_node->path_cost+1){
                available_node->path_cost = current_node->path_cost + 1;
                available_node->predecessor = current_node;
                available_node->priority = 0.0;
            }  
        }
    }

    double front_right_x = x_cordinate + box_dimension;
    double front_right_y = y_cordinate - box_dimension;
    
    if(!found_nodes.hasValue(front_right_x,front_right_y,z_cordinate)){
        struct Graph_Node *temp_node = new Graph_Node;
        temp_node->x_cordinate = front_right_x;
        temp_node->y_cordinate = front_right_y;
        temp_node->z_cordinate = z_cordinate;
        temp_node->path_cost = current_node->path_cost + 1;
        temp_node->predecessor = current_node;
        temp_node->priority = 0.0;
        found_nodes.setValue(front_right_x,front_right_y,z_cordinate,temp_node);
        node_queue.push(temp_node);
    }else{
        if(!occupied_points.hasValue(front_right_x,front_right_y,z_cordinate)){
            return;
        }else{
            struct Graph_Node *available_node = found_nodes.getValue(front_right_x,front_right_y,z_cordinate);
            if(available_node->path_cost>current_node->path_cost+1){
                available_node->path_cost = current_node->path_cost + 1;
                available_node->predecessor = current_node;
                available_node->priority = 0.0;
            }
        }
    }
    struct Graph_Node *next_node;// = node_queue.pop();
    getAdjecentSquareCentroids(next_node->x_cordinate,next_node->y_cordinate,next_node->z_cordinate,box_dimension,next_node);
}

void retrieveDataFromOctomap(const octomap_msgs::OctomapConstPtr& msg){
    AbstractOcTree* tree = msgToMap(*msg);
    OcTree* octree = dynamic_cast<OcTree*>(tree); 

    double count = 0;
    for(OcTree::leaf_iterator it = octree->begin_leafs(),end=octree->end_leafs(); it!= end; ++it){
        double point_x = it.getX();
        double point_y = it.getY();
        double point_z = it.getZ();
        if(it->getValue()>0){
            if(!occupied_points.hasValue(point_x,point_y,point_z)){
                occupied_points.setValue(point_x,point_y,point_z,1);
            }
            count = count+1;
        }else{
            if(!occupied_points.hasValue(point_x,point_y,point_z)){
                occupied_points.setValue(point_x,point_y,point_z,0);
            }
        }
    }
    printf("%f\n",count);
    // std::cout<<"VOLUME::::"<<v<<endl;


}


int main (int argc, char** argv) {
    ros::init (argc, argv, "cloud_sub");
    ros::NodeHandle node_handler;
    ros::Rate loop_rate(10);
    ros::Subscriber subscriber_node;
    // subscriber_node = node_handler.subscribe<PointCloud>("/octomap_point_cloud_centers", 100, point_cloud_subscriber);
    subscriber_node = node_handler.subscribe("/octomap_full", 100, retrieveDataFromOctomap);
    // subscriber_node = node_handler.subscribe("/odom", 1, update_odometry_data);

    ros::spin();   
 }

