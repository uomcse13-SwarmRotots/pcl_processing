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
  typedef float coord_t;
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
    float x, y, z;
    MapIndex()
    :x(0), y(0), z(0){
    }
    MapIndex(float x_, float y_, float z_)
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

Array3D<float> occupied_points;
Array3D<float> checked_points;

struct Graph_Node{
    float x_cordinate;
    float y_cordinate;
    float z_cordinate;
    Graph_Node *predecessor;
    int path_cost;
    float priority;
};

struct Graph_Node *start_node;
struct Graph_Node *end_node;
Array3D<struct Graph_Node*> found_nodes;
queue<struct Graph_Node*> node_queue;

struct Graph_Node *breadth_array[1000];
int breadth_array_size;

float starting_cordinate_x;
float starting_cordinate_y;
float starting_cordinate_z;

inline float round(float val){
    val=val*1000;
    if(val<0){
        return ceil(val-0.5)/1000;
    }
    return floor(val+0.5)/1000;
}

void clearVariables(){
    found_nodes = NULL;
    // node_queue = NULL;
}

/*
 * Function to check square condition (traversable/untraversable)
 * retunr value status
 * -1 -undiscovered
 *  0 -untraversable
 *  1 -traversable
 */


int checkSquareCondition(float x_cordinate, float y_cordinate, float z_cordinate, float box_dimension){
    float start_x = x_cordinate - box_dimension/2;
    float end_x = x_cordinate + box_dimension/2;
    float start_y = y_cordinate - box_dimension/2;
    float end_y = y_cordinate + box_dimension/2;
    float start_z = z_cordinate - box_dimension/2;
    float end_z = z_cordinate + box_dimension/2;

    float max_height_cahnge = box_dimension/2;
    float unit_length = 0.025;
    float max_height;
    float min_height;
    float height;
    float average_height;
    float previous_average_height;
    bool plane_exsist;
    bool plane_ok;
    bool column_free;
    float i,j,k;

    previous_average_height =0;

    for(i=start_x;i<=end_x;i=i+unit_length){
        max_height = 0;
        min_height = box_dimension; 
        average_height = 0;
        for(j=start_y;j<=end_y;j=j+unit_length){
            //column_free = false;
            height =0;
            for(k=start_z;j<=end_z;k=k+unit_length){
                if(occupied_points.hasValue(i,j,k)){
                    if(occupied_points.getValue(i,j,k)>0){
                        height+=unit_length;
                    } else {
                        //column_free = true;
                        break;
                    }
                }
                else{
                    return -1;
                }
            }
            if(min_height>height)   min_height=height;
            if(max_height<height)   max_height=height;
            average_height+=height;        
        }
        average_height = average_height/((end_y-start_y)/unit_length); 
        if(i!=start_x){
            if((average_height-previous_average_height)>max_height_cahnge){
                return 0;
            }
        }
        previous_average_height = average_height;
    } 
    return 1;
        
}

void set_publish_end_point(double x_cordinate, double y_cordinate, double z_cordinate){
    printf("ending cordinates are = %f %f %f \n",x_cordinate,y_cordinate,z_cordinate);
}

void getAdjecentSquareCentroids(float x_cordinate,float y_cordinate, float z_cordinate, float box_dimension,struct Graph_Node *node){
    struct Graph_Node *current_node= node;

 
    printf("start cordinate %f %f %f \n",x_cordinate,y_cordinate,z_cordinate);
    if(!found_nodes.hasValue(x_cordinate,y_cordinate,z_cordinate)){
        found_nodes.setValue(x_cordinate,y_cordinate,z_cordinate,current_node);
    }

    float front_x = round(x_cordinate + box_dimension);
    float front_y = round(y_cordinate);
    
    if(!found_nodes.hasValue(front_x,front_y,z_cordinate)){
        if(!occupied_points.hasValue(front_x,front_y,z_cordinate)){
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = front_x;
            temp_node->y_cordinate = front_y;
            temp_node->z_cordinate = z_cordinate;
            temp_node->path_cost = current_node->path_cost + 1;
            temp_node->predecessor = current_node;
            temp_node->priority = 0.0;
            breadth_array[breadth_array_size] = temp_node;
            breadth_array_size = breadth_array_size + 1;
        }else{
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = front_x;
            temp_node->y_cordinate = front_y;
            temp_node->z_cordinate = z_cordinate;
            temp_node->path_cost = current_node->path_cost + 1;
            temp_node->predecessor = current_node;
            temp_node->priority = 0.0;
            found_nodes.setValue(front_x,front_y,z_cordinate,temp_node);
            int traversable = checkSquareCondition(front_x,front_y,z_cordinate,0.5);
            // if(traversable){
                node_queue.push(temp_node);
                //printf("( %f %f %f )\n",front_x,front_y,z_cordinate);
            // }    
        }
    }else{
        if(!occupied_points.hasValue(front_x,front_y,z_cordinate)){
            //printf("( Return on 2: %f %f %f )\n",front_x,front_y,z_cordinate);
            //return;
        }else{
            struct Graph_Node *available_node = found_nodes.getValue(front_x,front_y,z_cordinate);
            if(available_node->path_cost>current_node->path_cost+1){
                available_node->path_cost = current_node->path_cost + 1;
                available_node->predecessor = current_node;
                available_node->priority = 0.0;
            }
            ////printf("( updated %f %f %f )\n",front_x,front_y,z_cordinate);  
        }
    }
    
    float front_left_x = round(x_cordinate + box_dimension);
    float front_left_y = round(y_cordinate + box_dimension);
    
    if(!found_nodes.hasValue(front_left_x,front_left_y,z_cordinate)){
        if(!occupied_points.hasValue(front_left_x,front_left_y,z_cordinate)){
            
            // occupied_points.setValue(front_left_x,front_left_y,z_cordinate,0);
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = front_left_x;
            temp_node->y_cordinate = front_left_y;
            temp_node->z_cordinate = z_cordinate;
            temp_node->path_cost = current_node->path_cost + 1;
            temp_node->predecessor = current_node;
            temp_node->priority = 0.0;
            breadth_array[breadth_array_size] = temp_node;
            breadth_array_size = breadth_array_size + 1;
            printf("( Return on 1: %f %f %f )\n",front_left_x,front_left_y,z_cordinate);
        }else{
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = front_left_x;
            temp_node->y_cordinate = front_left_y;
            temp_node->z_cordinate = z_cordinate;
            temp_node->path_cost = current_node->path_cost + 1;
            temp_node->predecessor = current_node;
            temp_node->priority = 0.0;
            found_nodes.setValue(front_left_x,front_left_y,z_cordinate,temp_node);
            int traversable = checkSquareCondition(front_left_x,front_left_y,z_cordinate,0.5);
            // if(traversable){
                node_queue.push(temp_node);
                //printf("( %f %f %f )\n",front_left_x,front_left_y,z_cordinate);
            // }  
        }
    }else{
        if(!occupied_points.hasValue(front_left_x,front_left_y,z_cordinate)){
            //printf("( Return on 2: %f %f %f )\n",front_left_x,front_left_y,z_cordinate);
            //return;
        }else{
            struct Graph_Node *available_node = found_nodes.getValue(front_left_x,front_left_y,z_cordinate);
            if(available_node->path_cost>current_node->path_cost+1){
                available_node->path_cost = current_node->path_cost + 1;
                available_node->predecessor = current_node;
                available_node->priority = 0.0;
            }
            ////printf("( updated %f %f %f )\n",front_left_x,front_left_y,z_cordinate);
        }
    }

    float left_x = round(x_cordinate);
    float left_y = round(y_cordinate + box_dimension);
    
    if(!found_nodes.hasValue(left_x,left_y,z_cordinate)){
        if(!occupied_points.hasValue(left_x,left_y,z_cordinate)){
            
            //return;
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = left_x;
            temp_node->y_cordinate = left_y;
            temp_node->z_cordinate = z_cordinate;
            temp_node->path_cost = current_node->path_cost + 1;
            temp_node->predecessor = current_node;
            temp_node->priority = 0.0;
            // found_nodes.setValue(left_x,left_y,z_cordinate,temp_node);
            breadth_array[breadth_array_size] = temp_node;
            breadth_array_size = breadth_array_size + 1;
            printf("( Return on 1: %f %f %f )\n",left_x,left_y,z_cordinate);
        }else{
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = left_x;
            temp_node->y_cordinate = left_y;
            temp_node->z_cordinate = z_cordinate;
            temp_node->path_cost = current_node->path_cost + 1;
            temp_node->predecessor = current_node;
            temp_node->priority = 0.0;
            found_nodes.setValue(left_x,left_y,z_cordinate,temp_node);
            // int traversable = checkSquareCondition(left_x,left_y,z_cordinate,0.5);
            // if(traversable){
                node_queue.push(temp_node);
                //printf("( %f %f %f )\n",left_x,left_y,z_cordinate);
            // } 
        }
    }else{
        if(!occupied_points.hasValue(left_x,left_y,z_cordinate)){
            //printf("( Return on 2: %f %f %f )\n",left_x,left_y,z_cordinate);
            //return;
        }else{
            struct Graph_Node *available_node = found_nodes.getValue(left_x,left_y,z_cordinate);
            if(available_node->path_cost>current_node->path_cost+1){
                available_node->path_cost = current_node->path_cost + 1;
                available_node->predecessor = current_node;
                available_node->priority = 0.0;
            }
            ////printf("( updated %f %f %f )\n",left_x,left_y,z_cordinate);
        }
    }

    float back_left_x = round(x_cordinate - box_dimension);
    float back_left_y = round(y_cordinate + box_dimension);
    
    if(!found_nodes.hasValue(back_left_x,back_left_y,z_cordinate)){
        if(!occupied_points.hasValue(back_left_x,back_left_y,z_cordinate)){
            
            //return;
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = back_left_x;
            temp_node->y_cordinate = back_left_y;
            temp_node->z_cordinate = z_cordinate;
            temp_node->path_cost = current_node->path_cost + 1;
            temp_node->predecessor = current_node;
            temp_node->priority = 0.0;
            breadth_array[breadth_array_size] = temp_node;
            breadth_array_size = breadth_array_size + 1;
            printf("( Return on 1: %f %f %f )\n",back_left_x,back_left_y,z_cordinate);
        }else{
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = back_left_x;
            temp_node->y_cordinate = back_left_y;
            temp_node->z_cordinate = z_cordinate;
            temp_node->path_cost = current_node->path_cost + 1;
            temp_node->predecessor = current_node;
            temp_node->priority = 0.0;
            found_nodes.setValue(back_left_x,back_left_y,z_cordinate,temp_node);
            // int traversable = checkSquareCondition(back_left_x,back_left_y,z_cordinate,0.5);
            // if(traversable){
                node_queue.push(temp_node);
                //printf("( %f %f %f )\n",back_left_x,back_left_y,z_cordinate);
            // } 
        }
    }else{
        if(!occupied_points.hasValue(back_left_x,back_left_y,z_cordinate)){
            //printf("( Return on 2: %f %f %f )\n",back_left_x,back_left_y,z_cordinate);
            //return;
        }else{
        struct Graph_Node *available_node = found_nodes.getValue(back_left_x,back_left_y,z_cordinate);
            if(available_node->path_cost>current_node->path_cost+1){
                available_node->path_cost = current_node->path_cost + 1;
                available_node->predecessor = current_node;
                available_node->priority = 0.0;
            }
            ////printf("( updated %f %f %f )\n",back_left_x,back_left_y,z_cordinate); 
        }
    }

    float back_x = round(x_cordinate - box_dimension);
    float back_y = round(y_cordinate);
    
    if(!found_nodes.hasValue(back_x,back_y,z_cordinate)){
        if(!occupied_points.hasValue(back_x,back_y,z_cordinate)){
            
            //return;
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = back_x;
            temp_node->y_cordinate = back_y;
            temp_node->z_cordinate = z_cordinate;
            temp_node->path_cost = current_node->path_cost + 1;
            temp_node->predecessor = current_node;
            temp_node->priority = 0.0;
            breadth_array[breadth_array_size] = temp_node;
            breadth_array_size = breadth_array_size + 1;
            printf("( Return on 1: %f %f %f )\n",back_x,back_y,z_cordinate);
        }else{
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = back_x;
            temp_node->y_cordinate = back_y;
            temp_node->z_cordinate = z_cordinate;
            temp_node->path_cost = current_node->path_cost + 1;
            temp_node->predecessor = current_node;
            temp_node->priority = 0.0;
            found_nodes.setValue(back_x,back_y,z_cordinate,temp_node);
            // int traversable = checkSquareCondition(back_x,back_y,z_cordinate,0.5);
            // if(traversable){
                node_queue.push(temp_node);
                //printf("( %f %f %f )\n",back_x,back_y,z_cordinate);
            // }
        }
    }else{
        if(!occupied_points.hasValue(back_x,back_y,z_cordinate)){
            //printf("( Return on 2: %f %f %f )\n",back_x,back_y,z_cordinate);
            //return;
            
        }else{
            struct Graph_Node *available_node = found_nodes.getValue(back_x,back_y,z_cordinate);
            if(available_node->path_cost>current_node->path_cost+1){
                available_node->path_cost = current_node->path_cost + 1;
                available_node->predecessor = current_node;
                available_node->priority = 0.0;
            }
            // //printf("( updated  %f %f %f )\n",back_x,back_y,z_cordinate);
        }
    }

    float back_right_x = round(x_cordinate - box_dimension);
    float back_right_y = round(y_cordinate - box_dimension);
    
    if(!found_nodes.hasValue(back_right_x,back_right_y,z_cordinate)){
        if(!occupied_points.hasValue(back_right_x,back_right_y,z_cordinate)){
            
            //return;
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = back_right_x;
            temp_node->y_cordinate = back_right_y;
            temp_node->z_cordinate = z_cordinate;
            temp_node->path_cost = current_node->path_cost + 1;
            temp_node->predecessor = current_node;
            temp_node->priority = 0.0;
            breadth_array[breadth_array_size] = temp_node;
            breadth_array_size = breadth_array_size + 1;
            printf("( Return on 1: %f %f %f )\n",back_right_x,back_right_y,z_cordinate);
        }else{
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = back_right_x;
            temp_node->y_cordinate = back_right_y;
            temp_node->z_cordinate = z_cordinate;
            temp_node->path_cost = current_node->path_cost + 1;
            temp_node->predecessor = current_node;
            temp_node->priority = 0.0;
            found_nodes.setValue(back_right_x,back_right_y,z_cordinate,temp_node);
            // int traversable = checkSquareCondition(back_right_x,back_right_y,z_cordinate,0.5);
            // if(traversable){
                node_queue.push(temp_node);
                //printf("( %f %f %f )\n",back_right_x,back_right_y,z_cordinate);
            // }
        }
    }else{
        if(!occupied_points.hasValue(back_right_x,back_right_y,z_cordinate)){
            //printf("( Return on 2: %f %f %f )\n",back_right_x,back_right_y,z_cordinate);
            //return;
        }else{
            struct Graph_Node *available_node = found_nodes.getValue(back_right_x,back_right_y,z_cordinate);
            if(available_node->path_cost>current_node->path_cost+1){
                available_node->path_cost = current_node->path_cost + 1;
                available_node->predecessor = current_node;
                available_node->priority = 0.0;
            }
            ////printf("( updated %f %f %f )\n",back_right_x,back_right_y,z_cordinate);
        }
    }

    float right_x = round(x_cordinate);
    float right_y = round(y_cordinate - box_dimension);
    
    if(!found_nodes.hasValue(right_x,right_y,z_cordinate)){
        if(!occupied_points.hasValue(right_x,right_y,z_cordinate)){
            
            // std::cout << right_x <<endl;
            // while(current_node!= NULL){
            // printf("Return on = %f %f %f )\n",current_node->x_cordinate,current_node->y_cordinate,current_node->z_cordinate);
            //     current_node = current_node->predecessor;
            // }
            // return;
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = right_x;
            temp_node->y_cordinate = right_y;
            temp_node->z_cordinate = z_cordinate;
            temp_node->path_cost = current_node->path_cost + 1;
            temp_node->predecessor = current_node;
            temp_node->priority = 0.0;
            breadth_array[breadth_array_size] = temp_node;
            breadth_array_size = breadth_array_size + 1;
            printf("( Return on 1 : %f %f %f )\n",right_x,right_y,z_cordinate);
        }else{
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = right_x;
            temp_node->y_cordinate = right_y;
            temp_node->z_cordinate = z_cordinate;
            temp_node->path_cost = current_node->path_cost + 1;
            temp_node->predecessor = current_node;
            temp_node->priority = 0.0;
            found_nodes.setValue(right_x,right_y,z_cordinate,temp_node);
            // int traversable = checkSquareCondition(right_x,right_y,z_cordinate,0.5);
            // if(traversable){
                node_queue.push(temp_node);
                //printf("( %f %f %f )\n",right_x,right_y,z_cordinate);
            // }
        }
    }else{
        if(!occupied_points.hasValue(right_x,right_y,z_cordinate)){
            //printf("( Return on 2: %f %f %f )\n",right_x,right_y,z_cordinate);
            //return;
        }else{
            struct Graph_Node *available_node = found_nodes.getValue(right_x,right_y,z_cordinate);
            if(available_node->path_cost>current_node->path_cost+1){
                available_node->path_cost = current_node->path_cost + 1;
                available_node->predecessor = current_node;
                available_node->priority = 0.0;
            }
            ////printf("( updated %f %f %f )\n",right_x,right_y,z_cordinate);  
        }
    }

    float front_right_x = round(x_cordinate + box_dimension);
    float front_right_y = round(y_cordinate - box_dimension);
    
    if(!found_nodes.hasValue(front_right_x,front_right_y,z_cordinate)){
        if(!occupied_points.hasValue(front_right_x,front_right_y,z_cordinate)){
            float value = occupied_points.getValue(front_right_x,front_right_y,z_cordinate);
            // printf("failed");
           
            //printf("%d\n",occupied_points.hasValue(front_right_x,front_right_y,z_cordinate));
            //return;
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = front_right_x;
            temp_node->y_cordinate = front_right_y;
            temp_node->z_cordinate = z_cordinate;
            temp_node->path_cost = current_node->path_cost + 1;
            temp_node->predecessor = current_node;
            temp_node->priority = 0.0;
            breadth_array[breadth_array_size] = temp_node;
            breadth_array_size = breadth_array_size + 1;
            printf("( Return on 1: %f %f %f )\n",front_right_x,front_right_y,z_cordinate);
        }else{
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = front_right_x;
            temp_node->y_cordinate = front_right_y;
            temp_node->z_cordinate = z_cordinate;
            temp_node->path_cost = current_node->path_cost + 1;
            temp_node->predecessor = current_node;
            temp_node->priority = 0.0;
            found_nodes.setValue(front_right_x,front_right_y,z_cordinate,temp_node);
            // int traversable = checkSquareCondition(front_right_x,front_right_y,z_cordinate,0.5);
            // if(traversable){
                node_queue.push(temp_node);
                //printf("( %f %f %f )\n",front_right_x,front_right_y,z_cordinate);
            // }
        }
    }else{
        if(!occupied_points.hasValue(front_right_x,front_right_y,z_cordinate)){
            //printf("( Return on 2: %f %f %f )\n",front_right_x,front_right_y,z_cordinate);
            //return;
        }else{
            struct Graph_Node *available_node = found_nodes.getValue(front_right_x,front_right_y,z_cordinate);
            if(available_node->path_cost>current_node->path_cost+1){
                available_node->path_cost = current_node->path_cost + 1;
                available_node->predecessor = current_node;
                available_node->priority = 0.0;
            }
            ////printf("( updated %f %f %f )\n",front_right_x,front_right_y,z_cordinate);
        }
    }

    if(breadth_array_size == 0){
        if(node_queue.empty()){
            printf("kg");
            while(current_node!= NULL){
                printf("Node Traversed = %f %f %f )\n",current_node->x_cordinate,current_node->y_cordinate,current_node->z_cordinate);
                current_node = current_node->predecessor;
            }
            return;
        }else{
            struct Graph_Node *next_node = node_queue.front();
            node_queue.pop();
            printf("End One Round\n");
            getAdjecentSquareCentroids(next_node->x_cordinate,next_node->y_cordinate,next_node->z_cordinate,box_dimension,next_node);
        }    
    }else{
        int i;
        int minimum_position = 0;
        double minimum_path_cost = 100000000;

        for (i=0; i < breadth_array_size;i++ ) {
            printf("for loop\n");
            struct Graph_Node *temp_node = breadth_array[i];
            printf("breadth array\n");
            int temp_path_cost = temp_node->predecessor->path_cost + 1;
            printf("path cost\n");
            if(temp_path_cost<minimum_path_cost){
                minimum_position = i;
                minimum_path_cost = temp_path_cost;
            }
            //delete breadth_array[i];
        }

        struct Graph_Node *minimum_cost_node = breadth_array[minimum_position];
        
        for (i=0; i < breadth_array_size;i++ ) {
            delete breadth_array[i];
        }
        breadth_array_size = 0;
        printf("deleted\n");
        while(minimum_cost_node != NULL){
            printf("Node Traversed = %f %f %f )\n",minimum_cost_node->x_cordinate,minimum_cost_node->y_cordinate,minimum_cost_node->z_cordinate);
            minimum_cost_node=minimum_cost_node->predecessor;
        } 
        return;
    }    
}

void initialize_first_node(float x_cordinate,float y_cordinate, float z_cordinate){
    struct Graph_Node *temp_node = new Graph_Node;
    temp_node->x_cordinate = x_cordinate;
    temp_node->y_cordinate = y_cordinate;
    temp_node->z_cordinate = z_cordinate;
    temp_node->path_cost = 0;
    temp_node->predecessor = NULL;
    temp_node->priority = 0.0;
    start_node = temp_node;
    getAdjecentSquareCentroids(x_cordinate,y_cordinate,z_cordinate,0.500000,start_node);
}

void retrieveDataFromOctomap(const octomap_msgs::OctomapConstPtr& msg){
    AbstractOcTree* tree = msgToMap(*msg);
    OcTree* octree = dynamic_cast<OcTree*>(tree); 

    float count = 0;
    for(OcTree::leaf_iterator it = octree->begin_leafs(),end=octree->end_leafs(); it!= end; ++it){
        float point_x = it.getX();
        float point_y = it.getY();
        float point_z = it.getZ();
        if(it->getValue()>0){
            if(!occupied_points.hasValue(point_x,point_y,point_z)){
                occupied_points.setValue(point_x,point_y,point_z,1);
                printf("( %f %f %f )\n",point_x,point_y,point_z);
            }
            count = count+1;
        }else{
            if(!occupied_points.hasValue(point_x,point_y,point_z)){
                occupied_points.setValue(point_x,point_y,point_z,0);
            }
        }
        
    }
    //printf("%f\n",count);
    initialize_first_node(3.825000,1.625000,-0.025000);
    // std::cout<<"VOLUME::::"<<v<<endl;
}


int main (int argc, char** argv) {
    ros::init (argc, argv, "cloud_sub");
    ros::NodeHandle node_handler;
    ros::Rate loop_rate(1000);
    ros::Subscriber subscriber_node;
    // subscriber_node = node_handler.subscribe<PointCloud>("/octomap_point_cloud_centers", 100, point_cloud_subscriber);
    subscriber_node = node_handler.subscribe("/octomap_full", 10000, retrieveDataFromOctomap);
    // subscriber_node = node_handler.subscribe("/odom", 1, update_odometry_data);

    ros::spin();   
 }

