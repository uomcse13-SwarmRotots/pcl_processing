// #include <iostream>
// #include <string>
// #include <sstream>
// #include <math.h>

// #include <octomap/OcTree.h>

// #include <bitset>
// #include <map>
// #include <iostream>
// #include <ros/ros.h>
// #include <pcl_ros/point_cloud.h>
// #include <pcl/point_types.h>
// #include <boost/foreach.hpp>
// #include <tf/transform_datatypes.h>
// #include <nav_msgs/Odometry.h>
// #include <tf/tf.h>
// #include <geometry_msgs/Twist.h>
// #include <geometry_msgs/Pose.h>
// #include <tf/transform_datatypes.h>
// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>
// #include <queue>

// #include <octomap_msgs/conversions.h>
// #include <octomap/octomap.h>
// #include <fstream>

// #include <pcl/point_cloud.h>
// #include <pcl/octree/octree_search.h>

// #include <iostream>
// #include <vector>
// #include <ctime>
// #include <pcl/kdtree/kdtree_flann.h>
// #include <octomap_msgs/GetOctomap.h>
// #include <algorithm>

// #include <boost/lambda/lambda.hpp>  // _1
// #include <boost/lambda/bind.hpp>    // bind()
// #include <boost/tuple/tuple_io.hpp>
// #include <octomap/octomap.h>
// #include <octomap/OcTree.h>
// #include<octomap/OcTreeBase.h>
// #include <pcl/common/common.h>
// #include <Eigen/SVD>

// const float  PI_F=3.14159265358979f;

// namespace {
//   typedef float coord_t;
//   typedef boost::tuple<coord_t,coord_t,coord_t> point_t;

//   coord_t distance_sq(const point_t& a, const point_t& b) { // or boost::geometry::distance
//     coord_t x = a.get<0>() - b.get<0>();
//     coord_t y = a.get<1>() - b.get<1>();
//     coord_t z = a.get<2>() - b.get<2>();
//     return x*x + y*y + z*z;
//   }
// }

// using octomap_msgs::GetOctomap;
// using namespace std;
// using namespace octomap;
// pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

// using namespace boost::lambda; // _1, _2, bind()
// vector<point_t> points;

// struct MapIndex{
//     float x, y, z;
//     MapIndex()
//     :x(0), y(0), z(0){
//     }
//     MapIndex(float x_, float y_, float z_)
//     :x(x_), y(y_), z(z_){
//     }
// };

// bool operator<(const MapIndex &v1, const MapIndex &v2){
//     if (v1.z > v2.z)
//         return false;
//     if (v1.z < v2.z)
//         return true;
//     if (v1.y > v2.y)
//         return false;
//     if (v1.y < v2.y)
//         return true;
//     if (v1.x < v2.x)
//         return true;
//     return false;
// }

// template<typename Val> struct Array3D{
//     typedef std::map<MapIndex, Val> Data;
//     Data data;
//     Val defaultValue;
//     const Val& getValue(float x, float y, float z) const{
//         MapIndex index(x, y, z);
//         typename Data::const_iterator found = data.find(index);
//         if (found == data.end())
//             return defaultValue;
//         return found->second;
//     }
//     void setValue(float x, float y, float z, const Val &val){
//         data.insert(std::make_pair(MapIndex(x, y, z), val));
//     }
//     bool hasValue(float x, float y, float z) const{
//         typename Data::const_iterator found = data.find(MapIndex(x, y, z));
//         return found != data.end();
//     }
//     Array3D(const Val& defaultValue_ = Val())
//     :defaultValue(defaultValue_){
//     }
// };

// Array3D<float> occupied_points;
// Array3D<float> checked_points;

// struct Graph_Node{
//     float x_cordinate;
//     float y_cordinate;
//     float z_cordinate;
//     Graph_Node *predecessor;
//     int path_cost;
//     float priority;
// };

// struct Graph_Node *start_node;
// struct Graph_Node *end_node;
// Array3D<struct Graph_Node*> found_nodes;
// queue<struct Graph_Node*> node_queue;

// struct Graph_Node *breadth_array_free_cells[1000];
// int breadth_array_free_cells_size;

// float starting_cordinate_x;
// float starting_cordinate_y;
// float starting_cordinate_z;
// float current_yaw_angle;

// inline float round(float val){
//     val=val*1000;
//     if(val<0){
//         return ceil(val-0.5)/1000;
//     }
//     return floor(val+0.5)/1000;
// }

// void clearVariables(){
//     found_nodes = NULL;
//     // node_queue = NULL;
// }

// /*
//  * Function to check square condition (traversable/untraversable)
//  * retunr value status
//  * -1 -undiscovered
//  *  0 -untraversable
//  *  1 -traversable
//  */


// int checkSquareCondition(float x_cordinate, float y_cordinate, float z_cordinate, float box_dimension){
//     float start_x = x_cordinate - box_dimension/2;
//     float end_x = x_cordinate + box_dimension/2;
//     float start_y = y_cordinate - box_dimension/2;
//     float end_y = y_cordinate + box_dimension/2;
//     float start_z = z_cordinate - box_dimension/2;
//     float end_z = z_cordinate + box_dimension/2;

//     float max_height_cahnge = box_dimension/2;
//     float unit_length = 0.025;
//     float max_height;
//     float min_height;
//     float height;
//     float average_height;
//     float previous_average_height;
//     bool plane_exsist;
//     bool plane_ok;
//     bool column_free;
//     float i,j,k;

//     previous_average_height =0;

//     for(i=start_x;i<=end_x;i=i+unit_length){
//         max_height = 0;
//         min_height = box_dimension; 
//         average_height = 0;
//         for(j=start_y;j<=end_y;j=j+unit_length){
//             //column_free = false;
//             height =0;
//             for(k=start_z;j<=end_z;k=k+unit_length){
//                 if(occupied_points.hasValue(i,j,k)){
//                     if(occupied_points.getValue(i,j,k)>0){
//                         height+=unit_length;
//                     } else {
//                         //column_free = true;
//                         break;
//                     }
//                 }
//                 else{
//                     return -1;
//                 }
//             }
//             if(min_height>height)   min_height=height;
//             if(max_height<height)   max_height=height;
//             average_height+=height;        
//         }
//         average_height = average_height/((end_y-start_y)/unit_length); 
//         if(i!=start_x){
//             if((average_height-previous_average_height)>max_height_cahnge){
//                 return 0;
//             }
//         }
//         previous_average_height = average_height;
//     } 
//     return 1;
        
// }

// void convert_odom_angle_to_radians(float yaw_in_radians){
//     if(yaw_in_radians<0){
//         yaw_in_radians = 360 - abs(yaw_in_radians)*180/PI_F;
//     }
//     current_yaw_angle = yaw_in_radians;
// }

// double get_turning_angle_between_two_points(double start_x_cordinate,double start_y_cordinate, double end_x_cordinate, double end_y_cordinate){
//     float angle_between_points = atan2(end_y_cordinate - start_y_cordinate, end_x_cordinate - start_x_cordinate)*180/PI_F;
//     return angle_between_points;
// }

// double calculate_weight_to_the_point(struct Graph_Node *temp_current_node){
//     float rotation = 0;
//     float distance = 0;
//     while(temp_current_node != NULL){
//         if(temp_current_node->predecessor!=NULL & temp_current_node->predecessor->predecessor==NULL ){
//             rotation = rotation + abs(current_yaw_angle - get_turning_angle_between_two_points(temp_current_node->x_cordinate,temp_current_node->y_cordinate,temp_current_node->predecessor->x_cordinate,temp_current_node->predecessor->y_cordinate));
//             distance = distance + 1;
//             return rotation*100+distance*1000;//,distance
//         }else{
//             rotation = rotation + abs(get_turning_angle_between_two_points(temp_current_node->x_cordinate,temp_current_node->y_cordinate,temp_current_node->predecessor->x_cordinate,temp_current_node->predecessor->y_cordinate));
//             distance = distance + 1;
//         }    
//     }
// }

// void set_publish_end_point(double x_cordinate, double y_cordinate, double z_cordinate){
//     printf("ending cordinates are = %f %f %f \n",x_cordinate,y_cordinate,z_cordinate);
// //     ros::init(argc, argv, "target_point_publisher");
// //     ros::NodeHandle n;
// //     ros::Publisher chatter_pub = n.advertise<std_msgs::String>("target_publisher", 1000);

// //     ros::Rate loop_rate(10);

// //     int count = 0;
// //     while (ros::ok())  {
// //         std_msgs::String msg;
// //         std::stringstream ss;
// //         ss << std::to_string(x_cordinate) + std::to_string(y_cordinate) + std::to_string(z_cordinate)<< count;
// //         msg.data = ss.str();
// //         ROS_INFO("%s", msg.data.c_str());
// //         chatter_pub.publish(msg);
// //         ros::spinOnce();
// //         loop_rate.sleep();
// //         ++count;
// //   }
// //   return 0;
// }




// void getAdjecentSquareCentroids(float x_cordinate,float y_cordinate, float z_cordinate, float box_dimension,struct Graph_Node *node){
//     struct Graph_Node *current_node= node;

//     printf("start cordinate %f %f %f \n",x_cordinate,y_cordinate,z_cordinate);
//     if(!found_nodes.hasValue(x_cordinate,y_cordinate,z_cordinate)){
//         found_nodes.setValue(x_cordinate,y_cordinate,z_cordinate,current_node);
//     }

//     float front_x = round(x_cordinate + box_dimension);
//     float front_y = round(y_cordinate);
    
//     if(!found_nodes.hasValue(front_x,front_y,z_cordinate)){
//         if(!occupied_points.hasValue(front_x,front_y,z_cordinate)){
//             struct Graph_Node *temp_node = new Graph_Node;
//             temp_node->x_cordinate = front_x;
//             temp_node->y_cordinate = front_y;
//             temp_node->z_cordinate = z_cordinate;
//             temp_node->path_cost = current_node->path_cost + 1;
//             temp_node->predecessor = current_node;
//             temp_node->priority = 0.0;
//             breadth_array_free_cells[breadth_array_free_cells_size] = temp_node;
//             breadth_array_free_cells_size = breadth_array_free_cells_size + 1;
//         }else{
//             struct Graph_Node *temp_node = new Graph_Node;
//             temp_node->x_cordinate = front_x;
//             temp_node->y_cordinate = front_y;
//             temp_node->z_cordinate = z_cordinate;
//             temp_node->path_cost = current_node->path_cost + 1;
//             temp_node->predecessor = current_node;
//             temp_node->priority = 0.0;
//             found_nodes.setValue(front_x,front_y,z_cordinate,temp_node);
//             int traversable = checkSquareCondition(front_x,front_y,z_cordinate,0.5);
//             // if(traversable){
//                 node_queue.push(temp_node);
//                 //printf("( %f %f %f )\n",front_x,front_y,z_cordinate);
//             // }    
//         }
//     }else{
//         if(!occupied_points.hasValue(front_x,front_y,z_cordinate)){

//         }else{
//             struct Graph_Node *available_node = found_nodes.getValue(front_x,front_y,z_cordinate);
//             if(available_node->path_cost>current_node->path_cost+1){
//                 available_node->path_cost = current_node->path_cost + 1;
//                 available_node->predecessor = current_node;
//                 available_node->priority = 0.0;
//             }
//         }
//     }
    
//     float front_left_x = round(x_cordinate + box_dimension);
//     float front_left_y = round(y_cordinate + box_dimension);
    
//     if(!found_nodes.hasValue(front_left_x,front_left_y,z_cordinate)){
//         if(!occupied_points.hasValue(front_left_x,front_left_y,z_cordinate)){
//             struct Graph_Node *temp_node = new Graph_Node;
//             temp_node->x_cordinate = front_left_x;
//             temp_node->y_cordinate = front_left_y;
//             temp_node->z_cordinate = z_cordinate;
//             temp_node->path_cost = current_node->path_cost + 1;
//             temp_node->predecessor = current_node;
//             temp_node->priority = 0.0;
//             breadth_array_free_cells[breadth_array_free_cells_size] = temp_node;
//             breadth_array_free_cells_size = breadth_array_free_cells_size + 1;
//             printf("( Return on 1: %f %f %f )\n",front_left_x,front_left_y,z_cordinate);
//         }else{
//             struct Graph_Node *temp_node = new Graph_Node;
//             temp_node->x_cordinate = front_left_x;
//             temp_node->y_cordinate = front_left_y;
//             temp_node->z_cordinate = z_cordinate;
//             temp_node->path_cost = current_node->path_cost + 1;
//             temp_node->predecessor = current_node;
//             temp_node->priority = 0.0;
//             found_nodes.setValue(front_left_x,front_left_y,z_cordinate,temp_node);
//             int traversable = checkSquareCondition(front_left_x,front_left_y,z_cordinate,0.5);
//             // if(traversable){
//                 node_queue.push(temp_node);
//                 //printf("( %f %f %f )\n",front_left_x,front_left_y,z_cordinate);
//             // }  
//         }
//     }else{
//         if(!occupied_points.hasValue(front_left_x,front_left_y,z_cordinate)){

//         }else{
//             struct Graph_Node *available_node = found_nodes.getValue(front_left_x,front_left_y,z_cordinate);
//             if(available_node->path_cost>current_node->path_cost+1){
//                 available_node->path_cost = current_node->path_cost + 1;
//                 available_node->predecessor = current_node;
//                 available_node->priority = 0.0;
//             }
//         }
//     }

//     float left_x = round(x_cordinate);
//     float left_y = round(y_cordinate + box_dimension);
    
//     if(!found_nodes.hasValue(left_x,left_y,z_cordinate)){
//         if(!occupied_points.hasValue(left_x,left_y,z_cordinate)){
//             struct Graph_Node *temp_node = new Graph_Node;
//             temp_node->x_cordinate = left_x;
//             temp_node->y_cordinate = left_y;
//             temp_node->z_cordinate = z_cordinate;
//             temp_node->path_cost = current_node->path_cost + 1;
//             temp_node->predecessor = current_node;
//             temp_node->priority = 0.0;
//             breadth_array_free_cells[breadth_array_free_cells_size] = temp_node;
//             breadth_array_free_cells_size = breadth_array_free_cells_size + 1;
//             printf("( Return on 1: %f %f %f )\n",left_x,left_y,z_cordinate);
//         }else{
//             struct Graph_Node *temp_node = new Graph_Node;
//             temp_node->x_cordinate = left_x;
//             temp_node->y_cordinate = left_y;
//             temp_node->z_cordinate = z_cordinate;
//             temp_node->path_cost = current_node->path_cost + 1;
//             temp_node->predecessor = current_node;
//             temp_node->priority = 0.0;
//             found_nodes.setValue(left_x,left_y,z_cordinate,temp_node);
//             // int traversable = checkSquareCondition(left_x,left_y,z_cordinate,0.5);
//             // if(traversable){
//                 node_queue.push(temp_node);
//             // } 
//         }
//     }else{
//         if(!occupied_points.hasValue(left_x,left_y,z_cordinate)){

//         }else{
//             struct Graph_Node *available_node = found_nodes.getValue(left_x,left_y,z_cordinate);
//             if(available_node->path_cost>current_node->path_cost+1){
//                 available_node->path_cost = current_node->path_cost + 1;
//                 available_node->predecessor = current_node;
//                 available_node->priority = 0.0;
//             }
//         }
//     }

//     float back_left_x = round(x_cordinate - box_dimension);
//     float back_left_y = round(y_cordinate + box_dimension);
    
//     if(!found_nodes.hasValue(back_left_x,back_left_y,z_cordinate)){
//         if(!occupied_points.hasValue(back_left_x,back_left_y,z_cordinate)){
//             struct Graph_Node *temp_node = new Graph_Node;
//             temp_node->x_cordinate = back_left_x;
//             temp_node->y_cordinate = back_left_y;
//             temp_node->z_cordinate = z_cordinate;
//             temp_node->path_cost = current_node->path_cost + 1;
//             temp_node->predecessor = current_node;
//             temp_node->priority = 0.0;
//             breadth_array_free_cells[breadth_array_free_cells_size] = temp_node;
//             breadth_array_free_cells_size = breadth_array_free_cells_size + 1;
//             printf("( Return on 1: %f %f %f )\n",back_left_x,back_left_y,z_cordinate);
//         }else{
//             struct Graph_Node *temp_node = new Graph_Node;
//             temp_node->x_cordinate = back_left_x;
//             temp_node->y_cordinate = back_left_y;
//             temp_node->z_cordinate = z_cordinate;
//             temp_node->path_cost = current_node->path_cost + 1;
//             temp_node->predecessor = current_node;
//             temp_node->priority = 0.0;
//             found_nodes.setValue(back_left_x,back_left_y,z_cordinate,temp_node);
//             // int traversable = checkSquareCondition(back_left_x,back_left_y,z_cordinate,0.5);
//             // if(traversable){
//                 node_queue.push(temp_node);
//             // } 
//         }
//     }else{
//         if(!occupied_points.hasValue(back_left_x,back_left_y,z_cordinate)){

//         }else{
//         struct Graph_Node *available_node = found_nodes.getValue(back_left_x,back_left_y,z_cordinate);
//             if(available_node->path_cost>current_node->path_cost+1){
//                 available_node->path_cost = current_node->path_cost + 1;
//                 available_node->predecessor = current_node;
//                 available_node->priority = 0.0;
//             } 
//         }
//     }

//     float back_x = round(x_cordinate - box_dimension);
//     float back_y = round(y_cordinate);
    
//     if(!found_nodes.hasValue(back_x,back_y,z_cordinate)){
//         if(!occupied_points.hasValue(back_x,back_y,z_cordinate)){
//             struct Graph_Node *temp_node = new Graph_Node;
//             temp_node->x_cordinate = back_x;
//             temp_node->y_cordinate = back_y;
//             temp_node->z_cordinate = z_cordinate;
//             temp_node->path_cost = current_node->path_cost + 1;
//             temp_node->predecessor = current_node;
//             temp_node->priority = 0.0;
//             breadth_array_free_cells[breadth_array_free_cells_size] = temp_node;
//             breadth_array_free_cells_size = breadth_array_free_cells_size + 1;
//             printf("( Return on 1: %f %f %f )\n",back_x,back_y,z_cordinate);
//         }else{
//             struct Graph_Node *temp_node = new Graph_Node;
//             temp_node->x_cordinate = back_x;
//             temp_node->y_cordinate = back_y;
//             temp_node->z_cordinate = z_cordinate;
//             temp_node->path_cost = current_node->path_cost + 1;
//             temp_node->predecessor = current_node;
//             temp_node->priority = 0.0;
//             found_nodes.setValue(back_x,back_y,z_cordinate,temp_node);
//             // int traversable = checkSquareCondition(back_x,back_y,z_cordinate,0.5);
//             // if(traversable){
//                 node_queue.push(temp_node);
//                 //printf("( %f %f %f )\n",back_x,back_y,z_cordinate);
//             // }
//         }
//     }else{
//         if(!occupied_points.hasValue(back_x,back_y,z_cordinate)){

//         }else{
//             struct Graph_Node *available_node = found_nodes.getValue(back_x,back_y,z_cordinate);
//             if(available_node->path_cost>current_node->path_cost+1){
//                 available_node->path_cost = current_node->path_cost + 1;
//                 available_node->predecessor = current_node;
//                 available_node->priority = 0.0;
//             }
//         }
//     }

//     float back_right_x = round(x_cordinate - box_dimension);
//     float back_right_y = round(y_cordinate - box_dimension);
    
//     if(!found_nodes.hasValue(back_right_x,back_right_y,z_cordinate)){
//         if(!occupied_points.hasValue(back_right_x,back_right_y,z_cordinate)){
//             struct Graph_Node *temp_node = new Graph_Node;
//             temp_node->x_cordinate = back_right_x;
//             temp_node->y_cordinate = back_right_y;
//             temp_node->z_cordinate = z_cordinate;
//             temp_node->path_cost = current_node->path_cost + 1;
//             temp_node->predecessor = current_node;
//             temp_node->priority = 0.0;
//             breadth_array_free_cells[breadth_array_free_cells_size] = temp_node;
//             breadth_array_free_cells_size = breadth_array_free_cells_size + 1;
//             printf("( Return on 1: %f %f %f )\n",back_right_x,back_right_y,z_cordinate);
//         }else{
//             struct Graph_Node *temp_node = new Graph_Node;
//             temp_node->x_cordinate = back_right_x;
//             temp_node->y_cordinate = back_right_y;
//             temp_node->z_cordinate = z_cordinate;
//             temp_node->path_cost = current_node->path_cost + 1;
//             temp_node->predecessor = current_node;
//             temp_node->priority = 0.0;
//             found_nodes.setValue(back_right_x,back_right_y,z_cordinate,temp_node);
//             // int traversable = checkSquareCondition(back_right_x,back_right_y,z_cordinate,0.5);
//             // if(traversable){
//             node_queue.push(temp_node);
//                 //printf("( %f %f %f )\n",back_right_x,back_right_y,z_cordinate);
//             // }
//         }
//     }else{
//         if(!occupied_points.hasValue(back_right_x,back_right_y,z_cordinate)){

//         }else{
//             struct Graph_Node *available_node = found_nodes.getValue(back_right_x,back_right_y,z_cordinate);
//             if(available_node->path_cost>current_node->path_cost+1){
//                 available_node->path_cost = current_node->path_cost + 1;
//                 available_node->predecessor = current_node;
//                 available_node->priority = 0.0;
//             }
//         }
//     }

//     float right_x = round(x_cordinate);
//     float right_y = round(y_cordinate - box_dimension);
    
//     if(!found_nodes.hasValue(right_x,right_y,z_cordinate)){
//         if(!occupied_points.hasValue(right_x,right_y,z_cordinate)){
//             struct Graph_Node *temp_node = new Graph_Node;
//             temp_node->x_cordinate = right_x;
//             temp_node->y_cordinate = right_y;
//             temp_node->z_cordinate = z_cordinate;
//             temp_node->path_cost = current_node->path_cost + 1;
//             temp_node->predecessor = current_node;
//             temp_node->priority = 0.0;
//             breadth_array_free_cells[breadth_array_free_cells_size] = temp_node;
//             breadth_array_free_cells_size = breadth_array_free_cells_size + 1;
//             printf("( Return on 1 : %f %f %f )\n",right_x,right_y,z_cordinate);
//         }else{
//             struct Graph_Node *temp_node = new Graph_Node;
//             temp_node->x_cordinate = right_x;
//             temp_node->y_cordinate = right_y;
//             temp_node->z_cordinate = z_cordinate;
//             temp_node->path_cost = current_node->path_cost + 1;
//             temp_node->predecessor = current_node;
//             temp_node->priority = 0.0;
//             found_nodes.setValue(right_x,right_y,z_cordinate,temp_node);
//             // int traversable = checkSquareCondition(right_x,right_y,z_cordinate,0.5);
//             // if(traversable){
//                 node_queue.push(temp_node);
//             // }
//         }
//     }else{
//         if(!occupied_points.hasValue(right_x,right_y,z_cordinate)){

//         }else{
//             struct Graph_Node *available_node = found_nodes.getValue(right_x,right_y,z_cordinate);
//             if(available_node->path_cost>current_node->path_cost+1){
//                 available_node->path_cost = current_node->path_cost + 1;
//                 available_node->predecessor = current_node;
//                 available_node->priority = 0.0;
//             }  
//         }
//     }

//     float front_right_x = round(x_cordinate + box_dimension);
//     float front_right_y = round(y_cordinate - box_dimension);
    
//     if(!found_nodes.hasValue(front_right_x,front_right_y,z_cordinate)){
//         if(!occupied_points.hasValue(front_right_x,front_right_y,z_cordinate)){
//             struct Graph_Node *temp_node = new Graph_Node;
//             temp_node->x_cordinate = front_right_x;
//             temp_node->y_cordinate = front_right_y;
//             temp_node->z_cordinate = z_cordinate;
//             temp_node->path_cost = current_node->path_cost + 1;
//             temp_node->predecessor = current_node;
//             temp_node->priority = 0.0;
//             breadth_array_free_cells[breadth_array_free_cells_size] = temp_node;
//             breadth_array_free_cells_size = breadth_array_free_cells_size + 1;
//             printf("( Return on 1: %f %f %f )\n",front_right_x,front_right_y,z_cordinate);
//         }else{
//             struct Graph_Node *temp_node = new Graph_Node;
//             temp_node->x_cordinate = front_right_x;
//             temp_node->y_cordinate = front_right_y;
//             temp_node->z_cordinate = z_cordinate;
//             temp_node->path_cost = current_node->path_cost + 1;
//             temp_node->predecessor = current_node;
//             temp_node->priority = 0.0;
//             found_nodes.setValue(front_right_x,front_right_y,z_cordinate,temp_node);
//             // int traversable = checkSquareCondition(front_right_x,front_right_y,z_cordinate,0.5);
//             // if(traversable){
//                 node_queue.push(temp_node);
//             // }
//         }
//     }else{
//         if(!occupied_points.hasValue(front_right_x,front_right_y,z_cordinate)){

//         }else{
//             struct Graph_Node *available_node = found_nodes.getValue(front_right_x,front_right_y,z_cordinate);
//             if(available_node->path_cost>current_node->path_cost+1){
//                 available_node->path_cost = current_node->path_cost + 1;
//                 available_node->predecessor = current_node;
//                 available_node->priority = 0.0;
//             }
//         }
//     }

//     if(breadth_array_free_cells_size == 0){
//         if(node_queue.empty()){
//             printf("kg");
//             while(current_node!= NULL){
//                 printf("Node Traversed = %f %f %f )\n",current_node->x_cordinate,current_node->y_cordinate,current_node->z_cordinate);
//                 current_node = current_node->predecessor;
//             }
//             return;
//         }else{
//             struct Graph_Node *next_node = node_queue.front();
//             node_queue.pop();
//             printf("End One Round\n");
//             getAdjecentSquareCentroids(next_node->x_cordinate,next_node->y_cordinate,next_node->z_cordinate,box_dimension,next_node);
//         }    
//     }else{
//         int i;
//         int minimum_position = 0;
//         double minimum_path_cost = 100000000;

//         for (i=0; i < breadth_array_free_cells_size;i++ ) {
//             printf("for loop\n");
//             struct Graph_Node *temp_node = breadth_array_free_cells[i];
//             printf("breadth array\n");
//             int temp_path_cost = temp_node->predecessor->path_cost + 1;
//             printf("path cost\n");
//             if(temp_path_cost<minimum_path_cost){
//                 minimum_position = i;
//                 minimum_path_cost = temp_path_cost;
//             }
//         }

//         struct Graph_Node *minimum_cost_node = breadth_array_free_cells[minimum_position];
        
//         for (i=0; i < breadth_array_free_cells_size;i++ ) {
//             delete breadth_array_free_cells[i];
//         }
//         breadth_array_free_cells_size = 0;
//         printf("deleted\n");
//         while(minimum_cost_node != NULL){
//             printf("Node Traversed = %f %f %f )\n",minimum_cost_node->x_cordinate,minimum_cost_node->y_cordinate,minimum_cost_node->z_cordinate);
//             minimum_cost_node=minimum_cost_node->predecessor;
//         } 
//         return;
//     }    
// }

// void initialize_first_node(float x_cordinate,float y_cordinate, float z_cordinate){
//     struct Graph_Node *temp_node = new Graph_Node;
//     temp_node->x_cordinate = x_cordinate;
//     temp_node->y_cordinate = y_cordinate;
//     temp_node->z_cordinate = z_cordinate;
//     temp_node->path_cost = 0;
//     temp_node->predecessor = NULL;
//     temp_node->priority = 0.0;
//     start_node = temp_node;
//     getAdjecentSquareCentroids(x_cordinate,y_cordinate,z_cordinate,0.500000,start_node);
// }

// void retrieveDataFromOctomap(const octomap_msgs::OctomapConstPtr& msg){
//     AbstractOcTree* tree = msgToMap(*msg);
//     OcTree* octree = dynamic_cast<OcTree*>(tree); 

//     float count = 0;
//     for(OcTree::leaf_iterator it = octree->begin_leafs(),end=octree->end_leafs(); it!= end; ++it){
//         float point_x = it.getX();
//         float point_y = it.getY();
//         float point_z = it.getZ();
//         if(it->getValue()>0){
//             if(!occupied_points.hasValue(point_x,point_y,point_z)){
//                 occupied_points.setValue(point_x,point_y,point_z,1);
//                 printf("( %f %f %f )\n",point_x,point_y,point_z);
//             }
//             count = count+1;
//         }else{
//             if(!occupied_points.hasValue(point_x,point_y,point_z)){
//                 occupied_points.setValue(point_x,point_y,point_z,0);
//             }
//         }
        
//     }
//     initialize_first_node(3.825000,1.625000,-0.025000);
// }

// int main (int argc, char** argv) {
//     ros::init (argc, argv, "cloud_sub");
//     ros::NodeHandle node_handler;
//     ros::Rate loop_rate(1000);
//     ros::Subscriber subscriber_node;
//     // subscriber_node = node_handler.subscribe<PointCloud>("/octomap_point_cloud_centers", 100, point_cloud_subscriber);
//     subscriber_node = node_handler.subscribe("/octomap_full", 10000, retrieveDataFromOctomap);
//     // subscriber_node = node_handler.subscribe("/odom", 1, update_odometry_data);

//     ros::spin();   
//  }

#include <Eigen/SVD>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
// #include <octomap/octomap.h>
#include <octomap/OcTree.h>
// #include <octomap/OcTreeLUT.h>
#include <fstream> 
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <list>
#include <cmath>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

using namespace octomap;
using namespace octomath;
using std::cout;
using std::endl;

#define MAX_ITER 500


void tree2PointCloud(OcTree *tree, pcl::PointCloud<pcl::PointXYZ>& pclCloud){
    cout<<"Acheived tree to point cloud" << endl << endl;
    int count = 0;
    for(OcTree::leaf_iterator it = tree->begin_leafs(),end = tree->end_leafs(); it != end; ++it){
        if(tree->isNodeOccupied(*it)){
            pclCloud.push_back(pcl::PointXYZ(it.getX(), it.getY(), it.getZ()));
            count++;
        }
    }
    printf("Count is = %d\n",count);
}

int getSign(double value){
    if(value == 0){
        return 0;
    }else if(value > 0){
        return 1;
    }else{
        return -1;
    }
}

bool pointInBBox(pcl::PointXYZ& point, pcl::PointXYZ& bboxMin, pcl::PointXYZ& bboxMax){
    return (point.x < bboxMax.x && point.x > bboxMin.x) && (point.y < bboxMax.y && point.y > bboxMin.y) && (point.z < bboxMax.z && point.z > bboxMin.z);
}


Eigen::Matrix4f getICPTransformation(pcl::PointCloud<pcl::PointXYZ>& cloud1,pcl::PointCloud<pcl::PointXYZ>& cloud2, Eigen::Matrix4f& tfEst, double mapRes){
    cout<<"Started ICPTransformation" << endl << endl;
    pcl::transformPointCloud(cloud2,cloud2,tfEst);
    pcl::PointXYZ minCloud1;
    pcl::PointXYZ maxCloud1;
    pcl::getMinMax3D(cloud1,minCloud1,maxCloud1);
    cout<<"Passed first step ICPTransformation" << endl << endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2filtered(new pcl::PointCloud<pcl::PointXYZ>);

    for(pcl::PointCloud<pcl::PointXYZ>::iterator it=cloud2.begin(); it!=cloud2.end();it++){
        if(pointInBBox(*it,minCloud1,maxCloud1)){
            cloud2filtered->push_back(*it);
        }
    }

    cout<<"Passed Second step ICPTransformation" << endl << endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1filtered(new pcl::PointCloud<pcl::PointXYZ>);


    pcl::PointXYZ minCloud2filtered;
    pcl::PointXYZ maxCloud2filtered;
    pcl::getMinMax3D(*cloud2filtered,minCloud2filtered,maxCloud2filtered);

    minCloud2filtered = pcl::PointXYZ(minCloud2filtered.x-1,minCloud2filtered.y-1,minCloud2filtered.z-1);
    maxCloud2filtered = pcl::PointXYZ(maxCloud2filtered.x+1,maxCloud2filtered.y+1,maxCloud2filtered.z+1);

    for(pcl::PointCloud<pcl::PointXYZ>::iterator it=cloud1.begin(); it!=cloud1.end(); it++){
        if(pointInBBox(*it,minCloud2filtered,maxCloud2filtered)){
            cloud1filtered->push_back(*it);
        }
    }

    cout<<"Passed Third step ICPTransformation" << endl << endl;

    PointCloud::Ptr src(new PointCloud);
    PointCloud::Ptr tgt(new PointCloud);
    pcl::VoxelGrid<PointT> grid;
    grid.setLeafSize(mapRes*10,mapRes*10,mapRes*10);
    grid.setInputCloud(cloud1filtered);
    grid.filter(*tgt);
    
    grid.setInputCloud(cloud2filtered);
    grid.filter(*src);

    pcl::IterativeClosestPointNonLinear<PointT,PointT> reg;
    reg.setTransformationEpsilon(mapRes/60);
    reg.setMaxCorrespondenceDistance(10*mapRes);

    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(),prev;
    PointCloud::Ptr reg_result;

    cout<<"Passed Fourth step ICPTransformation" << endl << endl;

    printf("Size of : %lu\n", src->size());
    printf("Size of : %lu\n", tgt->size());

    if(src->size() < tgt->size()){
        cout<<"Inside main if of ICPTransformation" << endl << endl;
        reg.setInputSource(src);
        reg.setInputTarget(tgt);
        reg_result = src;
        reg.setMaximumIterations(2);
        for(int i=0l; i<MAX_ITER; ++i){
            src = reg_result;
            reg.setInputSource(src);
            reg.align(*reg_result);
            Ti = reg.getFinalTransformation()* Ti;

            if(reg.getMaxCorrespondenceDistance() > 0.2){
                if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
                    reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 1);
            }else if(reg.getMaxCorrespondenceDistance()>0.002){
                if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
                    reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
            }
            prev = reg.getLastIncrementalTransformation();
        }
    }else{

        cout<<"Inside main else of ICPTransformation" << endl << endl;

        reg.setInputSource(tgt);
        reg.setInputTarget(src);

        reg_result = tgt;
        reg.setMaximumIterations(2);
        for(int i=0; i<MAX_ITER; ++i){
            tgt = reg_result;
            reg.setInputSource(tgt);
            reg.align(*reg_result);
            Ti = reg.getFinalTransformation()*Ti;

            if(reg.getMaxCorrespondenceDistance() > 0.2){
                if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
                    reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 1);
            }else if(reg.getMaxCorrespondenceDistance()>0.002){
                if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
                    reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
            }
            prev = reg.getLastIncrementalTransformation();
        }
        Ti = Ti.inverse();    
    }
    return Ti*tfEst;
}

void transformTree(OcTree *tree, Eigen::Matrix4f& transform){
    double treeRes = tree->getResolution();
    OcTree* transformed = new OcTree(treeRes);

    Eigen::Matrix3f rotation;
    Eigen::Matrix3f invRotation;
    Eigen::Matrix4f invTransform;
    rotation << transform(0,0),transform(0,1),transform(0,2),
                transform(1,0),transform(1,1),transform(1,2),
                transform(2,0),transform(2,1),transform(2,2);
    invRotation = rotation.transpose();
    invTransform << transform(0,0),transform(0,1),transform(0,2), -transform(0,3),
                transform(1,0),transform(1,1),transform(1,2), -transform(1,3),
                transform(2,0),transform(2,1),transform(2,2), -transform(2,3),
                0,0,0,1;

    double minX, maxX, minY, maxY, minZ, maxZ;

    tree->getMetricMin(minX,minY,minZ);
    tree->getMetricMax(maxX,maxY,maxZ);

    //OcTreeLUT ocTreeLUT(treeRes);
    std::vector<point3d> points;

    points.push_back(point3d(maxX,minY,minZ));
    points.push_back(point3d(minX,minY,minZ));
    points.push_back(point3d(minX,maxY,minZ));
    points.push_back(point3d(maxX,maxY,minZ));
    points.push_back(point3d(maxX,minY,maxZ));
    points.push_back(point3d(minX,minY,maxZ));
    points.push_back(point3d(minX,maxY,maxZ));
    points.push_back(point3d(maxX,maxY,maxZ));

    for(unsigned i=0; i<points.size(); i++){
        Eigen::Vector4f point(points[i].x(),points[i].y(),points[i].z(),1);
        point = transform * point;
        points[i] = point3d(point(0),point(1),point(2));
    }

    minX = points[0].x();
    maxX = points[0].x();
    minY = points[0].y();
    maxY = points[0].y();
    minZ = points[0].z();
    maxZ = points[0].z();

    for(unsigned i=0; i<points.size(); i++){
        minX = (points[i].x()<minX)?points[i].x():minX;
        maxX = (points[i].y()<minY)?points[i].y():minY;
        minY = (points[i].z()<minZ)?points[i].z():minZ;
        maxY = (points[i].x()<maxX)?points[i].x():maxX;
        minZ = (points[i].y()<maxY)?points[i].y():maxY;
        maxZ = (points[i].z()<maxZ)?points[i].z():maxZ;
    }

    for(double z=minZ-treeRes/2; z<(maxZ+treeRes/2);z+=treeRes){
        for(double y=minY-treeRes/2; y<(maxY+treeRes/2);y+=treeRes){
            for(double x=minX-treeRes/2; x<(maxX+treeRes/2);x+=treeRes){
                OcTreeKey destVoxel = transformed->coordToKey(point3d(x,y,z));
                Eigen::Vector4f point(x,y,z,1);
                point = invTransform*point;
                point3d sourcePoint = point3d(point(0),point(1),point(2));
                OcTreeKey sourceVoxel = tree->coordToKey(sourcePoint);
                point3d nn=tree->keyToCoord(sourceVoxel);

                OcTreeNode *oldNode = tree->search(sourceVoxel);

                double c000,c001,c010,c011,c100,c101,c110,c111,c00,c01,c10,c11,c0,c1;
                double xd,yd,zd;

                xd = (sourcePoint.x()-nn.x())/treeRes;
                yd = (sourcePoint.y()-nn.y())/treeRes;
                zd = (sourcePoint.z()-nn.z())/treeRes;

                if(oldNode!=NULL){
                    c000 = oldNode->getOccupancy();
                    OcTreeNode *node;

                    if((node = tree->search(point3d(nn.x(),nn.y(),nn.z()+getSign(zd)*treeRes)))!=NULL){
                        c001 = node->getOccupancy();                        
                    }else{
                        c001 = 0;
                    }

                    if((node = tree->search(point3d(nn.x(),nn.y()+getSign(yd)*treeRes,nn.z())))!=NULL){
                        c010 = node->getOccupancy();                        
                    }else{
                        c010 = 0;
                    }

                    if((node = tree->search(point3d(nn.x(),nn.y()+getSign(yd)*treeRes,nn.z()+getSign(zd)*treeRes)))!=NULL){
                        c011 = node->getOccupancy();                        
                    }else{
                        c011 = 0;
                    }

                    if((node = tree->search(point3d(nn.x()+getSign(xd)*treeRes,nn.y(),nn.z())))!=NULL){
                        c100 = node->getOccupancy();                        
                    }else{
                        c100 = 0;
                    }

                    if((node = tree->search(point3d(nn.x()+getSign(xd)*treeRes,nn.y(),nn.z()+getSign(zd)*treeRes)))!=NULL){
                        c101 = node->getOccupancy();                        
                    }else{
                        c101 = 0;
                    }

                    if((node = tree->search(point3d(nn.x()+getSign(xd)*treeRes,nn.y()+getSign(yd)*treeRes,nn.z())))!=NULL){
                        c110 = node->getOccupancy();                        
                    }else{
                        c110 = 0;
                    }

                    if((node = tree->search(point3d(nn.x()+getSign(xd)*treeRes,nn.y()+getSign(yd)*treeRes,nn.z()+getSign(zd)*treeRes)))!=NULL){
                        c111 = node->getOccupancy();                        
                    }else{
                        c111 = 0;
                    }

                    c00 = (1-fabs(xd))*c000 + fabs(xd)*c100;
                    c01 = (1-fabs(xd))*c001 + fabs(xd)*c101;
                    c10 = (1-fabs(xd))*c010 + fabs(xd)*c110;
                    c11 = (1-fabs(xd))*c011 + fabs(xd)*c111;

                    c0 = (1-fabs(yd))*c00 + fabs(yd)*c10;
                    c1 = (1-fabs(yd))*c01 + fabs(yd)*c11;

                    OcTreeNode *newNode = transformed->updateNode(destVoxel,true);
                    newNode->setLogOdds(logodds((-fabs(zd))*c0 + fabs(zd)*c1));
                }
            }
        }

    }

    tree->swapContent(*transformed);
    delete transformed;
}


void expandLevel(std::vector<OcTreeNode *> *nodePtrs){
    unsigned size = nodePtrs->size();
    cout<<"Reached Second Expand" << endl << endl;
    for(unsigned i=0; i<size; i++){
        OcTreeNode *parent = nodePtrs->front();
        
        parent->expandNode();
        nodePtrs->erase(nodePtrs->begin());
        for(unsigned j=0; j<8; j++){
            nodePtrs->push_back(parent->getChild(j));
        }
    }
}

unsigned expandNodeMultiLevel(OcTree *tree, OcTreeNode *node, unsigned currentDepth, unsigned levels){
    if(currentDepth == (int)tree->getTreeDepth()){
        return 0;
    }

    int levelsCounter = 0;
    std::vector<OcTreeNode *> nodePtrs;
    nodePtrs.push_back(node);

    for(unsigned i=0; i<levels; i++){
        if(currentDepth == (int)tree->getTreeDepth()){
            return levelsCounter;
        }
        expandLevel(&nodePtrs);
        levelsCounter++;
        currentDepth++;
    }
    return levelsCounter;
}

int getNodeDepth(OcTree* tree, point3d& point, OcTreeNode* node){
    for(int depth=tree->getTreeDepth(); depth>1; depth--){
        if(tree->search(point,depth)==node){
            return depth;
        }
    }
    return -1;
}

int main(int argc, char** argv){
    std::string filename1 = std::string(argv[1]);
    std::string filename2 = std::string(argv[2]);
    std::string outputFileName = std::string(argv[3]);

    cout<< "\nReading octree files......\n";

    double roll, pitch, yaw;
    double res = 0.05;

    point3d translation;
    if(argc == 7 || argc == 10){
        translation = point3d(atof(argv[4]),atof(argv[5]),atof(argv[6]));
    }

    if(argc == 10){
        roll = atof(argv[7]);
        pitch = atof(argv[8]);
        yaw = atof(argv[9]);
    }else{
        roll = 0;
        pitch = 0;
        yaw = 0;
    }

    Pose6D pose(translation.x(),translation.y(),translation.z(),roll,pitch,yaw);
    Eigen::Matrix4f transform;
    std::vector<double> coeffs;
    pose.rot().toRotMatrix(coeffs);

    transform<< coeffs[0], coeffs[1], coeffs[2], translation.x(),
                coeffs[3], coeffs[4], coeffs[5], translation.y(),
                coeffs[6], coeffs[7], coeffs[8], translation.z(),
                0,0,0,1;
    
    OcTree* tree1 = dynamic_cast<OcTree*>(OcTree::read(filename1));
    OcTree* tree2 = dynamic_cast<OcTree*>(OcTree::read(filename2));

    std::cout<<transform<<std::endl;

    cout<<"Registering map to improve tf estimate" << endl << endl;

    pcl::PointCloud<pcl::PointXYZ> tree1Points;
    tree2PointCloud(tree1, tree1Points);
    pcl::PointCloud<pcl::PointXYZ> tree2Points;
    tree2PointCloud(tree2,tree2Points);

    transform = getICPTransformation(tree1Points, tree2Points, transform,res);

    printf("passedICP %d\n",1);
    if(roll != 0 || pitch != 0 || yaw != 0 || translation.x() != 0 || translation.y() != 0 || translation.z() != 0){
        transformTree(tree2,transform);
    }

    for(OcTree::leaf_iterator it = tree2->begin_leafs(); it != tree2->end_leafs(); ++it){
        if(tree2->isNodeOccupied(*it)){
            it->setLogOdds(logodds(0.6));
        }

        OcTreeNode *nodeIn1 = tree1->search(it.getCoordinate());
        OcTreeKey nodeKey = tree1->coordToKey(it.getCoordinate());
        point3d point = it.getCoordinate();
        if(nodeIn1 != NULL){
            int depthIn1 = getNodeDepth(tree1, point, nodeIn1);
            if(depthIn1 != -1){
                int depthDiff = it.getDepth() - depthIn1;
                if(depthDiff == 0){
                    tree1->updateNode(nodeKey, it->getLogOdds());
                }else if(depthDiff > 0){
                    for(int i=0; i<depthDiff; i++){
                        if(depthIn1 == (int)tree1->getTreeDepth()){
                            break;
                        }
                        cout<<"Reached first expand" << endl << endl;
                        nodeIn1->expandNode();
                        nodeKey = tree1->coordToKey(point);
                        depthIn1++;
                    }
                    nodeIn1->setLogOdds(logodds(nodeIn1->getOccupancy()+it->getOccupancy()));
                }else if(depthDiff<0){
                    expandNodeMultiLevel(tree2,tree2->search(point),it.getDepth(),abs(depthDiff));
                }
            }
        }else{
            OcTreeNode* newNode = tree1->updateNode(point,true);
            newNode->setLogOdds(it->getLogOdds());
        }
    }

    std::cout<<"Compressing merged result\n";
    tree1->prune();
    tree1->write(outputFileName);
    delete tree1;
    delete tree2;
}




