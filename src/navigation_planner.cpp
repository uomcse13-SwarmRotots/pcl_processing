#include "navigation_planner.h"

NavigationPlanner::NavigationPlanner(ros::NodeHandle &nh, std::string topic){
    node_handle_ = nh;
    topic_ = topic;

}

NavigationPlanner::~NavigationPlanner(){}

inline float NavigationPlanner::round(float val){
    val=val*1000;
    if(val<0){
        return ceil(val-0.5)/1000;
    }
    return floor(val+0.5)/1000;
}

void NavigationPlanner::clearVariables(){
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


int NavigationPlanner::checkSquareCondition(float x_cordinate, float y_cordinate, float z_cordinate, float box_dimension){
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

void NavigationPlanner::convert_odom_angle_to_radians(float yaw_in_radians){
    if(yaw_in_radians<0){
        yaw_in_radians = 360 - abs(yaw_in_radians)*180/PI_F;
    }
    current_yaw_angle = yaw_in_radians;
}

double NavigationPlanner::get_turning_angle_between_two_points(double start_x_cordinate,double start_y_cordinate, double end_x_cordinate, double end_y_cordinate){
    float angle_between_points = atan2(end_y_cordinate - start_y_cordinate, end_x_cordinate - start_x_cordinate)*180/PI_F;
    return angle_between_points;
}

double NavigationPlanner::calculate_weight_to_the_point(struct Graph_Node *temp_current_node){
    float rotation = 0;
    float distance = 0;
    while(temp_current_node != NULL){
        if(temp_current_node->predecessor!=NULL & temp_current_node->predecessor->predecessor==NULL ){
            rotation = rotation + abs(current_yaw_angle - get_turning_angle_between_two_points(temp_current_node->x_cordinate,temp_current_node->y_cordinate,temp_current_node->predecessor->x_cordinate,temp_current_node->predecessor->y_cordinate));
            distance = distance + 1;
            return rotation*100+distance*1000;//,distance
        }else{
            rotation = rotation + abs(get_turning_angle_between_two_points(temp_current_node->x_cordinate,temp_current_node->y_cordinate,temp_current_node->predecessor->x_cordinate,temp_current_node->predecessor->y_cordinate));
            distance = distance + 1;
        }    
    }
}

void NavigationPlanner::set_publish_end_point(double x_cordinate, double y_cordinate, double z_cordinate){
    printf("ending cordinates are = %f %f %f \n",x_cordinate,y_cordinate,z_cordinate);
//     ros::init(argc, argv, "target_point_publisher");
//     ros::NodeHandle n;
//     ros::Publisher chatter_pub = n.advertise<std_msgs::String>("target_publisher", 1000);

//     ros::Rate loop_rate(10);

//     int count = 0;
//     while (ros::ok())  {
//         std_msgs::String msg;
//         std::stringstream ss;
//         ss << x_cordinate) + y_cordinate) + z_cordinate)<< count;
//         msg.data = ss.str();
//         ROS_INFO("%s", msg.data.c_str());
//         chatter_pub.publish(msg);
//         ros::spinOnce();
//         loop_rate.sleep();
//         ++count;
//   }
//   return 0;
}

void NavigationPlanner::getAdjecentSquareCentroids(float x_cordinate,float y_cordinate, float z_cordinate, float box_dimension,struct Graph_Node *node){
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
            breadth_array_free_cells[breadth_array_free_cells_size] = temp_node;
            breadth_array_free_cells_size = breadth_array_free_cells_size + 1;
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

        }else{
            struct Graph_Node *available_node = found_nodes.getValue(front_x,front_y,z_cordinate);
            if(available_node->path_cost>current_node->path_cost+1){
                available_node->path_cost = current_node->path_cost + 1;
                available_node->predecessor = current_node;
                available_node->priority = 0.0;
            }
        }
    }
    
    float front_left_x = round(x_cordinate + box_dimension);
    float front_left_y = round(y_cordinate + box_dimension);
    
    if(!found_nodes.hasValue(front_left_x,front_left_y,z_cordinate)){
        if(!occupied_points.hasValue(front_left_x,front_left_y,z_cordinate)){
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = front_left_x;
            temp_node->y_cordinate = front_left_y;
            temp_node->z_cordinate = z_cordinate;
            temp_node->path_cost = current_node->path_cost + 1;
            temp_node->predecessor = current_node;
            temp_node->priority = 0.0;
            breadth_array_free_cells[breadth_array_free_cells_size] = temp_node;
            breadth_array_free_cells_size = breadth_array_free_cells_size + 1;
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

        }else{
            struct Graph_Node *available_node = found_nodes.getValue(front_left_x,front_left_y,z_cordinate);
            if(available_node->path_cost>current_node->path_cost+1){
                available_node->path_cost = current_node->path_cost + 1;
                available_node->predecessor = current_node;
                available_node->priority = 0.0;
            }
        }
    }

    float left_x = round(x_cordinate);
    float left_y = round(y_cordinate + box_dimension);
    
    if(!found_nodes.hasValue(left_x,left_y,z_cordinate)){
        if(!occupied_points.hasValue(left_x,left_y,z_cordinate)){
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = left_x;
            temp_node->y_cordinate = left_y;
            temp_node->z_cordinate = z_cordinate;
            temp_node->path_cost = current_node->path_cost + 1;
            temp_node->predecessor = current_node;
            temp_node->priority = 0.0;
            breadth_array_free_cells[breadth_array_free_cells_size] = temp_node;
            breadth_array_free_cells_size = breadth_array_free_cells_size + 1;
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
            // } 
        }
    }else{
        if(!occupied_points.hasValue(left_x,left_y,z_cordinate)){

        }else{
            struct Graph_Node *available_node = found_nodes.getValue(left_x,left_y,z_cordinate);
            if(available_node->path_cost>current_node->path_cost+1){
                available_node->path_cost = current_node->path_cost + 1;
                available_node->predecessor = current_node;
                available_node->priority = 0.0;
            }
        }
    }

    float back_left_x = round(x_cordinate - box_dimension);
    float back_left_y = round(y_cordinate + box_dimension);
    
    if(!found_nodes.hasValue(back_left_x,back_left_y,z_cordinate)){
        if(!occupied_points.hasValue(back_left_x,back_left_y,z_cordinate)){
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = back_left_x;
            temp_node->y_cordinate = back_left_y;
            temp_node->z_cordinate = z_cordinate;
            temp_node->path_cost = current_node->path_cost + 1;
            temp_node->predecessor = current_node;
            temp_node->priority = 0.0;
            breadth_array_free_cells[breadth_array_free_cells_size] = temp_node;
            breadth_array_free_cells_size = breadth_array_free_cells_size + 1;
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
            // } 
        }
    }else{
        if(!occupied_points.hasValue(back_left_x,back_left_y,z_cordinate)){

        }else{
        struct Graph_Node *available_node = found_nodes.getValue(back_left_x,back_left_y,z_cordinate);
            if(available_node->path_cost>current_node->path_cost+1){
                available_node->path_cost = current_node->path_cost + 1;
                available_node->predecessor = current_node;
                available_node->priority = 0.0;
            } 
        }
    }

    float back_x = round(x_cordinate - box_dimension);
    float back_y = round(y_cordinate);
    
    if(!found_nodes.hasValue(back_x,back_y,z_cordinate)){
        if(!occupied_points.hasValue(back_x,back_y,z_cordinate)){
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = back_x;
            temp_node->y_cordinate = back_y;
            temp_node->z_cordinate = z_cordinate;
            temp_node->path_cost = current_node->path_cost + 1;
            temp_node->predecessor = current_node;
            temp_node->priority = 0.0;
            breadth_array_free_cells[breadth_array_free_cells_size] = temp_node;
            breadth_array_free_cells_size = breadth_array_free_cells_size + 1;
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

        }else{
            struct Graph_Node *available_node = found_nodes.getValue(back_x,back_y,z_cordinate);
            if(available_node->path_cost>current_node->path_cost+1){
                available_node->path_cost = current_node->path_cost + 1;
                available_node->predecessor = current_node;
                available_node->priority = 0.0;
            }
        }
    }

    float back_right_x = round(x_cordinate - box_dimension);
    float back_right_y = round(y_cordinate - box_dimension);
    
    if(!found_nodes.hasValue(back_right_x,back_right_y,z_cordinate)){
        if(!occupied_points.hasValue(back_right_x,back_right_y,z_cordinate)){
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = back_right_x;
            temp_node->y_cordinate = back_right_y;
            temp_node->z_cordinate = z_cordinate;
            temp_node->path_cost = current_node->path_cost + 1;
            temp_node->predecessor = current_node;
            temp_node->priority = 0.0;
            breadth_array_free_cells[breadth_array_free_cells_size] = temp_node;
            breadth_array_free_cells_size = breadth_array_free_cells_size + 1;
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

        }else{
            struct Graph_Node *available_node = found_nodes.getValue(back_right_x,back_right_y,z_cordinate);
            if(available_node->path_cost>current_node->path_cost+1){
                available_node->path_cost = current_node->path_cost + 1;
                available_node->predecessor = current_node;
                available_node->priority = 0.0;
            }
        }
    }

    float right_x = round(x_cordinate);
    float right_y = round(y_cordinate - box_dimension);
    
    if(!found_nodes.hasValue(right_x,right_y,z_cordinate)){
        if(!occupied_points.hasValue(right_x,right_y,z_cordinate)){
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = right_x;
            temp_node->y_cordinate = right_y;
            temp_node->z_cordinate = z_cordinate;
            temp_node->path_cost = current_node->path_cost + 1;
            temp_node->predecessor = current_node;
            temp_node->priority = 0.0;
            breadth_array_free_cells[breadth_array_free_cells_size] = temp_node;
            breadth_array_free_cells_size = breadth_array_free_cells_size + 1;
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
            // }
        }
    }else{
        if(!occupied_points.hasValue(right_x,right_y,z_cordinate)){

        }else{
            struct Graph_Node *available_node = found_nodes.getValue(right_x,right_y,z_cordinate);
            if(available_node->path_cost>current_node->path_cost+1){
                available_node->path_cost = current_node->path_cost + 1;
                available_node->predecessor = current_node;
                available_node->priority = 0.0;
            }  
        }
    }

    float front_right_x = round(x_cordinate + box_dimension);
    float front_right_y = round(y_cordinate - box_dimension);
    
    if(!found_nodes.hasValue(front_right_x,front_right_y,z_cordinate)){
        if(!occupied_points.hasValue(front_right_x,front_right_y,z_cordinate)){
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = front_right_x;
            temp_node->y_cordinate = front_right_y;
            temp_node->z_cordinate = z_cordinate;
            temp_node->path_cost = current_node->path_cost + 1;
            temp_node->predecessor = current_node;
            temp_node->priority = 0.0;
            breadth_array_free_cells[breadth_array_free_cells_size] = temp_node;
            breadth_array_free_cells_size = breadth_array_free_cells_size + 1;
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
            // }
        }
    }else{
        if(!occupied_points.hasValue(front_right_x,front_right_y,z_cordinate)){

        }else{
            struct Graph_Node *available_node = found_nodes.getValue(front_right_x,front_right_y,z_cordinate);
            if(available_node->path_cost>current_node->path_cost+1){
                available_node->path_cost = current_node->path_cost + 1;
                available_node->predecessor = current_node;
                available_node->priority = 0.0;
            }
        }
    }

    if(breadth_array_free_cells_size == 0){
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

        for (i=0; i < breadth_array_free_cells_size;i++ ) {
            printf("for loop\n");
            struct Graph_Node *temp_node = breadth_array_free_cells[i];
            printf("breadth array\n");
            int temp_path_cost = temp_node->predecessor->path_cost + 1;
            printf("path cost\n");
            if(temp_path_cost<minimum_path_cost){
                minimum_position = i;
                minimum_path_cost = temp_path_cost;
            }
        }

        struct Graph_Node *minimum_cost_node = breadth_array_free_cells[minimum_position];
        printf("free_cells\n");
        for (i=0; i < breadth_array_free_cells_size;i++ ) {
            delete breadth_array_free_cells[i];
        }
        breadth_array_free_cells_size = 0;
        printf("deleted\n");
        while(minimum_cost_node != NULL){
            printf("Node Traversed = %f %f %f )\n",minimum_cost_node->x_cordinate,minimum_cost_node->y_cordinate,minimum_cost_node->z_cordinate);
            minimum_cost_node=minimum_cost_node->predecessor;
        } 
        return;
    }    
}

void NavigationPlanner::initialize_first_node(float x_cordinate,float y_cordinate, float z_cordinate){
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

int **NavigationPlanner::check_neighbourhood(const geometry_msgs::PoseStamped& pose, float box_dimension){
    float x_cordinate = pose.pose.position.x;
    float y_cordinate = pose.pose.position.y;
    float z_cordinate = pose.pose.position.z;

    int** array = 0;
    array = new int*[3];
    for (int h = 0; h < 3; h++){
        array[h] = new int[3];
        for (int w = 0; w < 3; w++){
                array[h][w] = 0;
        }
    }

    float front_x = round(x_cordinate + box_dimension);
    float front_y = round(y_cordinate);
    array[1][2] = checkSquareCondition(front_x,front_y,z_cordinate,box_dimension);

    float front_left_x = round(x_cordinate + box_dimension);
    float front_left_y = round(y_cordinate + box_dimension);
    array[0][2] = checkSquareCondition(front_left_x,front_left_y,z_cordinate,box_dimension);

    float left_x = round(x_cordinate);
    float left_y = round(y_cordinate + box_dimension);
    array[0][1] = checkSquareCondition(left_x,left_y,z_cordinate,box_dimension);
    
    float back_left_x = round(x_cordinate - box_dimension);
    float back_left_y = round(y_cordinate + box_dimension);
    array[0][0] = checkSquareCondition(back_left_x,back_left_y,z_cordinate,box_dimension);
    
    float back_x = round(x_cordinate - box_dimension);
    float back_y = round(y_cordinate);
    array[1][0] = checkSquareCondition(back_x,back_y,z_cordinate,box_dimension);
    
    float back_right_x = round(x_cordinate - box_dimension);
    float back_right_y = round(y_cordinate - box_dimension);
    array[2][0] = checkSquareCondition(back_right_x,back_right_y,z_cordinate,box_dimension);
    
    float right_x = round(x_cordinate);
    float right_y = round(y_cordinate - box_dimension);
    array[2][1] = checkSquareCondition(right_x,right_y,z_cordinate,box_dimension);
    
    float front_right_x = round(x_cordinate + box_dimension);
    float front_right_y = round(y_cordinate - box_dimension);
    array[2][2] = checkSquareCondition(front_right_x,front_right_y,z_cordinate,box_dimension);
    
    return array;
}

void NavigationPlanner::retrieveDataFromOctomap(const octomap_msgs::OctomapConstPtr& msg){
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
                // printf("( %f %f %f )\n",point_x,point_y,point_z);
            }
            count = count+1;
        }else{
            if(!occupied_points.hasValue(point_x,point_y,point_z)){
                occupied_points.setValue(point_x,point_y,point_z,0);
            }
        }
        
    }
    // initialize_first_node(0.675000,4.725000,0.025000);
}

void NavigationPlanner::neighbourhood_callback(const geometry_msgs::PoseStamped& pose){
    int **return_data = check_neighbourhood(pose,0.5);
    int i=0,j=0;
    for(i;i<3;i++){
        for(j;j<3;j++){
            ROS_INFO("Point %d %d value = %d",i,j,return_data[i][j]);
        }
    }    
}

void NavigationPlanner::start(){
    subscriber_node = node_handle_.subscribe(topic_, 1000, &NavigationPlanner::retrieveDataFromOctomap,this);
    ros::spin();
}