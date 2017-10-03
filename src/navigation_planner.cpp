#include "navigation_planner.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);



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

float NavigationPlanner::getRoundedPoint(float cordinate){
    // printf("cordinate %f\n",cordinate);
    if(cordinate != 0.000000f){
        float mod_cordinate = 1000 * cordinate;
        int mod_cordinate_int = (int)mod_cordinate;
        bool is_float = false;
        if(cordinate<0){
            is_float = true;
            
        }
        mod_cordinate_int = abs(mod_cordinate_int);
        int final_int;
        float final_number;
        
        std::string mod_cordinate_str = boost::lexical_cast<std::string>(mod_cordinate_int);
        string last_characters = mod_cordinate_str.substr(mod_cordinate_str.length() - 2);
        try {
            int last_int = boost::lexical_cast<int>(last_characters);
            
            if(last_int == 25 | last_int == 75){
                final_int = mod_cordinate_int;
            }else if(last_int > 25 & last_int < 50){
                final_int = mod_cordinate_int - (last_int - 25);
            }else if((last_int >= 50 & last_int < 75)){
                final_int = mod_cordinate_int + (75 - last_int);
            }else if(last_int > 75 & last_int < 100){
                final_int = mod_cordinate_int - (last_int - 75);
            }else if(last_int >= 0 & last_int < 25){
                final_int = mod_cordinate_int + (25 - last_int);
            }

            
            final_number= (float)final_int/1000;
            if(is_float){
                final_number = (-1) * final_number;
            }
            // printf("final number %f\n",final_number);
        } catch( boost::bad_lexical_cast const& ) {
            // std::cout << "Error: input string was not valid" << std::endl;
        }
        return final_number;
    }else{
        return 0.000000f;
    }
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
                    return 1;
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
    return -1;
        
}

void NavigationPlanner::convertOdomAngleToRadians(float yaw_in_radians){
    if(yaw_in_radians<0){
        yaw_in_radians = 360 - abs(yaw_in_radians)*180/PI_F;
    }
    current_yaw_angle = yaw_in_radians;
}

float NavigationPlanner::getTurningAngleBetweenTwoPoints(float start_x_cordinate,float start_y_cordinate, float end_x_cordinate, float end_y_cordinate){
    // printf("start turning \n");
    float angle_between_points_radians = atan2(end_y_cordinate - start_y_cordinate, end_x_cordinate - start_x_cordinate)*180/PI_F;
    float angle_in_degrees = angle_between_points_radians * 180 / PI_F;
    // printf("return turning \n");
    return angle_in_degrees;
}

float NavigationPlanner::calculateWeightToThePoint(struct Graph_Node *temp_current_node, float yaw_in_radians){
    stack<struct Graph_Node*> navigation_stack;

    while(temp_current_node != NULL){
        navigation_stack.push(temp_current_node);
        temp_current_node = temp_current_node->predecessor;
    }
    // printf("stack created \n");
    float angle_cost;
    float previous_angle = 100000;
    struct Graph_Node *current_node;
    while(!navigation_stack.empty()){
        struct Graph_Node *current_node = navigation_stack.top();
        navigation_stack.pop();
        // printf("turn angle \n");
        float temp_angle;
        if(navigation_stack.empty()){
            // printf("return stack \n");
            return angle_cost;
        }else{
            // printf("in else \n");
            temp_angle = getTurningAngleBetweenTwoPoints(current_node->x_cordinate,current_node->y_cordinate, navigation_stack.top()->x_cordinate,navigation_stack.top()->y_cordinate);
            // printf("passed critical else \n");
            if(previous_angle == 100000){
                previous_angle = yaw_in_radians * 180 / PI_F;
                // printf("return previous \n"); 
            }
            if(previous_angle == temp_angle){
                angle_cost += 0;
            }else if((previous_angle<0 & temp_angle<0) | (previous_angle>=0 & temp_angle>=0)){
                angle_cost += abs(previous_angle - temp_angle);
            }else {
                angle_cost += abs(previous_angle) + abs(temp_angle);
            }
            printf("Angle Cost : %f\n", angle_cost);
        }
        previous_angle = temp_angle;
    }
}


void NavigationPlanner::publishEndPoint(float x_cordinate, float y_cordinate, float z_cordinate){
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
    
    if(!found_nodes.hasValue(x_cordinate,y_cordinate,0)){
        found_nodes.setValue(x_cordinate,y_cordinate,0,current_node);
    }

    float front_x = round(x_cordinate + box_dimension);
    float front_y = round(y_cordinate);
    

    if(checkSquareCondition(front_x,front_y,z_cordinate, box_dimension) == -1){
        struct Graph_Node *temp_node = new Graph_Node;
        temp_node->x_cordinate = front_x;
        temp_node->y_cordinate = front_y;
        temp_node->z_cordinate = z_cordinate;
        temp_node->path_cost = current_node->path_cost + 1;
        temp_node->predecessor = current_node;
        temp_node->priority = 0.0;
        breadth_empty_queue.push(temp_node);
        printf("( Uninitialized Nodes: %f %f %f )\n",front_x,front_y,z_cordinate);
    }else if(checkSquareCondition(front_x,front_y,z_cordinate, box_dimension) == 1){
        if(!found_nodes.hasValue(front_x,front_y,0)){
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = front_x;
            temp_node->y_cordinate = front_y;
            temp_node->z_cordinate = z_cordinate;
            temp_node->path_cost = current_node->path_cost + 1;
            temp_node->predecessor = current_node;
            temp_node->priority = 0.0;
            found_nodes.setValue(front_x,front_y,0,temp_node);
            node_queue.push(temp_node);
            printf("( Traversable Nodes : %f %f %f )\n",front_x,front_y,z_cordinate);
        }    
    }else{
        //found_nodes.setValue(front_x,front_y,z_cordinate,temp_node);
    }
    
    
    float front_left_x = round(x_cordinate + box_dimension);
    float front_left_y = round(y_cordinate + box_dimension);
    
    
    if(checkSquareCondition(front_left_x,front_left_y,z_cordinate, box_dimension) == -1){
        struct Graph_Node *temp_node = new Graph_Node;
        temp_node->x_cordinate = front_x;
        temp_node->y_cordinate = front_y;
        temp_node->z_cordinate = z_cordinate;
        temp_node->path_cost = current_node->path_cost + 1;
        temp_node->predecessor = current_node;
        temp_node->priority = 0.0;
        breadth_empty_queue.push(temp_node);
        printf("( Uninitialized Nodes: %f %f %f )\n",front_left_x,front_left_y,z_cordinate);
    }else if(checkSquareCondition(front_left_x,front_left_y,z_cordinate, box_dimension) == 1){
        if(!found_nodes.hasValue(front_left_x,front_left_y,0)){
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = front_x;
            temp_node->y_cordinate = front_y;
            temp_node->z_cordinate = z_cordinate;
            temp_node->path_cost = current_node->path_cost + 1;
            temp_node->predecessor = current_node;
            temp_node->priority = 0.0;
            found_nodes.setValue(front_left_x,front_left_y,0,temp_node);
            node_queue.push(temp_node);
            printf("( Traversable Nodes : %f %f %f )\n",front_left_x,front_left_y,z_cordinate);
        }
    }else{
        //found_nodes.setValue(front_left_x,front_left_y,z_cordinate,temp_node);
    }
    

    float left_x = round(x_cordinate);
    float left_y = round(y_cordinate + box_dimension);
    
    if(checkSquareCondition(left_x,left_y,z_cordinate, box_dimension) == -1){
        struct Graph_Node *temp_node = new Graph_Node;
        temp_node->x_cordinate = front_x;
        temp_node->y_cordinate = front_y;
        temp_node->z_cordinate = z_cordinate;
        temp_node->path_cost = current_node->path_cost + 1;
        temp_node->predecessor = current_node;
        temp_node->priority = 0.0;
        breadth_empty_queue.push(temp_node);
        printf("( Uninitialized Nodes: %f %f %f )\n",left_x,left_y,z_cordinate);
    }else if(checkSquareCondition(left_x,left_y,z_cordinate, box_dimension) == 1){
        if(!found_nodes.hasValue(left_x,left_y,0)){
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = front_x;
            temp_node->y_cordinate = front_y;
            temp_node->z_cordinate = z_cordinate;
            temp_node->path_cost = current_node->path_cost + 1;
            temp_node->predecessor = current_node;
            temp_node->priority = 0.0;
            found_nodes.setValue(left_x,left_y,0,temp_node);
            node_queue.push(temp_node);
            printf("( Traversable Nodes : %f %f %f )\n",left_x,left_y,z_cordinate);
        }
    }else{
        //found_nodes.setValue(left_x,left_y,z_cordinate,temp_node);
    }

    float back_left_x = round(x_cordinate - box_dimension);
    float back_left_y = round(y_cordinate + box_dimension);
    
    
    if(checkSquareCondition(back_left_x,back_left_y,z_cordinate, box_dimension) == -1){
        struct Graph_Node *temp_node = new Graph_Node;
        temp_node->x_cordinate = front_x;
        temp_node->y_cordinate = front_y;
        temp_node->z_cordinate = z_cordinate;
        temp_node->path_cost = current_node->path_cost + 1;
        temp_node->predecessor = current_node;
        temp_node->priority = 0.0;
        breadth_empty_queue.push(temp_node);
        printf("( Uninitialized Nodes: %f %f %f )\n",back_left_x,back_left_y,z_cordinate);
    }else if(checkSquareCondition(back_left_x,back_left_y,z_cordinate, box_dimension) == 1){
        if(!found_nodes.hasValue(back_left_x,back_left_y,0)){
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = front_x;
            temp_node->y_cordinate = front_y;
            temp_node->z_cordinate = z_cordinate;
            temp_node->path_cost = current_node->path_cost + 1;
            temp_node->predecessor = current_node;
            temp_node->priority = 0.0;
            found_nodes.setValue(back_left_x,back_left_y,0,temp_node);
            node_queue.push(temp_node);
            printf("( Traversable Nodes : %f %f %f )\n",back_left_x,back_left_y,z_cordinate);
        }
    }else{
        //found_nodes.setValue(back_left_x,back_left_y,z_cordinate,temp_node);
    }

    float back_x = round(x_cordinate - box_dimension);
    float back_y = round(y_cordinate);
    
    if(checkSquareCondition(back_x,back_y,z_cordinate, box_dimension) == -1){
        struct Graph_Node *temp_node = new Graph_Node;
        temp_node->x_cordinate = front_x;
        temp_node->y_cordinate = front_y;
        temp_node->z_cordinate = z_cordinate;
        temp_node->path_cost = current_node->path_cost + 1;
        temp_node->predecessor = current_node;
        temp_node->priority = 0.0;
        breadth_empty_queue.push(temp_node);
        printf("( Uninitialized Nodes: %f %f %f )\n",back_x,back_y,z_cordinate);
    }else if(checkSquareCondition(back_x,back_y,z_cordinate, box_dimension) == 1){
        if(!found_nodes.hasValue(back_x,back_y,0)){
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = front_x;
            temp_node->y_cordinate = front_y;
            temp_node->z_cordinate = z_cordinate;
            temp_node->path_cost = current_node->path_cost + 1;
            temp_node->predecessor = current_node;
            temp_node->priority = 0.0;
            found_nodes.setValue(back_x,back_y,0,temp_node);
            node_queue.push(temp_node);
            printf("( Traversable Nodes : %f %f %f )\n",back_x,back_y,z_cordinate);
        }
    }else{
        //found_nodes.setValue(back_x,back_y,z_cordinate,temp_node);
    }

    float back_right_x = round(x_cordinate - box_dimension);
    float back_right_y = round(y_cordinate - box_dimension);
    
    if(checkSquareCondition(back_right_x,back_right_y,z_cordinate, box_dimension) == -1){
        struct Graph_Node *temp_node = new Graph_Node;
        temp_node->x_cordinate = front_x;
        temp_node->y_cordinate = front_y;
        temp_node->z_cordinate = z_cordinate;
        temp_node->path_cost = current_node->path_cost + 1;
        temp_node->predecessor = current_node;
        temp_node->priority = 0.0;
        breadth_empty_queue.push(temp_node);
        printf("( Uninitialized Nodes: %f %f %f )\n",back_right_x,back_right_y,z_cordinate);
    }else if(checkSquareCondition(back_right_x,back_right_y,z_cordinate, box_dimension) == 1){
        if(!found_nodes.hasValue(back_right_x,back_right_y,0)){
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = front_x;
            temp_node->y_cordinate = front_y;
            temp_node->z_cordinate = z_cordinate;
            temp_node->path_cost = current_node->path_cost + 1;
            temp_node->predecessor = current_node;
            temp_node->priority = 0.0;
            found_nodes.setValue(back_right_x,back_right_y,0,temp_node);
            node_queue.push(temp_node);
            printf("( Traversable Nodes : %f %f %f )\n",back_right_x,back_right_y,z_cordinate);
        }
    }else{
        //found_nodes.setValue(back_right_x,back_right_y,z_cordinate,temp_node);
    }

    float right_x = round(x_cordinate);
    float right_y = round(y_cordinate - box_dimension);
    
    if(checkSquareCondition(right_x,right_y,z_cordinate, box_dimension) == -1){
        struct Graph_Node *temp_node = new Graph_Node;
        temp_node->x_cordinate = front_x;
        temp_node->y_cordinate = front_y;
        temp_node->z_cordinate = z_cordinate;
        temp_node->path_cost = current_node->path_cost + 1;
        temp_node->predecessor = current_node;
        temp_node->priority = 0.0;
        breadth_empty_queue.push(temp_node);
        printf("( Uninitialized Nodes: %f %f %f )\n",right_x,right_y,z_cordinate);
    }else if(checkSquareCondition(right_x,right_y,z_cordinate, box_dimension) == 1){
        if(!found_nodes.hasValue(right_x,right_y,0)){
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = front_x;
            temp_node->y_cordinate = front_y;
            temp_node->z_cordinate = z_cordinate;
            temp_node->path_cost = current_node->path_cost + 1;
            temp_node->predecessor = current_node;
            temp_node->priority = 0.0;
            found_nodes.setValue(right_x,right_y,0,temp_node);
            node_queue.push(temp_node);
            printf("( Traversable Nodes : %f %f %f )\n",right_x,right_y,z_cordinate);
        }
    }else{
        //found_nodes.setValue(right_x,right_y,z_cordinate,temp_node);
    }

    float front_right_x = round(x_cordinate + box_dimension);
    float front_right_y = round(y_cordinate - box_dimension);
    
    if(checkSquareCondition(front_right_x,front_right_y,z_cordinate, box_dimension) == -1){
        struct Graph_Node *temp_node = new Graph_Node;
        temp_node->x_cordinate = front_x;
        temp_node->y_cordinate = front_y;
        temp_node->z_cordinate = z_cordinate;
        temp_node->path_cost = current_node->path_cost + 1;
        temp_node->predecessor = current_node;
        temp_node->priority = 0.0;
        breadth_empty_queue.push(temp_node);
        printf("( Uninitialized Nodes: %f %f %f )\n",front_right_x,front_right_y,z_cordinate);
    }else if(checkSquareCondition(front_right_x,front_right_y,z_cordinate, box_dimension) == 1){
        if(!found_nodes.hasValue(front_right_x,front_right_y,0)){
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = front_x;
            temp_node->y_cordinate = front_y;
            temp_node->z_cordinate = z_cordinate;
            temp_node->path_cost = current_node->path_cost + 1;
            temp_node->predecessor = current_node;
            temp_node->priority = 0.0;
            found_nodes.setValue(front_right_x,front_right_y,0,temp_node);
            node_queue.push(temp_node);
            printf("( Traversable Nodes : %f %f %f )\n",front_right_x,front_right_y,z_cordinate);
        }
    }else{
        //found_nodes.setValue(front_right_x,front_right_y,z_cordinate,temp_node);
    }

}


/*
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
            breadth_empty_queue.push(temp_node);
            printf("( Uninitialized Nodes: %f %f %f )\n",front_x,front_y,z_cordinate);
            // breadth_array_free_cells[breadth_array_free_cells_size] = temp_node;
            // breadth_array_free_cells_size = breadth_array_free_cells_size + 1;
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
                printf("( Traversable Nodes : %f %f %f )\n",front_x,front_y,z_cordinate);
            // }    
        }
    }else{
        if(!occupied_points.hasValue(front_x,front_y,z_cordinate)){

        }else{
            printf("(Node Found Previous - Try To Update)\n");
            struct Graph_Node *available_node = found_nodes.getValue(front_x,front_y,z_cordinate);
            if(available_node->path_cost>current_node->path_cost+1){
                available_node->path_cost = current_node->path_cost + 1;
                available_node->predecessor = current_node;
                available_node->priority = 0.0;
                printf("( Updated Nodes: %f %f %f )\n",front_x,front_y,z_cordinate);
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
            breadth_empty_queue.push(temp_node);
            // breadth_array_free_cells[breadth_array_free_cells_size] = temp_node;
            // breadth_array_free_cells_size = breadth_array_free_cells_size + 1;
            printf("( Uninitialized Nodes: %f %f %f )\n",front_left_x,front_left_y,z_cordinate);
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
                printf("( Traversable Nodes : %f %f %f )\n",front_left_x,front_left_y,z_cordinate);
            // }  
        }
    }else{
        if(!occupied_points.hasValue(front_left_x,front_left_y,z_cordinate)){

        }else{
            printf("(Node Found Previous - Try To Update)\n");
            struct Graph_Node *available_node = found_nodes.getValue(front_left_x,front_left_y,z_cordinate);
            if(available_node->path_cost>current_node->path_cost+1){
                available_node->path_cost = current_node->path_cost + 1;
                available_node->predecessor = current_node;
                available_node->priority = 0.0;
                printf("( Updated Nodes: %f %f %f )\n",front_left_x,front_left_y,z_cordinate);
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
            breadth_empty_queue.push(temp_node);
            // breadth_array_free_cells[breadth_array_free_cells_size] = temp_node;
            // breadth_array_free_cells_size = breadth_array_free_cells_size + 1;
            printf("( Uninitialized Nodes: %f %f %f )\n",left_x,left_y,z_cordinate);
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
                printf("( Traversable Nodes : %f %f %f )\n",left_x,left_y,z_cordinate);
                node_queue.push(temp_node);
            // } 
        }
    }else{
        if(!occupied_points.hasValue(left_x,left_y,z_cordinate)){

        }else{
            printf("(Node Found Previous - Try To Update)\n");
            struct Graph_Node *available_node = found_nodes.getValue(left_x,left_y,z_cordinate);
            if(available_node->path_cost>current_node->path_cost+1){
                available_node->path_cost = current_node->path_cost + 1;
                available_node->predecessor = current_node;
                available_node->priority = 0.0;
                printf("( Updated Nodes: %f %f %f )\n",left_x,left_y,z_cordinate);
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
            breadth_empty_queue.push(temp_node);
            // breadth_array_free_cells[breadth_array_free_cells_size] = temp_node;
            // breadth_array_free_cells_size = breadth_array_free_cells_size + 1;
            printf("( Uninitialized Nodes: %f %f %f )\n",back_left_x,back_left_y,z_cordinate);
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
                printf("( Traversable Nodes : %f %f %f )\n",back_left_x,back_left_y,z_cordinate);
                node_queue.push(temp_node);
            // } 
        }
    }else{
        if(!occupied_points.hasValue(back_left_x,back_left_y,z_cordinate)){

        }else{
            printf("(Node Found Previous - Try To Update)\n");
            struct Graph_Node *available_node = found_nodes.getValue(back_left_x,back_left_y,z_cordinate);
            if(available_node->path_cost>current_node->path_cost+1){
                available_node->path_cost = current_node->path_cost + 1;
                available_node->predecessor = current_node;
                available_node->priority = 0.0;
                printf("( Updated Nodes: %f %f %f )\n",back_left_x,back_left_y,z_cordinate);
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
            breadth_empty_queue.push(temp_node);
            // breadth_array_free_cells[breadth_array_free_cells_size] = temp_node;
            // breadth_array_free_cells_size = breadth_array_free_cells_size + 1;
            printf("( Uninitialized Nodes: %f %f %f )\n",back_x,back_y,z_cordinate);
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
                printf("( Traversable Nodes : %f %f %f )\n",back_x,back_y,z_cordinate);
            // }
        }
    }else{
        if(!occupied_points.hasValue(back_x,back_y,z_cordinate)){

        }else{
            printf("(Node Found Previous - Try To Update)\n");
            struct Graph_Node *available_node = found_nodes.getValue(back_x,back_y,z_cordinate);
            if(available_node->path_cost>current_node->path_cost+1){
                available_node->path_cost = current_node->path_cost + 1;
                available_node->predecessor = current_node;
                available_node->priority = 0.0;
                printf("( Updated Nodes: %f %f %f )\n",back_x,back_y,z_cordinate);
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
            breadth_empty_queue.push(temp_node);
            // breadth_array_free_cells[breadth_array_free_cells_size] = temp_node;
            // breadth_array_free_cells_size = breadth_array_free_cells_size + 1;
            printf("( Uninitialized Nodes: %f %f %f )\n",back_right_x,back_right_y,z_cordinate);
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
                printf("( Traversable Nodes : %f %f %f )\n",back_right_x,back_right_y,z_cordinate);
            // }
        }
    }else{
        if(!occupied_points.hasValue(back_right_x,back_right_y,z_cordinate)){

        }else{
            printf("(Node Found Previous - Try To Update)\n");
            struct Graph_Node *available_node = found_nodes.getValue(back_right_x,back_right_y,z_cordinate);
            if(available_node->path_cost>current_node->path_cost+1){
                available_node->path_cost = current_node->path_cost + 1;
                available_node->predecessor = current_node;
                available_node->priority = 0.0;
                printf("( Updated Nodes: %f %f %f )\n",back_right_x,back_right_y,z_cordinate);
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
            breadth_empty_queue.push(temp_node);
            // breadth_array_free_cells[breadth_array_free_cells_size] = temp_node;
            // breadth_array_free_cells_size = breadth_array_free_cells_size + 1;
            printf("( Uninitialized Nodes : %f %f %f )\n",right_x,right_y,z_cordinate);
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
                printf("( Traversable Nodes : %f %f %f )\n",right_x,right_y,z_cordinate);
            // }
        }
    }else{
        if(!occupied_points.hasValue(right_x,right_y,z_cordinate)){

        }else{
            printf("(Node Found Previous - Try To Update)\n");
            struct Graph_Node *available_node = found_nodes.getValue(right_x,right_y,z_cordinate);
            if(available_node->path_cost>current_node->path_cost+1){
                available_node->path_cost = current_node->path_cost + 1;
                available_node->predecessor = current_node;
                available_node->priority = 0.0;
                printf("( Updated Nodes: %f %f %f )\n",right_x,right_y,z_cordinate);
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
            breadth_empty_queue.push(temp_node);
            // breadth_array_free_cells[breadth_array_free_cells_size] = temp_node;
            // breadth_array_free_cells_size = breadth_array_free_cells_size + 1;
            printf("( Uninitialized Nodes: %f %f %f )\n",front_right_x,front_right_y,z_cordinate);
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
                printf("( Traversable Nodes : %f %f %f )\n",front_right_x,front_right_y,z_cordinate);
            // }
        }
    }else{
        if(!occupied_points.hasValue(front_right_x,front_right_y,z_cordinate)){

        }else{
            printf("(Node Found Previous - Try To Update)\n");
            struct Graph_Node *available_node = found_nodes.getValue(front_right_x,front_right_y,z_cordinate);
            if(available_node->path_cost>current_node->path_cost+1){
                available_node->path_cost = current_node->path_cost + 1;
                available_node->predecessor = current_node;
                available_node->priority = 0.0;
                printf("( Updated Nodes: %f %f %f )\n",front_right_x,front_right_y,z_cordinate);
            }
        }
    }

    // if(// breadth_array_free_cells_size == 0){
    //     if(node_queue.empty()){
    //         printf("kg");
    //         while(current_node!= NULL){
    //             printf("Node Traversed = %f %f %f )\n",current_node->x_cordinate,current_node->y_cordinate,current_node->z_cordinate);
    //             current_node = current_node->predecessor;
    //         }
    //         return;
    //     }else{
    //         struct Graph_Node *next_node = node_queue.front();
    //         node_queue.pop();
    //         printf("End One Round\n");
    //         getAdjecentSquareCentroids(next_node->x_cordinate,next_node->y_cordinate,next_node->z_cordinate,box_dimension,next_node);
    //     }    
    // }else{
    //     int i;
    //     int minimum_position = 0;
    //     float minimum_path_cost = 100000000;

    //     for (i=0; i < breadth_array_free_cells_size;i++ ) {
    //         printf("for loop\n");
    //         struct Graph_Node *temp_node = breadth_array_free_cells[i];
    //         printf("breadth array\n");
    //         int temp_path_cost = temp_node->predecessor->path_cost + 1;
    //         printf("path cost\n");
    //         if(temp_path_cost<minimum_path_cost){
    //             minimum_position = i;
    //             minimum_path_cost = temp_path_cost;
    //         }
    //     }

    //     struct Graph_Node *minimum_cost_node = breadth_array_free_cells[minimum_position];
    //     printf("free_cells\n");
    //     for (i=0; i < breadth_array_free_cells_size;i++ ) {
    //         delete breadth_array_free_cells[i];
    //     }
    //     // breadth_array_free_cells_size = 0;
    //     printf("deleted\n");
    //     while(minimum_cost_node != NULL){
    //         printf("Node Traversed = %f %f %f )\n",minimum_cost_node->x_cordinate,minimum_cost_node->y_cordinate,minimum_cost_node->z_cordinate);
    //         minimum_cost_node=minimum_cost_node->predecessor;
    //     } 
    //     return;
    // }    
}
*/


void NavigationPlanner::initializeFrstNode(float x_cordinate,float y_cordinate, float z_cordinate,float box_dimension){
    struct Graph_Node *temp_node = new Graph_Node;
    temp_node->x_cordinate = x_cordinate;
    temp_node->y_cordinate = y_cordinate;
    temp_node->z_cordinate = z_cordinate;
    temp_node->path_cost = 0;
    temp_node->predecessor = NULL;
    temp_node->priority = 0.0;
    start_node = temp_node;
    getAdjecentSquareCentroids(x_cordinate,y_cordinate,z_cordinate,box_dimension,start_node);
}

int **NavigationPlanner::checkNeighbourhood(const geometry_msgs::PoseStamped& pose, float box_dimension){
    float x_cordinate = pose.pose.position.x;
    float y_cordinate = pose.pose.position.y;
    float z_cordinate = pose.pose.position.z;

    float resolution = box_dimension;
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
}

void NavigationPlanner::retrieveDataFromOctomap(const octomap_msgs::OctomapConstPtr& msg){
    printf("start");
    AbstractOcTree* tree = fullMsgToMap(*msg);
    OcTree* octree = dynamic_cast<OcTree*>(tree); 

    int count = 0;
    int count1 = 0;
    for(OcTree::tree_iterator it = octree->begin_tree(),end=octree->end_tree(); it!= end; ++it){
        // float point_x = round(getRoundedPoint(it.getX()));
        // float point_y = round(getRoundedPoint(it.getY()));
        // float point_z = round(getRoundedPoint(it.getZ()));
        float point_x = it.getX();
        float point_y = it.getY();
        float point_z = it.getZ();

        // if(it.isLeaf()){
            printf("(  %f %f %f %f)\n",point_x,point_y,point_z,it->getValue());
            
            if(it->getValue()>0){
                // if(!occupied_points.hasValue(point_x,point_y,point_z)){
                occupied_points.setValue(point_x,point_y,point_z,1);
                count = count+1;   
                // }
                // printf("( Octree Nodes : %f %f %f %f)\n",point_x,point_y,point_z,it->getValue());
            
            }else{
                // if(!occupied_points.hasValue(point_x,point_y,point_z)){
                occupied_points.setValue(point_x,point_y,point_z,0);  
                count1++;  
                // }
                
                // if(point_x==1.325000f & point_y== 2.525000f){
                //     printf("( Free Octree Nodes : %f %f %f %f)\n",point_x,point_y,point_z,it->getValue());
                // }    
            }
        // }
       
        // if(it->getValue()>0){
        //     // if(!occupied_points.hasValue(point_x,point_y,point_z)){
        //     occupied_points.setValue(point_x,point_y,point_z,1);    
        //     // }
        //     // printf("( Octree Nodes : %f %f %f %f)\n",point_x,point_y,point_z,it->getValue());
           
        // }else{
        //     // if(!occupied_points.hasValue(point_x,point_y,point_z)){
        //     occupied_points.setValue(point_x,point_y,point_z,0);    
        //     // }
            
        //     // if(point_x==1.325000f & point_y== 2.525000f){
        //     //     printf("( Free Octree Nodes : %f %f %f %f)\n",point_x,point_y,point_z,it->getValue());
        //     // }
            
        // }
        if(point_x==1.325000f & point_y== 2.525000f){
            // printf("( Free Octree Nodes : %f %f %f %f)\n",point_x,point_y,point_z,it->getValue());
        }
        
    }
    printf("count occupied %d\n",count);
    printf("count free %d\n",count1);
    // struct Graph_Node *best_node = getBreadthFirstSearchNodes(4.475000,2.225000,-0.025000,0.500000);
    // while(best_node != NULL){
    //     printf("Traversing Points = %f %f %f\n", best_node->x_cordinate, best_node->y_cordinate, best_node->z_cordinate);
    //     best_node = best_node->predecessor;
    // }
    // initializeFrstNode(0.675000,4.725000,0.025000);
}

void NavigationPlanner::neighbourhoodCallback(const geometry_msgs::PoseStamped& pose){
    // int **return_data = checkNeighbourhood(pose,0.5);
    // int i,j;
    // for(i=0;i<3;i++){
    //     for(j=0;j<3;j++){
    //         ROS_INFO("Point %d %d value = %d",i,j,return_data[i][j]);
    //     }
    // }
    // ROS_INFO("%f",pose.pose.position.x);
    float x_cordinate = round(getRoundedPoint(pose.pose.position.x));
    ROS_INFO("%f",x_cordinate);
    float y_cordinate = round(getRoundedPoint(pose.pose.position.y));
    ROS_INFO("%f",y_cordinate);
    float z_cordinate = round(getRoundedPoint(pose.pose.position.z));
    ROS_INFO("%f",z_cordinate);
    
    
        // ROS_INFO("occupied lower");
        // if(occupied_points.hasValue(x_cordinate,y_cordinate,0.025000)){
        //     ROS_INFO("occupied both");
        // }else{
        //     ROS_INFO("occupied lower");
        // }
        // if(occupied_points.hasValue(x_cordinate,y_cordinate,0.075000)){
        //     ROS_INFO("occupied third");
        // }else{
        //     ROS_INFO("not occupied third");
        // }
    int i = 0;
    x_cordinate = round(x_cordinate);
    y_cordinate = round(y_cordinate);
    for(i=0; i<10; i++){
        float z_cordinate = round(-0.025000+0.050000*i);
        ROS_INFO("%f %f %f",x_cordinate,y_cordinate,z_cordinate);
        if(occupied_points.hasValue(x_cordinate,y_cordinate,z_cordinate)){
            if(occupied_points.getValue(x_cordinate,y_cordinate,z_cordinate)==1){
                ROS_INFO("occupied_point %f %f %f",x_cordinate,y_cordinate,z_cordinate);
            }else{
                ROS_INFO("free_point %f %f %f",x_cordinate,y_cordinate,z_cordinate);
            }
        }else{
            ROS_INFO("undiscovered  %f %f %f",x_cordinate,y_cordinate,z_cordinate);
        }
    }

   
    // initializeFrstNode(x_cordinate,y_cordinate,z_cordinate,0.5);

    // printf("%b",occupied_points.hasValue(x_cordinate,y_cordinate,z_cordinate));

}

int NavigationPlanner::pathTraversalCost(struct Graph_Node *graph_node){
    // while(graph_node->predecessor != NULL){
    //     traversal_path_cost += graph_node->path_cost;
    //     graph_node = graph_node->predecessor;
    // }
    if(graph_node != NULL && graph_node->predecessor != NULL){
        return graph_node->predecessor->path_cost+1;
    }
    return 0;
}

struct Graph_Node *NavigationPlanner::getBreadthFirstSearchNodes(float x_cordinate, float y_cordinate, float z_cordinate, float box_dimension){
    if(node_queue.empty()){
        initializeFrstNode(x_cordinate,y_cordinate,z_cordinate,box_dimension);
    }
    
    if(!breadth_empty_queue.empty()){
        struct Graph_Node *best_node;
        int minimum_path_cost;
        while(!breadth_empty_queue.empty()){
            struct Graph_Node *temp_breadth_node = breadth_empty_queue.front();
            breadth_empty_queue.pop();
            if(best_node == NULL){
                best_node = temp_breadth_node;
                minimum_path_cost = pathTraversalCost(temp_breadth_node);// + calculateWeightToThePoint(temp_breadth_node,1.12215)*0.001;
            }else{
                int temp_path_cost = pathTraversalCost(temp_breadth_node);// + calculateWeightToThePoint(temp_breadth_node,1.12215)*0.001;
                printf("Minimum Node Selector Candidate : %f %f %f = %d \n", temp_breadth_node->x_cordinate, temp_breadth_node->y_cordinate, temp_breadth_node->z_cordinate, temp_path_cost);
                if(temp_path_cost<minimum_path_cost){
                    minimum_path_cost = temp_path_cost;
                    best_node = temp_breadth_node;
                }
            }
        }
        return best_node;        
    }else{
        int node_queue_size = node_queue.size();
        int i=0;
        for(i; i< node_queue_size; i++){
            struct Graph_Node *temp_node = node_queue.front();
            node_queue.pop();
            getAdjecentSquareCentroids(temp_node->x_cordinate,temp_node->y_cordinate,temp_node->z_cordinate,box_dimension,temp_node);
        }
        
        getBreadthFirstSearchNodes(0.0,0.0,0.0,box_dimension);
    }
}


void cloud_call_back(const PointCloud::ConstPtr& msg){
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

void NavigationPlanner::start(){
    // subscriber_node = node_handle_.subscribe(topic_, 1000, &NavigationPlanner::retrieveDataFromOctomap,this);
    subscriber_node = node_handle_.subscribe(topic_, 1000, cloud_call_back);
    // ros::spin();
}