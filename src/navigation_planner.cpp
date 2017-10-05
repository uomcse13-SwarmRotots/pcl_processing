#include "navigation_planner.h"



pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

void NavigationPlanner::planerCoefficientApproximation(pcl::PointCloud<pcl::PointXYZ>::Ptr& plane_cloud){
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
        return;
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
}


void NavigationPlanner::clusterObjects(pcl::PointCloud<pcl::PointXYZ>::Ptr& object_cloud){
    std::cerr << "cluster objects " << std::endl;
    if(object_cloud->size()>0){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
        // Create the filtering object: downsample the dataset using a leaf size of 1cm
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        vg.setInputCloud (object_cloud);
        vg.setLeafSize (0.01f, 0.01f, 0.01f);
        vg.filter (*cloud_filtered);

        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
        pcl::PCDWriter writer;
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100);
        seg.setDistanceThreshold (0.02);

        int i=0, nr_points = (int) cloud_filtered->points.size ();
        while (cloud_filtered->points.size () > 0.3 * nr_points) {
            // Segment the largest planar component from the remaining cloud
            seg.setInputCloud (cloud_filtered);
            seg.segment (*inliers, *coefficients);
            if (inliers->indices.size () == 0) {
                std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
                break;
            }

            // Extract the planar inliers from the input cloud
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud (cloud_filtered);
            extract.setIndices (inliers);
            extract.setNegative (false);

            // Get the points associated with the planar surface
            extract.filter (*cloud_plane);
            std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

            // Remove the planar inliers, extract the rest
            extract.setNegative (true);
            extract.filter (*cloud_f);
            *cloud_filtered = *cloud_f;
        }

    // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (cloud_filtered);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (0.2); // 2cm
        ec.setMinClusterSize (25);
        ec.setMaxClusterSize (50000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud_filtered);
        ec.extract (cluster_indices);

        int j = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
            cloud_cluster->width = cloud_cluster->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
            std::stringstream ss;
            ss << "cloud_cluster_" << j << ".pcd";
            writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
            planerCoefficientApproximation(cloud_cluster);
            j++;
        }

        if(j==0){
            std::cout << "No points found after filter" << std::endl;
        }
    }else{
        std::cout << "No obstacles found - null input cloud" << std::endl; 
    }
}



void  NavigationPlanner::groundNonGroundExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_cube){
    
    pcl::PointIndicesPtr ground (new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    std::cerr << "Cloud before filtering: " << std::endl;
    //std::cerr << *cloud << std::endl;

    pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
    pmf.setInputCloud (cloud_cube);
    pmf.setMaxWindowSize (20);
    pmf.setSlope (1.0f);
    pmf.setInitialDistance (0.1f);
    pmf.setMaxDistance (3.0f);
    pmf.extract (ground->indices);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_cube);
    extract.setIndices (ground);
    extract.filter (*cloud_filtered);

    // std::cerr << "Ground cloud after filtering: " << std::endl;
    //std::cerr << *cloud_filtered << std::endl;
    pcl::PCDWriter writer;
    if(cloud_filtered->size()>0){
        
        writer.write<pcl::PointXYZ> ("samp11-utm_ground.pcd", *cloud_filtered, false);
        std::cerr << "Ground cloud saved " << std::endl;
        //Extract non-ground returns
    }else{
        std::cerr << "No Ground found: " << std::endl;
    }
    extract.setNegative (true);
    extract.filter (*cloud_filtered);

    // std::cerr << "Object cloud after filtering: " << std::endl;
    //std::cerr << *cloud_filtered << std::endl;
    if(cloud_filtered->size()>0){
        std::cerr << "Object cloud saved: " << std::endl;
        writer.write<pcl::PointXYZ> ("samp11-utm_object.pcd", *cloud_filtered, false);
        clusterObjects(cloud_filtered);
    }else{
        std::cerr << "No objects found: " << std::endl;
    }
    
    std::cerr << "end" << std::endl;
}

void NavigationPlanner::segmentBoundingCube(const geometry_msgs::PoseStamped& pose){

    float x_cordinate = pose.pose.position.x;
    float y_cordinate = pose.pose.position.y;
    float z_cordinate = pose.pose.position.z;

    float resolution = 0.5;
    float x_min = x_cordinate - resolution;
    float x_max = x_cordinate + resolution;
    float y_min = y_cordinate - resolution;
    float y_max = y_cordinate + resolution;
    float z_min = z_cordinate - resolution;
    float z_max = z_cordinate + resolution;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (z_min, z_max);
    pass.filter (*cloud_filtered);

    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (y_min, y_max);
    pass.filter (*cloud_filtered);

    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (x_min, x_max);
    pass.filter (*cloud_filtered);

    if(cloud_filtered->size()>0){
        groundNonGroundExtraction(cloud_filtered);
    }else{
        std::cerr << "No Data Points Near By - Uninitialized: " << std::endl;
    }
    

}

void NavigationPlanner::cloudCallback(const PointCloud::ConstPtr& msg){
//   printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    cloud->width  = msg->width;
    cloud->height = msg->height;
    cloud->points.resize (cloud->width * cloud->height);

    int i=0;
    BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points){
        // printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
        cloud->points[i].x = pt.x;
        cloud->points[i].y = pt.y;
        cloud->points[i].z = pt.z;
        i++;
    }
    //groundNonGroundExtraction(cloud);
}


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

// int **NavigationPlanner::checkNeighbourhood(const geometry_msgs::PoseStamped& pose){
//     float x_cordinate = pose.pose.position.x;
//     float y_cordinate = pose.pose.position.y;
//     float z_cordinate = pose.pose.position.z;

//     float resolution = 0.5;
//     x_min = x_cordinate - resolution;
//     x_max = x_cordinate + resolution;
//     y_min = y_cordinate - resolution;
//     y_max = y_cordinate + resolution;
//     z_min = z_cordinate - resolution;
//     z_max = z_cordinate + resolution;

//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

//     pcl::PassThrough<pcl::PointXYZ> pass;
//     pass.setInputCloud (cloud);
//     pass.setFilterFieldName ("z");
//     pass.setFilterLimits (z_min, z_max);in
//     pass.filter (*cloud_filtered);

//     pass.setInputCloud (cloud_filtered);
//     pass.setFilterFieldName ("y");
//     pass.setFilterLimits (y_min, y_max);
//     pass.filter (*cloud_filtered);

//     pass.setInputCloud (cloud_filtered);
//     pass.setFilterFieldName ("x");
//     pass.setFilterLimits (x_min, x_max);
//     pass.filter (*cloud_filtered);

//     int ok_count;
//     int occupied_count;


//     for(pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud_filtered->iterator; it!= cloud_filtered->end(); it++){
//         if(it->z > z_cordinate + 0.05){
//             occupied_count++;
//         }else{
//             ok_count++;
//         }   
//         //cout << it->x << ", " << it->y << ", " << it->z << endl;
//     } 
//     cout << ok_count << ", " << occupied_count << endl;
// }

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


void NavigationPlanner::start(){
    subscriber_node = node_handle_.subscribe(topic_, 1000, &NavigationPlanner::cloudCallback,this);
    // subscriber_node = node_handle_.subscribe(topic_, 1000, cloud_call_back);
    // ros::spin();
}