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



bool  NavigationPlanner::groundNonGroundExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_cube){
    
    pcl::PointIndicesPtr ground (new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    // std::cerr << "Cloud before filtering: " << std::endl;
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
        // std::cerr << "Ground cloud saved " << std::endl;
        //Extract non-ground returns
    }else{
        // std::cerr << "No Ground found: " << std::endl;
        return false;
    }
    extract.setNegative (true);
    extract.filter (*cloud_filtered);

    // std::cerr << "Object cloud after filtering: " << std::endl;
    //std::cerr << *cloud_filtered << std::endl;
    if(cloud_filtered->size()>0){
        // std::cerr << "Object cloud saved: " << std::endl;
        writer.write<pcl::PointXYZ> ("samp11-utm_object.pcd", *cloud_filtered, false);
        planerCoefficientApproximation(cloud_filtered);
    }else{
        // std::cerr << "No objects found: " << std::endl;
        return true;
    }
    // std::cerr << "end" << std::endl;
    return false;
}

int NavigationPlanner::segmentBoundingCube(float x_cordinate, float y_cordinate, float z_cordinate){

    // float x_cordinate = pose.pose.position.x;
    // float y_cordinate = pose.pose.position.y;
    // float z_cordinate = pose.pose.position.z;

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
        if(groundNonGroundExtraction(cloud_filtered)){
            std::cerr << "Can ENTER........" << std::endl;
            return 1;
        }else{
            std::cerr << "Cannot ENTER......." << std::endl;
            return 0;
        }
    }else{
        std::cerr << "No Data Points Near By - Uninitialized: " << std::endl;
        return -1;
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


struct Graph_Node* NavigationPlanner::breadthFirstSearch(float x_cordinate, float y_cordinate, float z_cordinate){

    float box_dimension = 0.5; // half of robot length

    int result = segmentBoundingCube(x_cordinate,y_cordinate,z_cordinate);
    struct Graph_Node *current_node =  new Graph_Node;;
    if(result == -1){
        return NULL;
        //uninitalized node found
    }else if(result == 1){
        current_node->x_cordinate = x_cordinate;
        current_node->y_cordinate = y_cordinate;
        current_node->z_cordinate = z_cordinate;
        current_node->predecessor = NULL;
    }


    float front_x = x_cordinate + box_dimension;
    float front_y = y_cordinate;
    result = segmentBoundingCube(front_x,front_y,z_cordinate);
    if(result == -1){
        return current_node;
        //uninitalized node found
    }else if(result == 1){
        struct Graph_Node *temp_node = new Graph_Node;
        temp_node->x_cordinate = front_x;
        temp_node->y_cordinate = front_y;
        temp_node->z_cordinate = z_cordinate;
        temp_node->predecessor = current_node;
        node_queue.push(temp_node);
    }      

 

    float front_left_x = x_cordinate + box_dimension;
    float front_left_y = y_cordinate + box_dimension;
    result = segmentBoundingCube(front_left_x,front_left_y,z_cordinate);
    if(result == -1){
        return current_node;
        //uninitalized node found
    }else if(result == 1){
        struct Graph_Node *temp_node = new Graph_Node;
        temp_node->x_cordinate = front_left_x;
        temp_node->y_cordinate = front_left_y;
        temp_node->z_cordinate = z_cordinate;
        temp_node->predecessor = current_node;
        node_queue.push(temp_node);
    }      

    float left_x = x_cordinate;
    float left_y = y_cordinate + box_dimension;
    result = segmentBoundingCube(left_x,left_y,z_cordinate);
    if(result == -1){
        return current_node;
        //uninitalized node found
    }else if(result == 1){
        struct Graph_Node *temp_node = new Graph_Node;
        temp_node->x_cordinate = left_x;
        temp_node->y_cordinate = left_y;
        temp_node->z_cordinate = z_cordinate;
        temp_node->predecessor = current_node;
        node_queue.push(temp_node);
    } 



    float back_left_x = x_cordinate - box_dimension;
    float back_left_y = y_cordinate + box_dimension;
    result = segmentBoundingCube(back_left_x,back_left_y,z_cordinate);
    if(result == -1){
        return current_node;
        //uninitalized node found
    }else if(result == 1){
        struct Graph_Node *temp_node = new Graph_Node;
        temp_node->x_cordinate = back_left_x;
        temp_node->y_cordinate = back_left_y;
        temp_node->z_cordinate = z_cordinate;
        temp_node->predecessor = current_node;
        node_queue.push(temp_node);
    } 


    float back_x = x_cordinate - box_dimension;
    float back_y = y_cordinate;
    result = segmentBoundingCube(back_x,back_y,z_cordinate);
    if(result == -1){
        return current_node;
        //uninitalized node found
    }else if(result == 1){
        struct Graph_Node *temp_node = new Graph_Node;
        temp_node->x_cordinate = back_x;
        temp_node->y_cordinate = back_y;
        temp_node->z_cordinate = z_cordinate;
        temp_node->predecessor = current_node;
        node_queue.push(temp_node);
    } 


    float back_right_x = x_cordinate - box_dimension;
    float back_right_y = y_cordinate - box_dimension;
    result = segmentBoundingCube(back_right_x,back_right_y,z_cordinate);
    if(result == -1){
        return current_node;
        //uninitalized node found
    }else if(result == 1){
        struct Graph_Node *temp_node = new Graph_Node;
        temp_node->x_cordinate = back_right_x;
        temp_node->y_cordinate = back_right_y;
        temp_node->z_cordinate = z_cordinate;
        temp_node->predecessor = current_node;
        node_queue.push(temp_node);
    } 


    float right_x = x_cordinate;
    float right_y = y_cordinate - box_dimension;
    result = segmentBoundingCube(right_x,right_y,z_cordinate);
    if(result == -1){
        return current_node;
        //uninitalized node found
    }else if(result == 1){
        struct Graph_Node *temp_node = new Graph_Node;
        temp_node->x_cordinate = right_x;
        temp_node->y_cordinate = right_y;
        temp_node->z_cordinate = z_cordinate;
        temp_node->predecessor = current_node;
        node_queue.push(temp_node);
    } 



    float front_right_x = x_cordinate + box_dimension;
    float front_right_y = y_cordinate - box_dimension;
    result = segmentBoundingCube(front_right_x,front_right_y,z_cordinate);
    if(result == -1){
        return current_node;
        //uninitalized node found
    }else if(result == 1){
        struct Graph_Node *temp_node = new Graph_Node;
        temp_node->x_cordinate = front_right_x;
        temp_node->y_cordinate = front_right_y;
        temp_node->z_cordinate = z_cordinate;
        temp_node->predecessor = current_node;
        node_queue.push(temp_node);
    } 

    if(node_queue.empty()){
        printf("No Nodes to Traverse");
        return current_node;
    }else{
        struct Graph_Node *next_node = node_queue.front();
        node_queue.pop();
        breadthFirstSearch(next_node->x_cordinate,next_node->y_cordinate,next_node->z_cordinate);
    }    
} 

void NavigationPlanner::startTraversal(const geometry_msgs::PoseStamped& pose){
    float x_cordinate = pose.pose.position.x;
    float y_cordinate = pose.pose.position.y;
    float z_cordinate = pose.pose.position.z;
    breadthFirstSearch(x_cordinate,y_cordinate,z_cordinate);
}


void NavigationPlanner::start(){
    subscriber_node = node_handle_.subscribe(topic_, 1000, &NavigationPlanner::cloudCallback,this);
    // subscriber_node = node_handle_.subscribe(topic_, 1000, cloud_call_back);
    // ros::spin();
}