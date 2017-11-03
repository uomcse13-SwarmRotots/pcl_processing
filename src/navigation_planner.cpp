#include "navigation_planner.h"
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/convex_hull.h>

#include <visualization_msgs/Marker.h>
#include <cmath>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
struct Graph_Node *current_node;

pcl::PointCloud<pcl::PointXYZ>::Ptr calculateConvexHull(vector<pcl::PointXYZ> point_vector,int point_type){
    pcl::CropHull<pcl::PointXYZ> cropHullFilter;
    pcl::PointCloud<pcl::PointXYZ>::Ptr hullCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr hullPoints(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::Vertices> hullPolygons;
    vector<pcl::PointXYZ>::iterator it;
    for(it = point_vector.begin(); it != point_vector.end(); it++){
        hullCloud->push_back(*it);
        //printf("count\n");
    }

    // setup hull filter
    pcl::ConvexHull<pcl::PointXYZ> cHull;
    cHull.setInputCloud(hullCloud);
    cHull.reconstruct(*hullPoints, hullPolygons);

    cropHullFilter.setHullIndices(hullPolygons);
    cropHullFilter.setHullCloud(hullPoints);
    cropHullFilter.setDim(2); // if you uncomment this, it will work
    cropHullFilter.setCropOutside(true); // this will remove points inside the hull

  //filter points
    cropHullFilter.setInputCloud(cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cropHullFilter.filter(*filtered_cloud);

    std::string file_name1="filtered";
    file_name1 = file_name1 + boost::lexical_cast<std::string>(point_type);
    file_name1 = file_name1+".pcd";

    std::string file_name2="hullPoints";
    file_name2 = file_name2 + boost::lexical_cast<std::string>(point_type);
    file_name2 = file_name2+".pcd";

    std::string file_name3="cloud";
    file_name3 = file_name3 + boost::lexical_cast<std::string>(point_type);
    file_name3 = file_name3+".pcd";

    if(filtered_cloud->size()>0){
        pcl::io::savePCDFile(file_name1, *filtered_cloud, true);
    }
    if(hullPoints->size()>0){
        pcl::io::savePCDFile(file_name2, *hullPoints,true);
    }
    if(cloud->size()>0){
        pcl::io::savePCDFile(file_name3, *cloud,true);
    }
    
    return filtered_cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr getConvexHull(float x_cordinate, float y_cordinate, float z_cordinate, int point_type, float box_dimension){
    vector<pcl::PointXYZ> point_vector; 
    if(point_type==2){
        pcl::PointXYZ point1;
        point1.x=x_cordinate - 2*box_dimension;
        point1.y=y_cordinate;
        point1.z=z_cordinate;
        point_vector.push_back(point1);
        
        pcl::PointXYZ point2;
        point2.x=x_cordinate - box_dimension;
        point2.y=y_cordinate - box_dimension;
        point2.z=z_cordinate;
        point_vector.push_back(point2);

        pcl::PointXYZ point3;
        point3.x=x_cordinate + box_dimension;
        point3.y=y_cordinate - box_dimension;
        point3.z=z_cordinate;
        point_vector.push_back(point3);

        pcl::PointXYZ point4;
        point4.x=x_cordinate + box_dimension;
        point4.y=y_cordinate + box_dimension;
        point4.z=z_cordinate;
        point_vector.push_back(point4);

        pcl::PointXYZ point5;
        point5.x=x_cordinate;
        point5.y=y_cordinate + 2*box_dimension;
        point5.z=z_cordinate;
        point_vector.push_back(point5);

        pcl::PointXYZ point6;
        point6.x=x_cordinate - 2*box_dimension;;
        point6.y=y_cordinate + 2*box_dimension;;
        point6.z=z_cordinate;
        point_vector.push_back(point6);
    }else if(point_type==1){
        pcl::PointXYZ point1;
        point1.x=x_cordinate - box_dimension;
        point1.y=y_cordinate - box_dimension;
        point1.z=z_cordinate;
        point_vector.push_back(point1);

        pcl::PointXYZ point2;
        point2.x=x_cordinate + box_dimension;
        point2.y=y_cordinate - box_dimension;
        point2.z=z_cordinate;
        point_vector.push_back(point2);

        pcl::PointXYZ point3;
        point3.x=x_cordinate + box_dimension;
        point3.y=y_cordinate + 2*box_dimension;
        point3.z=z_cordinate;
        point_vector.push_back(point3);

        pcl::PointXYZ point4;
        point4.x=x_cordinate - box_dimension;
        point4.y=y_cordinate + 2*box_dimension;
        point4.z=z_cordinate;
        point_vector.push_back(point4);
    }else if(point_type==3){
        pcl::PointXYZ point1;
        point1.x=x_cordinate + box_dimension;
        point1.y=y_cordinate + box_dimension;
        point1.z=z_cordinate;
        point_vector.push_back(point1);

        pcl::PointXYZ point2;
        point2.x=x_cordinate - 2*box_dimension;
        point2.y=y_cordinate + box_dimension;
        point2.z=z_cordinate;
        point_vector.push_back(point2);

        pcl::PointXYZ point3;
        point3.x=x_cordinate - 2*box_dimension;
        point3.y=y_cordinate - box_dimension;
        point3.z=z_cordinate;
        point_vector.push_back(point3);

        pcl::PointXYZ point4;
        point4.x=x_cordinate + box_dimension;
        point4.y=y_cordinate - box_dimension;
        point4.z=z_cordinate;
        point_vector.push_back(point4);
    }else if(point_type==4){
        pcl::PointXYZ point1;
        point1.x=x_cordinate + box_dimension;
        point1.y=y_cordinate + box_dimension;
        point1.z=z_cordinate;
        point_vector.push_back(point1);
        
        pcl::PointXYZ point2;
        point2.x=x_cordinate - box_dimension;
        point2.y=y_cordinate + box_dimension;
        point2.z=z_cordinate;
        point_vector.push_back(point2);

        pcl::PointXYZ point3;
        point3.x=x_cordinate - 2*box_dimension;
        point3.y=y_cordinate;
        point3.z=z_cordinate;
        point_vector.push_back(point3);

        pcl::PointXYZ point4;
        point4.x=x_cordinate - 2*box_dimension;
        point4.y=y_cordinate - 2*box_dimension;
        point4.z=z_cordinate;
        point_vector.push_back(point4);

        pcl::PointXYZ point5;
        point5.x=x_cordinate;
        point5.y=y_cordinate - 2*box_dimension;
        point5.z=z_cordinate;
        point_vector.push_back(point5);

        pcl::PointXYZ point6;
        point6.x=x_cordinate + box_dimension;;
        point6.y=y_cordinate - box_dimension;;
        point6.z=z_cordinate;
        point_vector.push_back(point6);
    }else if(point_type==5){
        pcl::PointXYZ point1;
        point1.x=x_cordinate - box_dimension;
        point1.y=y_cordinate + box_dimension;
        point1.z=z_cordinate;
        point_vector.push_back(point1);

        pcl::PointXYZ point2;
        point2.x=x_cordinate - box_dimension;
        point2.y=y_cordinate - 2*box_dimension;
        point2.z=z_cordinate;
        point_vector.push_back(point2);

        pcl::PointXYZ point3;
        point3.x=x_cordinate + box_dimension;
        point3.y=y_cordinate - 2*box_dimension;
        point3.z=z_cordinate;
        point_vector.push_back(point3);

        pcl::PointXYZ point4;
        point4.x=x_cordinate + box_dimension;
        point4.y=y_cordinate + box_dimension;
        point4.z=z_cordinate;
        point_vector.push_back(point4);
    }else if(point_type==6){
        pcl::PointXYZ point1;
        point1.x=x_cordinate + box_dimension;
        point1.y=y_cordinate + box_dimension;
        point1.z=z_cordinate;
        point_vector.push_back(point1);
        
        pcl::PointXYZ point2;
        point2.x=x_cordinate - box_dimension;
        point2.y=y_cordinate + box_dimension;
        point2.z=z_cordinate;
        point_vector.push_back(point2);

        pcl::PointXYZ point3;
        point3.x=x_cordinate - box_dimension;
        point3.y=y_cordinate - box_dimension;
        point3.z=z_cordinate;
        point_vector.push_back(point3);

        pcl::PointXYZ point4;
        point4.x=x_cordinate;
        point4.y=y_cordinate - 2*box_dimension;
        point4.z=z_cordinate;
        point_vector.push_back(point4);

        pcl::PointXYZ point5;
        point5.x=x_cordinate + 2*box_dimension;
        point5.y=y_cordinate - 2*box_dimension;
        point5.z=z_cordinate;
        point_vector.push_back(point5);

        pcl::PointXYZ point6;
        point6.x=x_cordinate + 2*box_dimension;
        point6.y=y_cordinate;
        point6.z=z_cordinate;
        point_vector.push_back(point6);
    }else if(point_type==7){
        pcl::PointXYZ point1;
        point1.x=x_cordinate - box_dimension;
        point1.y=y_cordinate + box_dimension;
        point1.z=z_cordinate;
        point_vector.push_back(point1);

        pcl::PointXYZ point2;
        point2.x=x_cordinate - box_dimension;
        point2.y=y_cordinate - box_dimension;
        point2.z=z_cordinate;
        point_vector.push_back(point2);

        pcl::PointXYZ point3;
        point3.x=x_cordinate + 2*box_dimension;
        point3.y=y_cordinate - box_dimension;
        point3.z=z_cordinate;
        point_vector.push_back(point3);

        pcl::PointXYZ point4;
        point4.x=x_cordinate + 2*box_dimension;
        point4.y=y_cordinate + box_dimension;
        point4.z=z_cordinate;
        point_vector.push_back(point4);
    }else if(point_type==8){
        pcl::PointXYZ point1;
        point1.x=x_cordinate + 2*box_dimension;
        point1.y=y_cordinate;
        point1.z=z_cordinate;
        point_vector.push_back(point1);
        
        pcl::PointXYZ point2;
        point2.x=x_cordinate + 2*box_dimension;
        point2.y=y_cordinate + 2*box_dimension;
        point2.z=z_cordinate;
        point_vector.push_back(point2);

        pcl::PointXYZ point3;
        point3.x=x_cordinate;
        point3.y=y_cordinate + 2*box_dimension;
        point3.z=z_cordinate;
        point_vector.push_back(point3);

        pcl::PointXYZ point4;
        point4.x=x_cordinate - box_dimension;
        point4.y=y_cordinate + box_dimension;
        point4.z=z_cordinate;
        point_vector.push_back(point4);

        pcl::PointXYZ point5;
        point5.x=x_cordinate - box_dimension;
        point5.y=y_cordinate - box_dimension;
        point5.z=z_cordinate;
        point_vector.push_back(point5);

        pcl::PointXYZ point6;
        point6.x=x_cordinate - box_dimension;
        point6.y=y_cordinate + box_dimension;
        point6.z=z_cordinate;
        point_vector.push_back(point6);
    }

    return calculateConvexHull(point_vector,point_type);

}


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



int  NavigationPlanner::groundNonGroundExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_cube){
    
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
    if(cloud_filtered->size()>100){
        writer.write<pcl::PointXYZ> ("samp11-utm_ground.pcd", *cloud_filtered, false);
        // std::cerr << "Ground cloud saved " << std::endl;
        //Extract non-ground returns
    }else{
        // std::cerr << "No Ground found: " << std::endl;
        return -1;
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
        return 1;
    }
    // std::cerr << "end" << std::endl;
    return 0;
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
        int state = groundNonGroundExtraction(cloud_filtered);
        if(state == 1){
            std::cerr << "Can ENTER........" << std::endl;
            return 1;
        }else if(state == 0){
            std::cerr << "Cannot ENTER......." << std::endl;
            return 0;
        }else{
            std::cerr << "No Data Points Near By - Uninitialized:" << std::endl;
            return -1;
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr convex_cloud;

    int result;
    // int result = segmentBoundingCube(x_cordinate,y_cordinate,z_cordinate);
    
    // if(result == -1){
    //     return NULL;
    //     //uninitalized node found
    // }else if(result == 1){
    //     current_node->x_cordinate = x_cordinate;
    //     current_node->y_cordinate = y_cordinate;
    //     current_node->z_cordinate = z_cordinate;
    //     current_node->predecessor = NULL;
    // }


    float front_x = x_cordinate + box_dimension;
    float front_y = y_cordinate;
    convex_cloud = getConvexHull(front_x,front_y,z_cordinate,1,0.5);
    result = groundNonGroundExtraction(convex_cloud);
    if(result == -1){
        ROS_INFO("Return on 1");
        return current_node;
        //uninitalized node found
    }else if(result == 1){
        struct Graph_Node *temp_node = new Graph_Node;
        temp_node->x_cordinate = front_x;
        temp_node->y_cordinate = front_y;
        temp_node->z_cordinate = z_cordinate;
        temp_node->predecessor = current_node;
        temp_node->path_cost = current_node->path_cost+1;
        if(found_nodes.hasValue(front_x,front_y,z_cordinate)){
            if(found_nodes.getValue(front_x,front_y,z_cordinate)->path_cost>temp_node->path_cost){
                node_queue.push(temp_node);
            }
        }else{
            found_nodes.setValue(front_x,front_y,z_cordinate,temp_node);
            node_queue.push(temp_node);
        }
    }

    float front_left_x = x_cordinate + box_dimension;
    float front_left_y = y_cordinate + box_dimension;
    convex_cloud = getConvexHull(front_left_x,front_left_y,z_cordinate,2,0.5);
    result = groundNonGroundExtraction(convex_cloud);
    if(result == -1){
        ROS_INFO("Return on 2");
        return current_node;
        //uninitalized node found
    }else if(result == 1){
        struct Graph_Node *temp_node = new Graph_Node;
        temp_node->x_cordinate = front_left_x;
        temp_node->y_cordinate = front_left_y;
        temp_node->z_cordinate = z_cordinate;
        temp_node->predecessor = current_node;
        temp_node->path_cost = current_node->path_cost+1;
        if(found_nodes.hasValue(front_left_x,front_left_y,z_cordinate)){
            if(found_nodes.getValue(front_left_x,front_left_y,z_cordinate)->path_cost>temp_node->path_cost){
                node_queue.push(temp_node);
            }
        }else{
            found_nodes.setValue(front_left_x,front_left_y,z_cordinate,temp_node);
            node_queue.push(temp_node);
        }
    }      

    float left_x = x_cordinate;
    float left_y = y_cordinate + box_dimension;
    convex_cloud = getConvexHull(left_x,left_y,z_cordinate,3,0.5);
    result = groundNonGroundExtraction(convex_cloud);
    if(result == -1){
        ROS_INFO("Return on 3");
        return current_node;
        //uninitalized node found
    }else if(result == 1){
        struct Graph_Node *temp_node = new Graph_Node;
        temp_node->x_cordinate = left_x;
        temp_node->y_cordinate = left_y;
        temp_node->z_cordinate = z_cordinate;
        temp_node->predecessor = current_node;
        temp_node->path_cost = current_node->path_cost+1;
        if(found_nodes.hasValue(left_x,left_y,z_cordinate)){
            if(found_nodes.getValue(left_x,left_y,z_cordinate)->path_cost>temp_node->path_cost){
                node_queue.push(temp_node);
            }
        }else{
            found_nodes.setValue(left_x,left_y,z_cordinate,temp_node);
            node_queue.push(temp_node);
        }
    } 



    float back_left_x = x_cordinate - box_dimension;
    float back_left_y = y_cordinate + box_dimension;
    convex_cloud = getConvexHull(back_left_x,back_left_y,z_cordinate,4,0.5);
    result = groundNonGroundExtraction(convex_cloud);
    if(result == -1){
        ROS_INFO("Return on 4");
        return current_node;
        //uninitalized node found
    }else if(result == 1){
        struct Graph_Node *temp_node = new Graph_Node;
        temp_node->x_cordinate = back_left_x;
        temp_node->y_cordinate = back_left_y;
        temp_node->z_cordinate = z_cordinate;
        temp_node->predecessor = current_node;
        temp_node->path_cost = current_node->path_cost+1;
        if(found_nodes.hasValue(back_left_x,back_left_y,z_cordinate)){
            if(found_nodes.getValue(back_left_x,back_left_y,z_cordinate)->path_cost>temp_node->path_cost){
                node_queue.push(temp_node);
            }
        }else{
            found_nodes.setValue(back_left_x,back_left_y,z_cordinate,temp_node);
            node_queue.push(temp_node);
        }
    } 


    float back_x = x_cordinate - box_dimension;
    float back_y = y_cordinate;
    convex_cloud = getConvexHull(back_x,back_y,z_cordinate,5,0.5);
    result = groundNonGroundExtraction(convex_cloud);
    if(result == -1){
        ROS_INFO("Return on 5");
        return current_node;
        //uninitalized node found
    }else if(result == 1){
        struct Graph_Node *temp_node = new Graph_Node;
        temp_node->x_cordinate = back_x;
        temp_node->y_cordinate = back_y;
        temp_node->z_cordinate = z_cordinate;
        temp_node->predecessor = current_node;
        temp_node->path_cost = current_node->path_cost+1;
        if(found_nodes.hasValue(back_x,back_y,z_cordinate)){
            if(found_nodes.getValue(back_x,back_y,z_cordinate)->path_cost>temp_node->path_cost){
                node_queue.push(temp_node);
            }
        }else{
            found_nodes.setValue(back_x,back_y,z_cordinate,temp_node);
            node_queue.push(temp_node);
        }
    } 


    float back_right_x = x_cordinate - box_dimension;
    float back_right_y = y_cordinate - box_dimension;
    convex_cloud = getConvexHull(back_right_x,back_right_y,z_cordinate,6,0.5);
    result = groundNonGroundExtraction(convex_cloud);
    if(result == -1){
        ROS_INFO("Return on 6");
        return current_node;
        //uninitalized node found
    }else if(result == 1){
        struct Graph_Node *temp_node = new Graph_Node;
        temp_node->x_cordinate = back_right_x;
        temp_node->y_cordinate = back_right_y;
        temp_node->z_cordinate = z_cordinate;
        temp_node->predecessor = current_node;
        temp_node->path_cost = current_node->path_cost+1;
        if(found_nodes.hasValue(back_right_x,back_right_y,z_cordinate)){
            if(found_nodes.getValue(back_right_x,back_right_y,z_cordinate)->path_cost>temp_node->path_cost){
                node_queue.push(temp_node);
            }
        }else{
            found_nodes.setValue(back_right_x,back_right_y,z_cordinate,temp_node);
            node_queue.push(temp_node);
        }
    } 


    float right_x = x_cordinate;
    float right_y = y_cordinate - box_dimension;
    convex_cloud = getConvexHull(right_x,right_y,z_cordinate,7,0.5);
    result = groundNonGroundExtraction(convex_cloud);
    if(result == -1){
        ROS_INFO("Return on 7");
        return current_node;
        //uninitalized node found
    }else if(result == 1){
        struct Graph_Node *temp_node = new Graph_Node;
        temp_node->x_cordinate = right_x;
        temp_node->y_cordinate = right_y;
        temp_node->z_cordinate = z_cordinate;
        temp_node->predecessor = current_node;
        temp_node->path_cost = current_node->path_cost+1;
        if(found_nodes.hasValue(right_x,right_y,z_cordinate)){
            if(found_nodes.getValue(right_x,right_y,z_cordinate)->path_cost>temp_node->path_cost){
                node_queue.push(temp_node);
            }
        }else{
            found_nodes.setValue(right_x,right_y,z_cordinate,temp_node);
            node_queue.push(temp_node);
        }
    } 



    float front_right_x = x_cordinate + box_dimension;
    float front_right_y = y_cordinate - box_dimension;
    convex_cloud = getConvexHull(front_left_x,front_right_y,z_cordinate,8,0.5);
    result = groundNonGroundExtraction(convex_cloud);
    if(result == -1){
        ROS_INFO("Return on 8");
        return current_node;
        //uninitalized node found
    }else if(result == 1){
        struct Graph_Node *temp_node = new Graph_Node;
        temp_node->x_cordinate = front_right_x;
        temp_node->y_cordinate = front_right_y;
        temp_node->z_cordinate = z_cordinate;
        temp_node->predecessor = current_node;
        temp_node->path_cost = current_node->path_cost+1;
        if(found_nodes.hasValue(front_left_x,front_right_y,z_cordinate)){
            if(found_nodes.getValue(front_left_x,front_right_y,z_cordinate)->path_cost>temp_node->path_cost){
                node_queue.push(temp_node);
            }
        }else{
            found_nodes.setValue(front_left_x,front_right_y,z_cordinate,temp_node);
            node_queue.push(temp_node);
        }
    } 

    if(node_queue.empty()){
        ROS_INFO("No Nodes to Traverse");
        return current_node;
    }else{
        struct Graph_Node *next_node = node_queue.front();
        node_queue.pop();
        ROS_INFO("Started Braeadth Search");
        current_node=next_node;
        return breadthFirstSearch(next_node->x_cordinate,next_node->y_cordinate,next_node->z_cordinate);
    }    
} 

void publishPath(struct Graph_Node *node){
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    struct Graph_Node *temp_node = node;

    while (ros::ok()){
        visualization_msgs::Marker points, line_strip, line_list;
        points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/odom";
        points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
        points.ns = line_strip.ns = line_list.ns = "points_and_lines";
        points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

        points.id = 0;
        line_strip.id = 1;
        line_list.id = 2;

        points.type = visualization_msgs::Marker::POINTS;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_list.type = visualization_msgs::Marker::LINE_LIST;

        // POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0.2;
        points.scale.y = 0.2;

        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        line_strip.scale.x = 0.1;
        line_list.scale.x = 0.1;

        // Points are green
        points.color.g = 1.0f;
        points.color.a = 1.0;

        // Line strip is blue
        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;

        // Line list is red
        line_list.color.r = 1.0;
        line_list.color.a = 1.0;

        //Create the vertices for the points and lines
        // for (uint32_t i = 0; i < 100; ++i){
        //   float y = 5 * sin( i / 100.0f * 2 * M_PI);
        //   float z = 5 * cos( i / 100.0f * 2 * M_PI);

        //   geometry_msgs::Point p;
        //   p.x = (int32_t)i - 50;
        //   p.y = y;
        //   p.z = z;

        //   points.points.push_back(p);
        //   line_strip.points.push_back(p);

        //   // The line list needs two points for each line
        //   line_list.points.push_back(p);
        //   p.z += 1.0;
        //   line_list.points.push_back(p);
        // }
        struct Graph_Node *temp_node1 = temp_node;

        while(temp_node1!=NULL){
            ROS_INFO("X %f , Y %f , Z %f",node->x_cordinate, node->y_cordinate, node->z_cordinate);
            geometry_msgs::Point p;
            p.x = temp_node1->x_cordinate;
            p.y = temp_node1->y_cordinate;
            p.z = temp_node1->z_cordinate;
            temp_node1 = temp_node1->predecessor;
            points.points.push_back(p);
            line_strip.points.push_back(p);
            line_list.points.push_back(p);
            p.z += 1.0;
            line_list.points.push_back(p);
        }

        marker_pub.publish(points);
        marker_pub.publish(line_strip);
        marker_pub.publish(line_list);
        current_node=NULL;
        ROS_INFO("END");

  }
}




void NavigationPlanner::startTraversal(const geometry_msgs::PoseStamped& pose){
    float x_cordinate = pose.pose.position.x;
    float y_cordinate = pose.pose.position.y;
    float z_cordinate = pose.pose.position.z;
    
    if(found_nodes.hasValue(x_cordinate,y_cordinate,z_cordinate)){
        current_node = found_nodes.getValue(x_cordinate,y_cordinate,z_cordinate);
        current_node->path_cost = 0;
        found_nodes.setValue(x_cordinate,y_cordinate,z_cordinate,current_node);
    }else{
        current_node =  new Graph_Node;
        current_node->x_cordinate = x_cordinate;
        current_node->y_cordinate = y_cordinate;
        current_node->z_cordinate = z_cordinate;
        current_node->path_cost = 0;
        current_node->predecessor = NULL;
        found_nodes.setValue(x_cordinate,y_cordinate,z_cordinate,current_node);
    }
    
    ROS_INFO("Start Node X %f , Y %f , Z %f",x_cordinate, y_cordinate, z_cordinate);
    // getConvexHull(x_cordinate,y_cordinate,z_cordinate,1,0.5);
    // getConvexHull(x_cordinate,y_cordinate,z_cordinate,2,0.5);
    // getConvexHull(x_cordinate,y_cordinate,z_cordinate,3,0.5);
    // getConvexHull(x_cordinate,y_cordinate,z_cordinate,4,0.5);
    // getConvexHull(x_cordinate,y_cordinate,z_cordinate,5,0.5);
    // getConvexHull(x_cordinate,y_cordinate,z_cordinate,6,0.5);
    // getConvexHull(x_cordinate,y_cordinate,z_cordinate,7,0.5);
    // getConvexHull(x_cordinate,y_cordinate,z_cordinate,8,0.5);
    struct Graph_Node *node = breadthFirstSearch(x_cordinate,y_cordinate,z_cordinate);
    ROS_INFO("FOUND NULL TRAVERSE");
    publishPath(node);
}


void NavigationPlanner::start(){
    subscriber_node = node_handle_.subscribe(topic_, 1000, &NavigationPlanner::cloudCallback,this);
    // subscriber_node = node_handle_.subscribe(topic_, 1000, cloud_call_back);
    // ros::spin();
}

