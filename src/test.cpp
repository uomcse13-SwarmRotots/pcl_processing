#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <cstdlib>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/console/parse.h>
#include <pcl/filters/crop_box.h>

void processCuboid(float minX, float minY, float minZ, float maxX, float maxY, float maxZ, pcl::PointCloud<pcl::PointXYZ>* cloudIn){
    Eigen::Vector4f minPoint;
    minPoint[0]=minX;
    minPoint[1]=minY;
    minPoint[2]=minZ;

    Eigen::Vector4f maxPoint;
    minPoint[0]=maxX;
    minPoint[1]=maxY;
    minPoint[2]=maxZ;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::CropBox<PointT> cropFilter;
    cropFilter.setInputCloud (cloudIn);
    cropFilter.setMin(minPoint);
    cropFilter.setMax(maxPoint);

    cropFilter.filter (*cloudOut); 


}


int main(){
    return 0;
}
