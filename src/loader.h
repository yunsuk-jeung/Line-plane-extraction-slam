#ifndef loader_h
#define loader_h

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <string>
#include <fstream>
#include <pcl/visualization/cloud_viewer.h>
#include <vector>
#include <cmath>
#define _USE_MATH_DEFINES
struct spherical_point{
    float r;
    float theta;
    float pi;
    int index;
    spherical_point(){
        r=0;
        theta=0;
        pi =0;
        index =-1;
    }
};

class loader
{
public:
    loader();
    void txt2pcl(std::string fileName);
    void viewer();
//    void create_depth_image_2();
    void remove_flat_region();
    void create_depth_image();

private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
//    std::vector< std::vector < spherical_point > > spherical_depth_image;
    spherical_point depth_image[100][4500];
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr vertical_cloud;
};

#endif /* loader_h */