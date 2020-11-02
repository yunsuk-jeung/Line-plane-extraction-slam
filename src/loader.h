#ifndef loader_h
#define loader_h

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <string>
#include <fstream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/integral_image_normal.h>
#include <vector>
#include <cmath>
#define _USE_MATH_DEFINES
#define ROW 64
#define COL 4500
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
    void remove_flat_region();
    void create_depth_image();
    void create_integral_image();
    void get_normal();


private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
//    std::vector< std::vector < spherical_point > > spherical_depth_image;
    spherical_point depth_image[64][4500];
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr vertical_cloud;
    pcl::PointCloud<pcl::Normal>::Ptr normals;
};

struct interval_point{
    int min_row;
    int max_row;
    int min_col;
    int max_col;
    interval_point(){
        min_row = 0;
        max_row=0;
        min_col=0;
        max_col=0;
    }
};


class image{
public:
    image(int row, int col);
    void integral_image(spherical_point (&depth_image)[64][4500], pcl::PointCloud<pcl::PointXYZRGB>::Ptr vertical_cloud);
    void set_boundary(int boundary_row, int boundary_col);

private:
    std::vector < std::vector < interval_point > > interval_image;
    std::vector < std::vector < double > > itg_x;
    std::vector < std::vector < double > > itg_y;
    std::vector < std::vector < double > > itg_z;
    std::vector < std::vector < double > > itg_xx;
    std::vector < std::vector < double > > itg_xy;
    std::vector < std::vector < double > > itg_xz;
    std::vector < std::vector < double > > itg_yy;
    std::vector < std::vector < double > > itg_yz;
    std::vector < std::vector < double > > itg_zz;
    std::vector < std::vector < int > > itg_num;
    int boundary_row;
    int boundary_col;

};
#endif /* loader_h */