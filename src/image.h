#ifndef image_h
#define image_h

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <string>
#include <fstream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/integral_image_normal.h>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#define _USE_MATH_DEFINES
#define ROW 64
#define COL 4500
#define DEPTH_THRESHOLD 0.6

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

struct interval_point{

    int left_col;
    int up_row;
    int right_col;
    int bottom_row;
    interval_point(){
        left_col = 0;
        up_row=0;
        right_col=0;
        bottom_row=0;
    }
};


class image{
public:
    image(int row, int col);
    void create_integral_image(const spherical_point (&depth_image)[ROW][COL], pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_vertical_cloud);
    void set_boundary(int row_boundary, int col_boundary);
    void create_interval_image(const spherical_point (&depth_image)[ROW][COL]);
    pcl::PointCloud<pcl::Normal>::Ptr get_normal(spherical_point (&depth_image)[ROW][COL], pcl::PointCloud<pcl::PointXYZRGB>::Ptr vertical_cloud);
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
    pcl::PointCloud<pcl::Normal>::Ptr normals;

};

#endif /* image_h */