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
#include <time.h>
#include <omp.h>

#define _USE_MATH_DEFINES
#define ROW 64
#define COL 4500
#define DEPTH_THRESHOLD 0.4
#define neighbouring_radius 0.2
#define INTERVAL_CIRCULAR_LEVELS 2


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
        left_col = -1;
        up_row=-1;
        right_col=-1;
        bottom_row=-1;
    }
};

struct integral_point{
    double itg_x;
    double itg_y;
    double itg_z;
    double itg_xx;
    double itg_xy;
    double itg_xz;
    double itg_yy;
    double itg_yz;
    double itg_zz;
    int itg_num;
        integral_point(){
            itg_x=0;
            itg_y=0;
            itg_z=0;
            itg_xx=0;
            itg_xy=0;
            itg_xz=0;
            itg_yy=0;
            itg_yz=0;
            itg_zz=0;
            itg_num=0;
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
    std::vector < std::vector < integral_point > > integral_image;

    int boundary_row;
    int boundary_col;
    pcl::PointCloud<pcl::Normal>::Ptr normals;

};

#endif /* image_h */