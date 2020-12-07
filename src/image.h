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
#include <algorithm>
#define _USE_MATH_DEFINES
#define ROW 16
#define COL 900
#define CLOSE_FLAT_REGION_RANGE 0.4
#define CLOSE_VERTICAL_THRESHOLD 3
#define FAR_FLAT_REGION_RANGE 0.5
#define FAR_VERTICAL_THRESHOLD 3
#define NORMAL_DEPTH_THRESHOLD 0.3
#define neighbouring_radius 0.3
#define INTERVAL_CIRCULAR_LEVELS 3
#define LINE_EIGENVALUE_THRESHOLD 0.02
#define PLANE_EIGENVALUE_THRESHOLD 0.03
#define CLUSTER_NEIGHBOR_ROW 4
#define CLUSTER_NEIGHBOR_COL 25
#define CLUSTER_NEIGHBOR_ROW_VALID 8
#define CLUSTER_NEIGHBOR_COL_VALID 100
#define CLUSTER_NEIGHBOR_DISTANCE 2.0
#define CLUSTER_NEIGHBOR_ANGLE 0.90

#define CORRESPONDENCE_DISTANCE_THRESHOLD 0.2
#define CORRESPONDENCE_ANGLE_THRESHOLD 0.9

#define ODOM_INITIAL_LAMBDA 0.1
#define ODOM_IDENTICAL_METRIX 0.00000001
#define ODOM_H_THRESHOLD 0.0001

struct feature_point{
    float origin_x;
    float origin_y;
    float origin_z;
    float nx;
    float ny;
    float nz;
    float lc_x;
    float lc_y;
    float lc_z;
    float rc_x;
    float rc_y;
    float rc_z;
};

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
            itg_x=0;//    pcl::visualization::CloudViewer viewer3("Cloud Viewer");
//
//
//    viewer3.showCloud(temp);
//
//    while (!viewer3.wasStopped ())
//    {
//
//    }
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
    void create_integral_image(const std::vector < std::vector < spherical_point > > &depth_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_vertical_cloud);
    void create_interval_image(const std::vector < std::vector < spherical_point > > &depth_image);
    pcl::PointCloud<pcl::Normal>::Ptr get_normal(std::vector < std::vector < spherical_point > > &depth_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr vertical_cloud);

private:
    std::vector < std::vector < interval_point > > interval_image;
//    std::vector < std::vector < integral_point > > integral_image;
    std::vector < std::vector < double > > itg_xx;
    std::vector < std::vector < double > > itg_xy;
    std::vector < std::vector < double > > itg_xz;
    std::vector < std::vector < double > > itg_yy;
    std::vector < std::vector < double > > itg_yz;
    std::vector < std::vector < double > > itg_zz;
    std::vector < std::vector < double > > itg_x;
    std::vector < std::vector < double > > itg_y;
    std::vector < std::vector < double > > itg_z;
    std::vector < std::vector < int > > itg_num;

    pcl::PointCloud<pcl::Normal>::Ptr normals;

};

#endif /* image_h */