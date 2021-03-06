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
const int ROW =16;
const int COL =900;
const float CLOSE_FLAT_REGION_RANGE =0.4;
const float CLOSE_VERTICAL_THRESHOLD =3;
const float FAR_FLAT_REGION_RANGE =0.5;
const float FAR_VERTICAL_THRESHOLD =3;
const float NORMAL_DEPTH_THRESHOLD =0.3;
const float neighbouring_radius =0.3;
const float INTERVAL_CIRCULAR_LEVELS =3;
const float LINE_EIGENVALUE_THRESHOLD =0.02;
const float PLANE_EIGENVALUE_THRESHOLD =0.03;
const float CLUSTER_NEIGHBOR_ROW =4;
const float CLUSTER_NEIGHBOR_COL =25;
const float CLUSTER_NEIGHBOR_ROW_VALID =8;
const float CLUSTER_NEIGHBOR_COL_VALID= 100;
const float CLUSTER_NEIGHBOR_DISTANCE =1;
const float CLUSTER_NEIGHBOR_ANGLE =0.95;
const float CORRESPONDENCE_DISTANCE_THRESHOLD =0.5;
const float CORRESPONDENCE_ANGLE_THRESHOLD =0.90;
const float ODOM_INITIAL_LAMBDA =1000;
const float ODOM_IDENTICAL_METRIX =0.00000001;
const float ODOM_H_THRESHOLD= 0.001;

struct feature_point{
    float origin_x;
    float origin_y;
    float origin_z;
    float nx;
    float ny;
    float nz;
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