#ifndef loader_h
#define loader_h

#include "image.h"


class loader
{
public:
    loader();
    void txt2pcl(std::string fileName);
    void viewer();
    void remove_flat_region();
    void create_depth_image();
    void create_image();
    void viewer2();


private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
//    std::vector< std::vector < spherical_point > > spherical_depth_image;
    spherical_point depth_image[64][4500];
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr vertical_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr vertical_cloud2;
    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud;
};


#endif /* loader_h */