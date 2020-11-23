#ifndef loader_h
#define loader_h

#include "image.h"


class loader
{
public:
    loader();
    void txt2pcl(std::string fileName);
    void csv2pcl(std::string fileName);
    void viewer();
    void remove_flat_region();
    void create_depth_image();
    void create_image();
    void viewer2();
    void clusterizer();



private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    spherical_point depth_image[ROW][COL];
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr vertical_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr vertical_cloud2;
    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud;
    std::vector < feature_point > Line;
    std::vector < feature_point > Plane;
};


#endif /* loader_h */