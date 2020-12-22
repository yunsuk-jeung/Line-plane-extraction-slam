#ifndef loader_h
#define loader_h

#include "image.h"


class loader
{
public:
    loader();
    void txt2pcl(const std::string &fileName, const int &num);
    void csv2pcl(const std::string &fileName, const int &num);
    void viewer();
    void remove_flat_region();
    void create_depth_image();
    void create_image();
    void viewer2();
    void clusterizer(std::vector < feature_point  > &Line, std::vector <  feature_point  > &Plane, std::vector < std::vector <float > > &Line_points, std::vector < std::vector <float> > &Plane_points, std::vector < std::vector <float> > &Liene_every_points, std::vector < std::vector <float> > &Plane_every_points);

private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
//    spherical_point depth_image[ROW][COL];
    std::vector < std::vector < spherical_point > > depth_image;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr vertical_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr vertical_cloud2;
    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud;

};


#endif /* loader_h */