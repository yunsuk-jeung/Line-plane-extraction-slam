#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include "loader.h"
#include "image.h"
#include "odom.h"
#include <omp.h>
#include "feature.h"

int main (int argc, char** argv)
{
    std::string home_env;
    std::string file_env;
    home_env = getenv("HOME") ;
    file_env = home_env + "/workspace/line_plane/src";
    std:: string fileName;
    fileName = file_env + "/bag2csv.csv";

      feature pre;
      pre.get_feature(fileName ,0);
      pre.get_feature(fileName,1);

//    loader pre_cloud;
//    pre_cloud.csv2pcl(fileName,0);
//    pre_cloud.create_depth_image();
//    pre_cloud.remove_flat_region();
//    pre_cloud.create_image();
//    pre_cloud.clusterizer();

//    std::vector < feature_point > Line;
//    std::vector < feature_point > Plane;
//
//    loader cloud;
//    cloud.csv2pcl(fileName,1);
//    cloud.create_depth_image();
//    cloud.remove_flat_region();
//    cloud.create_depth_image();
//    cloud.clusterizer();

////    luck.viewer2();

    return (0);
}


