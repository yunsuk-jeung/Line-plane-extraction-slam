#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include "loader.h"
#include "image.h"
#include "odom.h"
#include <omp.h>

int main (int argc, char** argv)
{
    std::string home_env;
    std::string file_env;
    home_env = getenv("HOME") ;
    file_env = home_env + "/workspace/line_plane/src/extract";
    std:: string fileName;
    fileName = file_env + "/bag2csv.csv";
//
//
    loader pre_cloud;
    pre_cloud.csv2pcl(fileName);
    pre_cloud.create_depth_image();
    pre_cloud.remove_flat_region();
    pre_cloud.create_image();
    pre_cloud.viewer();
    pre_cloud.viewer2();
    pre_cloud.clusterizer();



////    luck.viewer2();

    return (0);
}


