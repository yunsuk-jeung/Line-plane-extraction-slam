#include <iostream>
//#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include "loader.h"

int main (int argc, char** argv)
{
    std::string home_env;
    std::string file_env;
    home_env = getenv("HOME") ;
    file_env = home_env + "/Downloads/2011_09_26/2011_09_26_drive_0002_extract/velodyne_points/data";
    std:: string fileName;
    fileName = file_env + "/0000000000.txt";

    loader luck;
    luck.txt2pcl(fileName);

    luck.create_depth_image();

    luck.remove_flat_region();
    luck.viewer();


    return (0);
}