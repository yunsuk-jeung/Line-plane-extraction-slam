#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include "loader.h"
#include "image.h"
#include <omp.h>
#define len 100000000


int loop(int i,int j){

    return i + j;

}

int main (int argc, char** argv)
{

    std::string home_env;
    std::string file_env;
    home_env = getenv("HOME") ;
    file_env = home_env + "/workspace/line_plane/src/extract";
    std:: string fileName;
    fileName = file_env + "/0000000000.txt";

    //
    loader luck;
    luck.txt2pcl(fileName);
    luck.create_depth_image();
    luck.remove_flat_region();
    luck.create_image();
    luck.clusterizer();
//    luck.viewer();
//    luck.viewer2();

    return 0;
}

