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

      feature present;
      present.get_feature(fileName,1);

      odom exam;
      exam.example(pre,present);

    return (0);
}


