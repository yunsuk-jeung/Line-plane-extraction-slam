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
//    clock_t start,end;
//    start = clock();
    std::string home_env;
    std::string file_env;
    home_env = getenv("HOME") ;
    file_env = home_env + "/workspace/line_plane/src";
    std:: string fileName;
    fileName = file_env + "/bag2csv.csv";

      feature pre;
      pre.get_feature(fileName ,0);
      Eigen::Matrix <float,4,4> SE3;
      SE3.setZero();
      for (int i=0; i<4; i++){
          SE3(i,i)=1;
      }

      for(int i=0; i< 2; i++){
          feature present;
          present.get_feature(fileName,i);

          odom exam;
          SE3 *= exam.example(pre,present);
          pre.swap_feature(present);

          std::cout << SE3 << std::endl;

      }

    return (0);
}


