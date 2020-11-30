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
    clock_t start,end;
    start = clock();
    std::string home_env;
    std::string file_env;
    home_env = getenv("HOME") ;
    file_env = home_env + "/workspace/line_plane/src";
    std:: string fileName;
    std:: string fileName2;
    fileName = file_env + "/bag2csv.csv";
    fileName2 = file_env + "/SE3.csv";
    FILE* logFp_;
    logFp_ = fopen(fileName2.c_str(), "wb");

//    std::ifstream file(fileName.c_str());
//    std::string line;
//    int kk=0;
//    while (std::getline(file,line)){
//        kk++;
//    }
//    std::cout << kk << std::endl;

    feature pre;
    pre.get_feature(fileName ,0);
    Eigen::Matrix <float,4,4> SE3;
    SE3.setZero();
    for (int i=0; i<4; i++) {
        SE3(i, i) = 1;
    }
      
    for(int i=0; i< 1800; i++){
        feature present;
        present.get_feature(fileName,i);
        odom A;
        SE3 *= A.example(pre,present);
        pre.swap_feature(present);
        fprintf(logFp_, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n" ,
                SE3(0,0),SE3(0,1),SE3(0,2),SE3(0,3),
                SE3(1,0),SE3(1,1),SE3(1,2),SE3(1,3),
                SE3(2,0),SE3(2,1),SE3(2,2),SE3(2,3),
                SE3(3,0),SE3(3,1),SE3(3,2),SE3(3,3));
    }
    end = clock();
    std::cout << (end - start)/CLOCKS_PER_SEC << std::endl;
    return (0);
}


