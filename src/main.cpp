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
    fileName2 = file_env + "/SE3_point.csv";
    FILE* logFp_;
    logFp_ = fopen(fileName2.c_str(), "wb");
    Eigen::Matrix<float,4,1> point;
    point.setZero();
    point(3) =1;
    Eigen::Matrix<float,4,1> origin;
    Eigen::Matrix<float,4,1> origin_check;
    origin_check = point;

    feature pre;
    pre.get_feature(fileName ,3);
    Eigen::Matrix <float,4,4> SE3;
    Eigen::Matrix <float,4,4> SE3_check;
    SE3.setZero();
    for (int i=0; i<4; i++) {
        SE3(i, i) = 1;
    }
    SE3_check=SE3;

    for(int i=4; i<5 ; i++){
        feature present;
        present.get_feature(fileName,i);
        odom A;
        SE3 = SE3 * A.example(pre,present);
        pre.swap_feature(present);
        origin = SE3 * point;

        float d;
        d = sqrt(pow(origin(0)-origin_check(0),2) + pow(origin(1)-origin_check(1),2)
                + pow(origin(2)-origin_check(2),2));

        if (d > 0.2){
            origin = origin_check;
            SE3 = SE3_check;
        }else{
            origin_check = origin;
            SE3_check = SE3;
        }

        fprintf(logFp_, "%f,%f,%f\n" , origin(0),origin(1),origin(2));

        std::cout << i  << ':' <<(SE3*point).transpose() << " dist: " << d << std::endl;
        std::cout << "<<<<<<<<<<<<<<<<<<<<<<< \n" <<std::endl;
    }
    end = clock();

//    std::cout << (end - start)/CLOCKS_PER_SEC << std::endl;


    return (0);
}


