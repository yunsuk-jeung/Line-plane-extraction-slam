#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include "loader.h"
#include "image.h"
#include <omp.h>

int main (int argc, char** argv)
{
//    std::string home_env;
//    std::string file_env;
//    home_env = getenv("HOME") ;
//    file_env = home_env + "/workspace/line_plane/src/extract";
//    std:: string fileName;
//    fileName = file_env + "/0000000000.txt";
//
//
//    loader luck;
//    luck.txt2pcl(fileName);
//    luck.create_depth_image();
//    luck.remove_flat_region();
//    luck.create_image();
//    luck.clusterizer();
////    luck.viewer();
////    luck.viewer2();
////test odom
    std::vector < Eigen::Vector3f >  x1;
    std::vector < Eigen::Vector3f >  x2;
    std::vector < Eigen::Vector3f > Tx2;
    Eigen::Vector3f temp;
    Eigen::Matrix<float,1,6> T;
    Eigen::Matrix<float,1,6> dT;
    Eigen::Matrix<float,1,6> test_T;
    Eigen::Matrix3f Rz;
    Eigen::Matrix3f Ry;
    Eigen::Matrix3f Rx;
    Eigen::Matrix<float,3,3> R;
    Eigen::Matrix<float,3,1> t;
    Eigen::Vector3f trans;
    Eigen::Vector3f u;

    u(0)=0;
    u(1)=0;
    u(2)=1;

    for(int i=0; i<3; i++){
        for (int j=0; j<3;j++){
            temp(0)=i+1;
            temp(1)=j+1;
            temp(2)=0;
            x2.push_back(temp);
        }

    }
    for(int i=0; i<6; i++){
        T(i)=0;
    }
    T(0)=1;
    T(1)=1;
    T(5) = -M_PI/2;

    for(int i=0; i<3; i++){
        for (int j=0; j<3; j++){
            Rz(i,j) =0;
            Ry(i,j) =0;
            Rx(i,j) =0;
        }
    }
    Rz(0,0) = std::cos(T(5));
    Rz(0,1) = -std::sin(T(5));
    Rz(1,0) = std::sin(T(5));
    Rz(1,1) = Rz(0,0);
    Rz(2,2) =1;

    Ry(0,0) = std::cos(T(4));
    Ry(0,2) = std::sin(T(4));
    Ry(2,0) = -std::sin(T(4));
    Ry(2,2) = std::cos(T(4));
    Ry(1,1)=1;

    Rx(0,0)=1;
    Rx(1,1) = std::cos(T(3));
    Rx(1,2) = -std::sin(T(3));
    Rx(2,1) = std::sin(T(3));
    Rx(2,2) = std::cos(T(3));
    R = Rz * Ry * Rx;

    t(0)=T(0);
    t(1) = T(1);
    t(2) = T(2);

    for (int i=0; i<9; i++){
        x1.push_back(R*x2[i]+t);
    }

    for(int i=0; i<6; i++){
        T(i) =0;
    }



    for(int i=0; i<6; i++){
        dT(i) = 0.1;
    }


    return (0);
}


