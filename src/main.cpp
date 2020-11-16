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
    std::vector <double > x1;
    std::vector <double > y1;
    std::vector <double > z1;
    std::vector <double > x2;
    std::vector <double > y2;
    std::vector <double > z2;

    std::vector <Eigen::Vector3f > vx1;
    std::vector <Eigen::Vector3f > vx2;
    std::vector <Eigen::Vector3f > from_vx1;
    Eigen::Vector3f temp;
    Eigen::Matrix<float,1,6> T;
    Eigen::Matrix<float,1,6> dT;

    Eigen::Matrix3f Rz;
    Eigen::Matrix3f Ry;
    Eigen::Matrix3f Rx;
    Eigen::Matrix<float,3,3> rotation;

    Eigen::Vector3f trans;

    x1.push_back(2);
    x1.push_back(-2);
    x1.push_back(-2);
    x1.push_back(2);
    y1.push_back(2);
    y1.push_back(2);
    y1.push_back(-2);
    y1.push_back(-2);
    z1.push_back(0);
    z1.push_back(0);
    z1.push_back(0);
    z1.push_back(0);
    x2.push_back(2);
    x2.push_back(-2);
    x2.push_back(-2);
    x2.push_back(2);
    y2.push_back(1);
    y2.push_back(1);
    y2.push_back(-3);
    y2.push_back(-3);
    z2.push_back(0);
    z2.push_back(0);
    z2.push_back(0);
    z2.push_back(0);
    for (int i=0; i<4; i++){
        temp(0)=x1[i];
        temp(1)=y1[i];
        temp(2)=z1[i];
        vx1.push_back(temp);
    }
    for (int i=0; i<4; i++){
        temp(0)=x2[i];
        temp(1)=y2[i];
        temp(2)=z2[i];
        vx2.push_back(temp);
    }


    T(0)=0;
    T(1)=0;
    T(2)=0;
    T(3)=0;
    T(4)=0;
    T(5)=0;
    for(int i=0; i<3; i++){
        for (int j=0; j<3; j++){
            Rz(i,j) =0;
            Ry(i,j) =0;
            Rx(i,j) =0;
        }
    }
    Rz(0,0) = std::cos(T(3));
    Rz(0,1) = -std::sin(T(3));
    Rz(1,0) = std::sin(T(3));
    Rz(1,1) = Rz(0,0);
    Rz(2,2) =1;

    Ry(0,0) = std::cos(T(4));
    Ry(0,2) = std::sin(T(4));
    Ry(2,0) = -std::sin(T(4));
    Ry(2,2) = std::cos(T(4));
    Ry(1,1)=1;

    Rx(0,0)=1;
    Rx(1,1) = std::cos(T(5));
    Rx(1,2) = -std::sin(T(5));
    Rx(2,1) = std::sin(T(5));
    Rx(2,2) = std::cos(T(5));

    rotation = Rz * Ry * Rx;


    for(int i=0; i<3; i++){
        trans(i)=T(i);
    }
    for(int i=0; i< vx2.size();i++){
        from_vx1.push_back(rotation * vx2[i] + trans);
    }

    Eigen::Matrix<float,6,1> jaco;

    std::cout << from_vx1[0] << std::endl;


    return (0);
}


