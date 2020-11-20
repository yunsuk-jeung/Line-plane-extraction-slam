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
//    std::vector < Eigen::Vector3f >  x1;
//    std::vector < Eigen::Vector3f >  x2(9);
//    std::vector < Eigen::Vector3f > Tx2;
//    Eigen::Vector3f temp;
//    Eigen::Matrix<float,6,1> T;
//    Eigen::Matrix<float,6,1> next_T;
//    Eigen::Matrix<float,6,1> dT;
//    Eigen::Matrix<float,6,1> temp_T;
//    Eigen::Matrix3f Rz;
//    Eigen::Matrix3f Ry;
//    Eigen::Matrix3f Rx;
//    Eigen::Matrix<float,3,3> R;
//    Eigen::Matrix<float,3,1> t;
//    Eigen::Vector3f trans;
//    Eigen::Vector3f u;
//    Eigen::Matrix < float ,9,1> d;
//    Eigen::Matrix < float ,9,1> next_d;
//    Eigen::Matrix < float ,9,1> temp_d;
//
//    u(0)=1/sqrt(2);
//    u(1)=0;
//    u(2)=1/sqrt(2);
//
//    set_x2(x2);
//    for (int i=0; i< x2.size(); i++){
//        std::cout << x2[i] << std::endl;
//    }
//    for(int i=0; i<6; i++){
//        T(i)=0;
//    }
//    T(0)=1;
//    T(1)=1;
//    T(5) = -M_PI/2;
//
//    for(int i=0; i<3; i++){
//
//        for (int j=0; j<3; j++){
//            Rz(i,j) =0;
//            Ry(i,j) =0;
//            Rx(i,j) =0;
//        }
//    }
//    Rz(0,0) = std::cos(T(5));
//    Rz(0,1) = -std::sin(T(5));
//    Rz(1,0) = std::sin(T(5));
//    Rz(1,1) = Rz(0,0);
//    Rz(2,2) =1;
//
//    Ry(0,0) = std::cos(T(4));
//    Ry(0,2) = std::sin(T(4));
//    Ry(2,0) = -std::sin(T(4));
//    Ry(2,2) = std::cos(T(4));
//    Ry(1,1)=1;
//
//    Rx(0,0)=1;
//    Rx(1,1) = std::cos(T(3));
//    Rx(1,2) = -std::sin(T(3));
//    Rx(2,1) = std::sin(T(3));
//    Rx(2,2) = std::cos(T(3));
//    R = Rz * Ry * Rx;
//
//    t(0)=T(0);
//    t(1) = T(1);
//    t(2) = T(2);
//
//    for (int i=0; i<9; i++){
//        x1.push_back(R*x2[i]+t);
//        std::cout << x1[i] << std::endl;
//
//    }
//    for(int i=0; i<6; i++){
//        T(i) =0;
//    }
//    for(int i=0; i<6; i++){
//        dT(i) = 0.1;
//    }
//
//    next_T = T+dT;
//    Tx2.clear();
//
////    std::cout << T.transpose() << std::endl;
//    //// start for
//
//    for (int k=0; k<4; k++) {
//        R = get_rotation(T);
//        t = get_translation(T);
//
//        for (int i = 0; i < x2.size(); i++) {
//            Tx2.push_back(R * x2[i] + t);
//        }
//        for (int i = 0; i < x2.size(); i++) {
//            Eigen::Vector3f x2_x1;
//            x2_x1 = Tx2[i] - x1[i];
//            d(i) = (fabs(u.cross(x2_x1).norm()));
//        }
//        Tx2.clear();
//
//        Eigen::Matrix<float,9 ,6 > J;
//        J.setZero();
//
//        //// get jacobian
//        for (int i=0; i< 6; i++){
//            temp_T = T;
//            temp_T(i) = next_T(i);
//
//            R = get_rotation(temp_T);
//            t = get_translation(temp_T);
//
//            for (int ii = 0; ii < x2.size(); ii++) {
//            Tx2.push_back(R * x2[ii] + t);
//            }
//
//            for (int ii = 0; ii < x2.size(); ii++) {
//            Eigen::Vector3f x2_x1;
//            x2_x1 = Tx2[ii] - x1[ii];
//            next_d(ii) = (fabs(u.cross(x2_x1).norm()));
//            }
//
//            Tx2.clear();
//
//            for (int ii=0; ii < 9; ii++){
//                J(ii,i) = (next_d(ii)-d(ii))/dT(i);
//            }
//        }
//
//        Eigen::Matrix<float,6,6> H;
//        Eigen::Matrix<float,6,6> C;
//        Eigen::Matrix<float,6,6> K;
//
//
//        C = J.transpose() * J;
////        std::cout << C << std::endl;
////        std::cout << C.inverse() << std::endl;
//        H.setZero();
//        for (int i = 0; i < 6; ++i) {
//            H(i,i) = C(i,i);
//        }
//        H = H/H.norm();
//        K = C;
////        std::cout << K << std::endl;
//
//        T=next_T;
//        next_T = next_T - K.inverse() * J.transpose() * d;
//
//    }
//    std::cout << T.transpose() << std::endl;
//    std::cout << next_T.transpose() << std::endl;
//// simple L-M method
    std::vector < Eigen::Matrix<float,2,1> > x2(4);
    std::vector < Eigen::Matrix<float,2,1> > x1(4);
    x1[0](0)=2; x1[1](0)=-2; x1[2](0) = -2; x1[3](0)=2;
    x1[0](1)=2; x1[1](1)=2; x1[2](1) = -2; x1[3](1)=-2;
    x2[0](0)=-1; x2[1](0)=-1; x2[2](0) = 3; x2[3](0)=3;
    x2[0](1)=1; x2[1](1)=-3; x2[2](1) = -3; x2[3](1)=1;
    Eigen::Matrix<float, 3, 1> T;
    Eigen::Matrix<float, 3, 1> pre_T;
    Eigen::Matrix<float, 3, 1> dT;

    T.setZero();
    Eigen::Matrix<float,2,2> R;
    Eigen::Matrix<float,2,1> t;

    R.setZero();
    t.setZero();
    dT.setOnes();
    dT = 0.01 * dT;

    pre_T.setZero();
    T = pre_T + dT;


    R = d2_rotation(T);
    t = d2_translation(T);



    return (0);
}


