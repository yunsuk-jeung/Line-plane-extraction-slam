#include "odom.h"

Eigen::Matrix3f get_rotation(Eigen::Matrix<float,6,1> &T){
    Eigen::Matrix3f Rz;
    Eigen::Matrix3f Ry;
    Eigen::Matrix3f Rx;
    Eigen::Matrix3f R;

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
    return R;
}

Eigen::Matrix<float,3,1> get_translation(Eigen::Matrix<float,6,1> &T){
    Eigen::Matrix<float,3,1> t;

    t(0)=T(0);
    t(1) = T(1);
    t(2) = T(2);
    return t;
}

Eigen::Matrix<float,2,2> d2_rotation(Eigen::Matrix<float,3,1> &T){
    Eigen::Matrix<float, 2,2 > R;
    R.setZero();
    R(0,0)= std::cos(T(2));
    R(0,1)= -std::sin(T(2));
    R(1,0)= std::sin(T(2));
    R(1,1)= std::cos(T(2));

    return R;
}
//
Eigen::Matrix<float,2,1> d2_translation(Eigen::Matrix<float,3,1> &T){
    Eigen::Matrix<float, 2,1 > t;
    t.setZero();
    t(0)=T(0);
    t(1)=T(1);

    return t;
}

float get_distance(Eigen::Matrix<float,2,1> &x1, Eigen::Matrix<float,2,1> &Tx2){
    float d;
    for(int i=0;i <4; i++){
        d = sqrt(pow(x1(0) - Tx2(0),2)
                +pow(x1(1) - Tx2(1),2));
    }
    return d;
}


void set_x2(std::vector < Eigen::Vector3f >  &x2){
    x2[0](0) = 4;
    x2[0](1) = 0;
    x2[0](2) = 1;
    x2[1](0) = 3;
    x2[1](1) = 1;
    x2[1](2) = 2;
    x2[2](0) = 3;
    x2[2](1) = 3;
    x2[2](2) = -1;
    x2[3](0) = 0;
    x2[3](1) = 3;
    x2[3](2) = 0;
    x2[4](0) = -3;
    x2[4](1) = -3;
    x2[4](2) = 1;
    x2[5](0) = -3;
    x2[5](1) = 0;
    x2[5](2) = 0;
    x2[6](0) = -3;
    x2[6](1) = -2;
    x2[6](2) = 1;
    x2[7](0) = 0;
    x2[7](1) = -4;
    x2[7](2) = -1;
    x2[8](0) = 3;
    x2[8](1) = -2;
    x2[8](2) = -2;
}