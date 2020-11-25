#ifndef odom_h
#define odom_h

#include "feature.h"


Eigen::Matrix3f get_rotation(Eigen::Matrix<float,6,1> &T);
Eigen::Matrix<float,3,1> get_translation(Eigen::Matrix<float,6,1> &T);
void set_x2(std::vector < Eigen::Vector3f >  &x2);
Eigen::Matrix<float,2,2> d2_rotation(Eigen::Matrix<float,3,1> &T);
Eigen::Matrix<float,2,1> d2_translation(Eigen::Matrix<float,3,1> &T);
float get_distance(Eigen::Matrix<float,2,1> &x1, Eigen::Matrix<float,2,1> &Tx2);

class odom{
public:
    void example(feature &feature_1,feature &feature_2);
private:
    Eigen::Matrix<float,6,1> pre_T;
    Eigen::Matrix<float,6,1> T;

};



#endif /* odom_h */