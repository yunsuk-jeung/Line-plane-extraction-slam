#ifndef odom_h
#define odom_h

#include "image.h"


    Eigen::Matrix3f get_rotation(Eigen::Matrix<float,6,1> &T);
    Eigen::Matrix<float,3,1> get_translation(Eigen::Matrix<float,6,1> &T);



#endif /* odom_h */