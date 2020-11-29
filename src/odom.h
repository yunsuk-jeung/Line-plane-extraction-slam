#ifndef odom_h
#define odom_h

#include "feature.h"


Eigen::Matrix3f get_rotation(Eigen::Matrix<float,6,1> &T);
Eigen::Matrix<float,3,1> get_translation(Eigen::Matrix<float,6,1> &T);
int check_correspondence(feature_point &feature1, feature_point & feature2);
void find_match(std::vector<feature_point> &feature_1, std::vector<feature_point> &feature_2, std::vector<int> &match);

class odom{
public:
    odom();
    Eigen::Matrix<float,4,4> example(feature &feature_1,feature &feature_2);
private:
    Eigen::Matrix<float,6,1> T;
};



#endif /* odom_h */