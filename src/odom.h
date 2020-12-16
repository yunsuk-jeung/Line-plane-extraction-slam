#ifndef odom_h
#define odom_h

#include "feature.h"



Eigen::Matrix3f get_rotation(Eigen::Matrix<float,6,1> &T);
Eigen::Matrix<float,3,1> get_translation(Eigen::Matrix<float,6,1> &T);
int check_correspondence(feature_point &feature1, feature_point & feature2);
void find_match1(std::vector<feature_point> &feature_1, std::vector<feature_point> &feature_2, std::vector<int> &match);
void display(feature &feature_1, feature &feature_2);

class odom{
public:
    odom(Eigen::Matrix<float,6,1> input_T);
    Eigen::Matrix<float,6,1> example(feature &feature_1,feature &feature_2);
    void find_match (feature &feature_1,feature &feature_2, float &dist_threshold, float &angle_threshold);
    void match_update(odom &input_odom);
private:
    Eigen::Matrix<float,6,1> T;
    std::vector < int > line_match;
    std::vector < int > plane_match;
};



#endif /* odom_h */