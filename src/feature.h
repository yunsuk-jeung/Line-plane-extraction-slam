#ifndef feature_h
#define feature_h

#include "loader.h"

class feature{
public:
    void get_feature(std::string &fileName, int num);
    void swap_feature(feature &pre_feature);
    void rotate(Eigen::Matrix<float,6,1> &T);
    void copy(feature &input_feature);
    std::vector < feature_point > Line;
    std::vector < std::vector < float > > Line_points;
    std::vector < feature_point >  Plane;
    std::vector < std::vector < float > >  Plane_points;
    std::vector < std::vector < float > >  Line_every_points;
    std::vector < std::vector < float > >  Plane_every_points;

};
#endif /* feature_h */