#ifndef feature_h
#define feature_h

#include "loader.h"

class feature{
public:
    void get_feature(std::string &fileName, int num);
    void swap_feature(feature &pre_feature);
    std::vector < feature_point > Line;
    std::vector < std::vector < float > > Line_points;
    std::vector < feature_point >  Plane;
    std::vector < std::vector < float > >  Plane_points;
};
#endif /* feature_h */