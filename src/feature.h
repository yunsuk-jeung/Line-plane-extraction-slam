#ifndef feature_h
#define feature_h

#include "loader.h"

class feature{
public:
    void get_feature(std::string &fileName, int num);
private:
    std::vector < std::vector < feature_point > > Line;
    std::vector < std::vector < feature_point > > Plane;
};

#endif /* feature_h */