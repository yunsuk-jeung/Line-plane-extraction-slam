#include "feature.h"

void feature::get_feature(std::string &fileName, int num){

    loader cloud;
    cloud.csv2pcl(fileName,num);
    cloud.create_depth_image();
    cloud.remove_flat_region();
    cloud.create_image();
//    cloud.viewer();
//    cloud.viewer2();
    cloud.clusterizer(Line,Plane);


}