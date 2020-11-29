#include "feature.h"

void feature::get_feature(std::string &fileName, int num){

    loader cloud;
    cloud.csv2pcl(fileName,num);
    cloud.create_depth_image();
    cloud.remove_flat_region();
    cloud.create_image();
    cloud.clusterizer(Line,Plane);
//    cloud.viewer2();


}
void feature::swap_feature(feature &pre_feature){
    Line.swap(pre_feature.Line);
    Plane.swap(pre_feature.Plane);
}