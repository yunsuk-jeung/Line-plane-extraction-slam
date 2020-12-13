#include "feature.h"

void feature::get_feature(std::string &fileName, int num){

    loader cloud;
    cloud.csv2pcl(fileName,num);
    cloud.create_depth_image();
    cloud.remove_flat_region();
    cloud.create_image();
    cloud.clusterizer(Line,Plane,Line_points,Plane_points,Line_every_points,Plane_every_points);
//    cloud.viewer2();


}
void feature::swap_feature(feature &pre_feature){
    Line.swap(pre_feature.Line);
    Line_points.swap(pre_feature.Line_points);
    Plane.swap(pre_feature.Plane);
    Plane_points.swap(pre_feature.Plane_points);
    Line_every_points.swap(pre_feature.Line_every_points);
    Plane_every_points.swap(pre_feature.Plane_every_points);

}