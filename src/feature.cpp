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

void feature::copy(feature &input_feature){
    Line.clear();
    Line.assign(input_feature.Line.begin(),input_feature.Line.end() );
    Plane.clear();
    Plane.assign(input_feature.Plane.begin(),input_feature.Plane.end() );
    Line_points.clear();
    Line_points.assign(input_feature.Line_points.begin(),input_feature.Line_points.end() );
    Plane_points.clear();
    Plane_points.assign(input_feature.Plane_points.begin(),input_feature.Plane_points.end() );
    Line_every_points.clear();
    Line_every_points.assign(input_feature.Line_every_points.begin(),input_feature.Line_every_points.end() );
    Plane_every_points.clear();
    Plane_every_points.assign(input_feature.Plane_every_points.begin(),input_feature.Plane_every_points.end() );
}

Eigen::Matrix3f get_rotation(Eigen::Matrix<float,6,1> &T){
    Eigen::Matrix3f Rz;
    Eigen::Matrix3f Ry;
    Eigen::Matrix3f Rx;
    Eigen::Matrix3f R;

    for(int i=0; i<3; i++){
        for (int j=0; j<3; j++){
            Rz(i,j) =0;
            Ry(i,j) =0;
            Rx(i,j) =0;
        }
    }
    Rz(0,0) = std::cos(T(5));
    Rz(0,1) = -std::sin(T(5));
    Rz(1,0) = std::sin(T(5));
    Rz(1,1) = Rz(0,0);
    Rz(2,2) =1;

    Ry(0,0) = std::cos(T(4));
    Ry(0,2) = std::sin(T(4));
    Ry(2,0) = -std::sin(T(4));
    Ry(2,2) = std::cos(T(4));
    Ry(1,1)=1;

    Rx(0,0)=1;
    Rx(1,1) = std::cos(T(3));
    Rx(1,2) = -std::sin(T(3));
    Rx(2,1) = std::sin(T(3));
    Rx(2,2) = std::cos(T(3));
    R = Rz * Ry * Rx;
    return R;
}
Eigen::Matrix<float,3,1> get_translation(Eigen::Matrix<float,6,1> &T){
    Eigen::Matrix<float,3,1> t;
    t(0)=T(0);
    t(1) = T(1);
    t(2) = T(2);
    return t;
}
void feature::rotate(Eigen::Matrix<float, 6, 1> &T){
    Eigen::Matrix<float, 3, 3> R;
    Eigen::Matrix<float, 3, 1> t;
    Eigen::Matrix<float, 3, 1> p;
    Eigen::Matrix<float,3,1> new_p;
    Eigen::Matrix<float,3,1> n;
    Eigen::Matrix<float,3,1> new_n;
    R = get_rotation(T);
    t = get_translation(T);

    for (int i=0; i< Line.size(); i++){
        p(0) = Line[i].origin_x;
        p(1) = Line[i].origin_y;
        p(2) = Line[i].origin_z;
        n(0) = Line[i].nx;
        n(1) = Line[i].ny;
        n(2) = Line[i].nz;

        new_p = R*p +t;
        p = p+ n;
        new_n = R * p + t;
        new_n = new_n-new_p;
        Line[i].origin_x = new_p(0);
        Line[i].origin_y = new_p(1);
        Line[i].origin_z = new_p(2);
        Line[i].nx = new_n(0);
        Line[i].ny = new_n(1);
        Line[i].nz = new_n(2);
    }
    for (int i=0; i< Plane.size(); i++){
        p(0) = Plane[i].origin_x;
        p(1) = Plane[i].origin_y;
        p(2) = Plane[i].origin_z;
        n(0) = Plane[i].nx;
        n(1) = Plane[i].ny;
        n(2) = Plane[i].nz;

        new_p = R*p +t;
        p = p+ n;
        new_n = R * p + t;
        new_n = new_n-new_p;
        Plane[i].origin_x = new_p(0);
        Plane[i].origin_y = new_p(1);
        Plane[i].origin_z = new_p(2);
        Plane[i].nx = new_n(0);
        Plane[i].ny = new_n(1);
        Plane[i].nz = new_n(2);
    }
    for (int i=0; i< Line_points.size(); i++){
        for (int j=0; j<Line_points[i].size(); j=j+3){
            p(0)= Line_points[i][j];
            p(1) = Line_points[i][j+1];
            p(2) = Line_points[i][j+2];
            p= R * p +t;
            Line_points[i][j] = p(0);
            Line_points[i][j+1] = p(1);
            Line_points[i][j+2] = p(2);
        }

    }
    for (int i=0; i< Plane_points.size(); i++){
        for (int j=0; j<Plane_points[i].size(); j=j+3){
            p(0)= Plane_points[i][j];
            p(1) = Plane_points[i][j+1];
            p(2) = Plane_points[i][j+2];
            p= R * p +t;
            Plane_points[i][j] = p(0);
            Plane_points[i][j+1] = p(1);
            Plane_points[i][j+2] = p(2);
        }

    }
    for (int i=0; i< Line_every_points.size(); i++){
        for (int j=0; j<Line_every_points[i].size(); j=j+3){
            p(0)= Line_every_points[i][j];
            p(1) = Line_every_points[i][j+1];
            p(2) = Line_every_points[i][j+2];
            p= R * p +t;
            Line_every_points[i][j] = p(0);
            Line_every_points[i][j+1] = p(1);
            Line_every_points[i][j+2] = p(2);
        }

    }
    for (int i=0; i< Plane_every_points.size(); i++){
        for (int j=0; j<Plane_every_points[i].size(); j=j+3){

            p(0)= Plane_every_points[i][j];
            p(1) = Plane_every_points[i][j+1];
            p(2) = Plane_every_points[i][j+2];
            p= R * p +t;
            Plane_every_points[i][j] = p(0);
            Plane_every_points[i][j+1] = p(1);
            Plane_every_points[i][j+2] = p(2);
        }

    }

}