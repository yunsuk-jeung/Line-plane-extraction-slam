#include "odom.h"

Eigen::Matrix <float,4,4 > get_SE3(Eigen::Matrix<float,6,1> &T);

//Eigen::Matrix3f get_rotation(Eigen::Matrix<float,6,1> &T){
//    Eigen::Matrix3f Rz;
//    Eigen::Matrix3f Ry;
//    Eigen::Matrix3f Rx;
//    Eigen::Matrix3f R;
//
//    for(int i=0; i<3; i++){
//        for (int j=0; j<3; j++){
//            Rz(i,j) =0;
//            Ry(i,j) =0;
//            Rx(i,j) =0;
//        }
//    }
//    Rz(0,0) = std::cos(T(5));
//    Rz(0,1) = -std::sin(T(5));
//    Rz(1,0) = std::sin(T(5));
//    Rz(1,1) = Rz(0,0);
//    Rz(2,2) =1;
//
//    Ry(0,0) = std::cos(T(4));
//    Ry(0,2) = std::sin(T(4));
//    Ry(2,0) = -std::sin(T(4));
//    Ry(2,2) = std::cos(T(4));
//    Ry(1,1)=1;
//
//    Rx(0,0)=1;
//    Rx(1,1) = std::cos(T(3));
//    Rx(1,2) = -std::sin(T(3));
//    Rx(2,1) = std::sin(T(3));
//    Rx(2,2) = std::cos(T(3));
//    R = Rz * Ry * Rx;
//    return R;
//}
//Eigen::Matrix<float,3,1> get_translation(Eigen::Matrix<float,6,1> &T){
//    Eigen::Matrix<float,3,1> t;
//    t(0)=T(0);
//    t(1) = T(1);
//    t(2) = T(2);
//    return t;
//}

int check_correspondence(feature_point &feature1, feature_point & feature2, float &dist_threshold, float &angle_threshold){
    float d,theta;
    d = sqrt(pow((feature1.origin_x - feature2.origin_x),2) + pow((feature1.origin_y - feature2.origin_y),2) + 0.1*pow((feature1.origin_z - feature2.origin_z),2));
    theta = fabs(feature1.nx * feature2.nx + feature1.ny * feature2.ny + feature1.nz * feature2.nz);
    int found_correspondence;
    if (d < dist_threshold && theta > angle_threshold){
        found_correspondence =1;
        return found_correspondence;
    }else{
        found_correspondence =0;
        return found_correspondence;
    }
}

void find_match1(std::vector<feature_point> &feature1, std::vector<feature_point> &feature2, std::vector<int> &match, float &dist_threshold, float &angle_threshold){
    float pre_d;
    float d;
    for (int i=0; i<feature1.size(); i++){
        match.push_back(-1);
        pre_d=100;
        for(int j=0; j<feature2.size(); j++){
            if(check_correspondence(feature1[i],feature2[j], dist_threshold, angle_threshold) == 0){
                continue;
            }
            else{
                d = sqrt(pow((feature1[i].origin_x - feature2[j].origin_x),2) + pow((feature1[i].origin_y - feature2[j].origin_y),2) + pow((feature1[i].origin_z - feature2[j].origin_z),2));
                if (d<pre_d){
                    match[i]= j;
                    pre_d =d;
                }
            }

        }
    }
}
float line_point_distance(Eigen::Matrix<float,3,1> &u,Eigen::Matrix<float,3,1> &p1,Eigen::Matrix<float,3,1> & p2){
    Eigen::Matrix<float,3,1> d;
    float dd;
    d= p1-p2;
    dd = fabs(u.cross(d).norm());
    return dd;
}
float plane_point_distance(Eigen::Matrix<float,3,1> &u,Eigen::Matrix<float,3,1> &p1,Eigen::Matrix<float,3,1> & p2){
    Eigen::Matrix<float,3,1> d;
    float dd;
    d= p1-p2;
    dd = fabs(u.transpose() * d) ;
    return dd;
}

void diagonal(Eigen::Matrix<float,Eigen::Dynamic,6> &J, Eigen::Matrix<float,6,6> &H){
    Eigen::Matrix<float, 6,6> JJ;
    JJ = J.transpose() * J;
    for(int i=0;i<6; i++){
        H(i,i) = JJ(i,i);
    }
}

void get_line_feature_distance(feature_point &Line_1, std::vector <float> &Line_1_points, feature_point &Line_2, std::vector <float> &Line_2_points, Eigen::Matrix<float, Eigen::Dynamic, 1> &d, int &k, Eigen::Matrix<float,3,3> &R, Eigen::Matrix<float,3,1> &t){
    Eigen::Matrix<float, 3, 1 > p1;
    Eigen::Matrix<float, 3, 1 > p2;
    Eigen::Matrix<float, 3, 1 > Tp2;
    Eigen::Matrix<float, 3, 1 > u;

    p1(0)=Line_1.origin_x;
    p1(1)=Line_1.origin_y;
    p1(2)=Line_1.origin_z;
    u(0)=Line_1.nx;
    u(1)=Line_1.ny;
    u(2)=Line_1.nz;

    p2(0)=Line_2.origin_x;
    p2(1)=Line_2.origin_y;
    p2(2)=Line_2.origin_z;

    p2 = R*p2 + t;
    d(k) = sqrt(pow(p1(0)-p2(0),2) + pow(p1(1)-p2(1),2) + pow(p1(2)-p2(2),2));
    k++;

    for (int i=0; i< Line_2_points.size(); i=i+3){
        p2(0)=Line_2_points[i];
        p2(1)=Line_2_points[i+1];
        p2(2)=Line_2_points[i+2];
        p2 = R * p2 +t;
        d(k) = line_point_distance(u,p1,p2);
        k++;
    }
}

void get_plane_feature_distance(feature_point &Plane_1, std::vector < float > &Plane_1_points, feature_point &Plane_2, std::vector < float > &Plane_2_points, Eigen::Matrix<float, Eigen::Dynamic, 1> &d, int &k, Eigen::Matrix<float,3,3> &R, Eigen::Matrix<float,3,1> &t){
    Eigen::Matrix<float, 3, 1 > p1;
    Eigen::Matrix<float, 3, 1 > p2;
    Eigen::Matrix<float, 3, 1 > Tp2;
    Eigen::Matrix<float, 3, 1 > u;


    p1(0)=Plane_1.origin_x;
    p1(1)=Plane_1.origin_y;
    p1(2)=Plane_1.origin_z;
    u(0)=Plane_1.nx;
    u(1)=Plane_1.ny;
    u(2)=Plane_1.nz;

    p2(0)=Plane_2.origin_x;
    p2(1)=Plane_2.origin_y;
    p2(2)=Plane_2.origin_z;

    p2 = R*p2 + t;
    d(k) = sqrt(pow(p1(0)-p2(0),2) + pow(p1(1)-p2(1),2) + pow(p1(2)-p2(2),2));
    k++;

    for (int i=0; i< Plane_2_points.size(); i=i+3){
        p2(0)=Plane_2_points[i];
        p2(1)=Plane_2_points[i+1];
        p2(2)=Plane_2_points[i+2];
        p2 = R * p2 +t;
        d(k) = plane_point_distance(u,p1,p2);
        k++;
    }
}

void get_line_jacobian(feature_point &Line_1, std::vector <float> &Line_1_points, feature_point &Line_2, std::vector <float> &Line_2_points, Eigen::Matrix<float, 6, 1 > &T, Eigen::Matrix<float, Eigen::Dynamic, 6> &J, int &k){

    double x1 = Line_1.origin_x;
    double y1 = Line_1.origin_y;
    double z1 = Line_1.origin_z;
    double nx = Line_1.nx;
    double ny = Line_1.ny;
    double nz = Line_1.nz;

    double tx = T(0);
    double ty = T(1);
    double tz = T(2);
    double r = T(3);
    double p = T(4);
    double w = T(5);
    double x2;
    double y2;
    double z2;
    
    x2 = Line_2.origin_x;
    y2 = Line_2.origin_y;
    z2 = Line_2.origin_z;

    J(k,0) = (2*tx - 2*x1 - 2*y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + 2*z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + 2*x2*cos(p)*cos(w))/     (2*   pow((pow((tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)),2) + pow((ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)),2) + pow((tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)),2)   ),(0.5))    );
    J(k,1) = (2*ty - 2*y1 + 2*y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - 2*z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + 2*x2*cos(p)*sin(w))/     (2*   pow((pow((tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)),2) + pow((ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)),2) + pow((tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)),2)   ),(0.5))    );
    J(k,2) = (2*tz - 2*z1 - 2*x2*sin(p) + 2*z2*cos(p)*cos(r) + 2*y2*cos(p)*sin(r))/                                                              (2*   pow((pow((tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)),2) + pow((ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)),2) + pow((tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)),2)   ),(0.5))    );
    J(k,3) = (2*(y2*cos(p)*cos(r) - z2*cos(p)*sin(r))*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) + 2*(y2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + z2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)))*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)) - 2*(y2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + z2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)))*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)))/       (2*   pow((pow((tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)),2) + pow((ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)),2) + pow((tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)),2)   ),(0.5))    );
    J(k,4) = (2*(z2*cos(p)*cos(r)*cos(w) - x2*cos(w)*sin(p) + y2*cos(p)*cos(w)*sin(r))*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)) - 2*(x2*cos(p) + z2*cos(r)*sin(p) + y2*sin(p)*sin(r))*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) + 2*(z2*cos(p)*cos(r)*sin(w) - x2*sin(p)*sin(w) + y2*cos(p)*sin(r)*sin(w))*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)))/                             (2*   pow((pow((tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)),2) + pow((ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)),2) + pow((tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)),2)   ),(0.5))    );
    J(k,5) = -(2*(y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w))*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)) - 2*(z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + x2*cos(p)*cos(w))*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)))/                                                                      (2*   pow((pow((tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)),2) + pow((ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)),2) + pow((tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)),2)   ),(0.5))    );

    k++;

    for (int i=0; i< Line_2_points.size(); i=i+3){
        x2=Line_2_points[i];
        y2=Line_2_points[i+1];
        z2=Line_2_points[i+2];

        J(k,0) =       -(2*nz*(nx*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w))) - 2*ny*(ny*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)) - nx*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w))))
                       /(2*pow ( (pow((ny*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)) - nx*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w))),2) +     pow(  (nx*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w))),2) +  pow(   (ny*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)))   ,2)      ),0.5));
        J(k,1) =  -(2*nz*(ny*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w))) + 2*nx*(ny*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)) - nx*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w))))/(2*    pow((  pow(   (ny*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)) - nx*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w))),2) + pow((nx*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w))),2) + pow((ny*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w))),2)),0.5));
        J(k,2) =  (2*nx*(nx*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w))) + 2*ny*(ny*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w))))/(2*    pow((   pow( (ny*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)) - nx*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w))) ,2) + pow(  (nx*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w))) ,2) + pow((ny*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w))) ,2)),0.5));
        J(k,3) =  (2*(nx*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)))*(nx*(y2*cos(p)*cos(r) - z2*cos(p)*sin(r)) - nz*(y2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + z2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)))) + 2*(ny*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)))*(ny*(y2*cos(p)*cos(r) - z2*cos(p)*sin(r)) + nz*(y2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + z2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)))) + 2*(nx*(y2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + z2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w))) + ny*(y2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + z2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r))))*(ny*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)) - nx*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w))))/(2*   pow((    pow((ny*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)) - nx*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w))),2) +    pow((nx*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w))),2) + pow( (ny*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w))),2)),0.5));
        J(k,4) =  -(2*(nx*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)))*(nx*(x2*cos(p) + z2*cos(r)*sin(p) + y2*sin(p)*sin(r)) + nz*(z2*cos(p)*cos(r)*cos(w) - x2*cos(w)*sin(p) + y2*cos(p)*cos(w)*sin(r))) - 2*(ny*(z2*cos(p)*cos(r)*cos(w) - x2*cos(w)*sin(p) + y2*cos(p)*cos(w)*sin(r)) - nx*(z2*cos(p)*cos(r)*sin(w) - x2*sin(p)*sin(w) + y2*cos(p)*sin(r)*sin(w)))*(ny*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)) - nx*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w))) + 2*(ny*(x2*cos(p) + z2*cos(r)*sin(p) + y2*sin(p)*sin(r)) + nz*(z2*cos(p)*cos(r)*sin(w) - x2*sin(p)*sin(w) + y2*cos(p)*sin(r)*sin(w)))*(ny*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w))))/(2*  pow((   pow((ny*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)) - nx*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)))  ,2) + pow((nx*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w))),2) + pow((ny*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w))),2))  ,0.5));
        J(k,5) =  -(2*(nx*(z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + x2*cos(p)*cos(w)) + ny*(y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)))*(ny*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)) - nx*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w))) - 2*nz*(nx*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)))*(y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)) + 2*nz*(ny*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)))*(z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + x2*cos(p)*cos(w)))/(2*   pow((pow((ny*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)) - nx*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w))),2) + pow((nx*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w))) ,2) + pow((ny*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)))    ,2)) ,0.5));

        k++;
    }
}

void get_plane_jacobian(feature_point &Plane_1, std::vector < float > &Plane_1_points, feature_point &Plane_2, std::vector < float > &Plane_2_points, Eigen::Matrix<float, 6, 1 > &T, Eigen::Matrix<float, Eigen::Dynamic, 6> &J, int &k){
    double x1 = Plane_1.origin_x;
    double y1 = Plane_1.origin_y;
    double z1 = Plane_1.origin_z;
    double nx = Plane_1.nx;
    double ny = Plane_1.ny;
    double nz = Plane_1.nz;


    double tx = T(0);
    double ty = T(1);
    double tz = T(2);
    double r = T(3);
    double p = T(4);
    double w = T(5);
    double x2 = Plane_2.origin_x;
    double y2 = Plane_2.origin_y;
    double z2 = Plane_2.origin_z;

    J(k,0) = (2*tx - 2*x1 - 2*y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + 2*z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + 2*x2*cos(p)*cos(w))/     (2*   pow((pow((tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)),2) + pow((ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)),2) + pow((tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)),2)   ),(0.5))    );
    J(k,1) = (2*ty - 2*y1 + 2*y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - 2*z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + 2*x2*cos(p)*sin(w))/     (2*   pow((pow((tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)),2) + pow((ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)),2) + pow((tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)),2)   ),(0.5))    );
    J(k,2) = (2*tz - 2*z1 - 2*x2*sin(p) + 2*z2*cos(p)*cos(r) + 2*y2*cos(p)*sin(r))/                                                              (2*   pow((pow((tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)),2) + pow((ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)),2) + pow((tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)),2)   ),(0.5))    );
    J(k,3) = (2*(y2*cos(p)*cos(r) - z2*cos(p)*sin(r))*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) + 2*(y2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + z2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)))*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)) - 2*(y2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + z2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)))*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)))/       (2*   pow((pow((tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)),2) + pow((ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)),2) + pow((tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)),2)   ),(0.5))    );
    J(k,4) = (2*(z2*cos(p)*cos(r)*cos(w) - x2*cos(w)*sin(p) + y2*cos(p)*cos(w)*sin(r))*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)) - 2*(x2*cos(p) + z2*cos(r)*sin(p) + y2*sin(p)*sin(r))*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) + 2*(z2*cos(p)*cos(r)*sin(w) - x2*sin(p)*sin(w) + y2*cos(p)*sin(r)*sin(w))*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)))/                             (2*   pow((pow((tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)),2) + pow((ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)),2) + pow((tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)),2)   ),(0.5))    );
    J(k,5) = -(2*(y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w))*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)) - 2*(z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + x2*cos(p)*cos(w))*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)))/                                                                      (2*   pow((pow((tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)),2) + pow((ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)),2) + pow((tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)),2)   ),(0.5))    );

    k++;

    for (int i=0; i< Plane_2_points.size(); i=i+3){
        x2=Plane_2_points[i];
        y2=Plane_2_points[i+1];
        z2=Plane_2_points[i+2];

        float distance = nz*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) + nx*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)) + ny*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w));

        if (distance >= 0){
            J(k,0) = nx;
            J(k,1) = ny;
            J(k,2) = nz;
            J(k,3) = nz*(y2*cos(p)*cos(r) - z2*cos(p)*sin(r)) + nx*(y2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + z2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r))) - ny*(y2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + z2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)));
            J(k,4) = nx*(z2*cos(p)*cos(r)*cos(w) - x2*cos(w)*sin(p) + y2*cos(p)*cos(w)*sin(r)) - nz*(x2*cos(p) + z2*cos(r)*sin(p) + y2*sin(p)*sin(r)) + ny*(z2*cos(p)*cos(r)*sin(w) - x2*sin(p)*sin(w) + y2*cos(p)*sin(r)*sin(w));
            J(k,5) = ny*(z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + x2*cos(p)*cos(w)) - nx*(y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w));
            k++;
        }else{
            J(k,0) = -nx;
            J(k,1) =-ny;
            J(k,2) =-nz;
            J(k,3) =ny*(y2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + z2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w))) - nx*(y2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + z2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r))) - nz*(y2*cos(p)*cos(r) - z2*cos(p)*sin(r));
            J(k,4) =nz*(x2*cos(p) + z2*cos(r)*sin(p) + y2*sin(p)*sin(r)) - nx*(z2*cos(p)*cos(r)*cos(w) - x2*cos(w)*sin(p) + y2*cos(p)*cos(w)*sin(r)) - ny*(z2*cos(p)*cos(r)*sin(w) - x2*sin(p)*sin(w) + y2*cos(p)*sin(r)*sin(w));
            J(k,5) =nx*(y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)) - ny*(z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + x2*cos(p)*cos(w));
            k++;
        }
    }
}

void get_distance_jacobian(feature_point &Plane_1, std::vector < float > &Plane_1_points, feature_point &Plane_2, std::vector < float > &Plane_2_points, Eigen::Matrix<float, 6, 1 > &T, Eigen::Matrix<float, Eigen::Dynamic, 3> &J, int &k){
    double x1 = Plane_1.origin_x;
    double y1 = Plane_1.origin_y;
    double z1 = Plane_1.origin_z;

    double x2;
    double y2;
    double z2;
    double tx = T(0);
    double ty = T(1);
    double tz = T(2);
    double r = T(3);
    double p = T(4);
    double w = T(5);
    x2 = Plane_2.origin_x;
    y2 = Plane_2.origin_y;
    z2 = Plane_2.origin_z;

    J(k,0) =  (2*tx - 2*x1 - 2*y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + 2*z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + 2*x2*cos(p)*cos(w))/     (2*   pow((pow((tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)),2) + pow((ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)),2) + pow((tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)),2)   ),0.5)    );

    J(k,1) =  (2*ty - 2*y1 + 2*y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - 2*z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + 2*x2*cos(p)*sin(w))/     (2*   pow((pow((tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)),2) + pow((ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)),2) + pow((tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)),2)   ),0.5 )    );

    J(k,2) =  (2*tz - 2*z1 - 2*x2*sin(p) + 2*z2*cos(p)*cos(r) + 2*y2*cos(p)*sin(r))/(2*pow((pow((tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)),2) + pow((ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)),2) + pow((tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)),2)),0.5));

//    J(k,3) = (2*(y2*cos(p)*cos(r) - z2*cos(p)*sin(r))*(tz - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) + 2*(y2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + z2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)))*(tx - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)) - 2*(y2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + z2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)))*(ty + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)))/(2*pow((pow((tz - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)),2) + pow((tx - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)),2) + pow((ty + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)),2) + 1),(1/2)));
//    J(k,4) = (2*(z2*cos(p)*cos(r)*cos(w) - x2*cos(w)*sin(p) + y2*cos(p)*cos(w)*sin(r))*(tx - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)) - 2*(x2*cos(p) + z2*cos(r)*sin(p) + y2*sin(p)*sin(r))*(tz - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) + 2*(z2*cos(p)*cos(r)*sin(w) - x2*sin(p)*sin(w) + y2*cos(p)*sin(r)*sin(w))*(ty + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)))/(2*pow((pow((tz - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)),2) + pow((tx - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)),2) + pow((ty + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)),2) + 1),(1/2))   );
//    J(k,5) = -(2*(y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w))*(tx - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)) - 2*(z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + x2*cos(p)*cos(w))*(ty + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)))/(2*   pow((pow((tz - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)),2) + pow((tx - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)),2) + pow((ty + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)),2) + 1),(1/2)));
    k++;
}

void get_point_point_distance(feature_point &Plane_1, std::vector < float > &Plane_1_points, feature_point &Plane_2, std::vector < float > &Plane_2_points, Eigen::Matrix<float, Eigen::Dynamic, 1> &d, int &k, Eigen::Matrix<float,3,3> &R, Eigen::Matrix<float,3,1> &t){
    Eigen::Matrix<float, 3, 1 > p1;
    Eigen::Matrix<float, 3, 1 > p2;

    p1(0)=Plane_1.origin_x;
    p1(1)=Plane_1.origin_y;
    p1(2)=Plane_1.origin_z;

    p2(0)=Plane_2.origin_x;
    p2(1)=Plane_2.origin_y;
    p2(2)=Plane_2.origin_z;

    p2 = R*p2 + t;
    d(k) = sqrt(pow(p1(0)-p2(0),2) + pow(p1(1)-p2(1),2) + pow(p1(2)-p2(2),2));
    k++;

}


void get_SE3(feature &feature_1, feature &feature_2, std::vector <int> &line_match, std::vector <int> &plane_match, Eigen::Matrix<float,6,1> &pre_T){
    int line_size =line_match.size();
    int plane_size = plane_match.size();
    int num_d=0;
    int num_d2=0;
    for (int i=0; i< line_size; i++){
        if(line_match[i] != -1){
            num_d += feature_2.Line_points[line_match[i]].size()/3;
            num_d++;
//            num_d2++;
        }
    }

    float line_num=num_d;
    if(line_num >0){
//        std::cout << "line_exsit" << ' ' ;
    }

    for (int i=0; i< plane_size; i++){
        if(plane_match[i] != -1){
            num_d += feature_2.Plane_points[plane_match[i]].size()/3;
            num_d ++;
//            num_d2++;
        }
    }
    float plane_num=num_d;
    if(num_d == 0){
        return;
    }
    Eigen::Matrix<float, Eigen::Dynamic, 6 > J;
    Eigen::Matrix<float, Eigen::Dynamic, 1 > d;
    Eigen::Matrix<float, Eigen::Dynamic, 1 > next_d;

    Eigen::Matrix<float, 6,1 > T;
    Eigen::Matrix<float, 6,1 > temp_pre_T;
    Eigen::Matrix<float, 6,1 > temp_T;
    Eigen::Matrix<float, 6,1 > dT;

    Eigen::Matrix<float, 3, 1 > p1;
    Eigen::Matrix<float, 3, 1 > p2;
    Eigen::Matrix<float, 3, 1 > Tp2;
    Eigen::Matrix<float, 3, 1 > u;
    Eigen::Matrix<float, 3, 3 > R;
    Eigen::Matrix<float, 3, 1 > t;

    float identical_matrix_checker = ODOM_IDENTICAL_METRIX;
    float initial_guess = 0.001;
    float lambda=ODOM_INITIAL_LAMBDA;
    float h_threshold = ODOM_H_THRESHOLD;
    float nu=2;
    J.resize(num_d, 6);
    d.resize(num_d,1) ;
    next_d.resize(num_d,1) ;
     int k=0;
     R = get_rotation(pre_T);
     t = get_translation(pre_T);
     //// pre_d
    for (int i=0;i<line_size; i++){
        if(line_match[i] == -1){
            continue;
        }
        get_line_feature_distance(feature_1.Line[i], feature_1.Line_points[i], feature_2.Line[line_match[i]], feature_2.Line_points[line_match[i]], d, k,R,t);
    }
    for (int i=0;i<plane_size; i++){
        if(plane_match[i] == -1){
            continue;
        }
        get_plane_feature_distance(feature_1.Plane[i], feature_1.Plane_points[i],feature_2.Plane[plane_match[i]], feature_2.Plane_points[plane_match[i]], d, k,R,t);
    }

//    std::cout << "matched feature #: " << num_d2<< std::endl;
    float total_d = d.transpose() * d;
//    std::cout << "first distance: " << total_d << std::endl;
//    std::cout << d.transpose() << std::endl;
//    //// next_d
//    for (int j=0; j<6; j++){
//        T.setZero();
//        T(j)=initial_guess;
//        R = get_rotation(T);
//        t = get_translation(T);
//        k=0;
//        for (int i=0; i<line_size; i++){
//            if(line_match[i] == -1){
//                continue;
//            }
//            get_line_feature_distance(feature_1.Line[i], feature_2.Line[line_match[i]], next_d, k, R,t);
//
//        }
//        for (int i=0;i<plane_size; i++){
//            if(plane_match[i] == -1){
//                continue;
//            }
//            get_plane_feature_distance(feature_1.Plane[i], feature_2.Plane[plane_match[i]], next_d, k,R,t);
//        }
//        for(int ii=0; ii< k; ii++){
//            J(ii,j) =(next_d(ii) - d(ii)) / initial_guess;
//        }
//
//    }
//    std::cout << J << std::endl;
//    std::cout << std::endl;
    k=0;
    ////new jaco
    for (int i=0;i<line_size; i++){
        if(line_match[i] == -1){
            continue;
        }
        get_line_jacobian(feature_1.Line[i], feature_1.Line_points[i], feature_2.Line[line_match[i]], feature_2.Line_points[line_match[i]], pre_T, J, k);
    }
    for (int i=0;i<plane_size; i++){
        if(plane_match[i] == -1){
            continue;
        }
        get_plane_jacobian(feature_1.Plane[i], feature_1.Plane_points[i],feature_2.Plane[plane_match[i]], feature_2.Plane_points[plane_match[i]], pre_T, J, k);
    }
    float distance = d.transpose()*d;

    if (d.transpose() * d == 0 ){
        return;
    }

    Eigen::Matrix<float , 6, 6> I;
    Eigen::Matrix<float, 6, 6> H;
    I.setZero();
    H.setZero();
    for(int i=0; i< 6; i++){
        I(i,i) =1;
    }
    T.setZero();
    for(int iter=0; iter<500; iter++){
        Eigen::Matrix<float, 6, 6> C;
        diagonal(J, H);
        C = J.transpose() * J + lambda * H;
        dT = C.inverse() * J.transpose() * d * -1;

        T = pre_T + dT;
        //// y(p)
        ////y(p+h)
        R = get_rotation(T);
        t = get_translation(T);
        k=0;
        for (int i=0; i<line_size; i++){
            if(line_match[i] == -1){
                continue;
            }
            get_line_feature_distance(feature_1.Line[i], feature_1.Line_points[i], feature_2.Line[line_match[i]], feature_2.Line_points[line_match[i]], next_d, k, R,t);
        }
        for (int i=0;i<plane_size; i++){
            if(plane_match[i] == -1){
                continue;
            }
            get_plane_feature_distance(feature_1.Plane[i], feature_1.Plane_points[i],feature_2.Plane[plane_match[i]], feature_2.Plane_points[plane_match[i]], next_d, k,R,t);
        }

        float jaco_numer;
        float h;
        float h_numer;
        h = d.transpose()*d;
        h = h- next_d.transpose() * next_d;
        h_numer = dT.transpose()*(lambda * H * dT - J.transpose()*d );
        h = h/h_numer;

        if(h > h_threshold){
            pre_T = T;
            ////new jaco
            k=0;
            for (int i=0;i<line_size; i++){
                if(line_match[i] == -1){
                    continue;
                }
                get_line_jacobian(feature_1.Line[i], feature_1.Line_points[i], feature_2.Line[line_match[i]], feature_2.Line_points[line_match[i]], pre_T, J, k);
            }
            for (int i=0;i<plane_size; i++){
                if(plane_match[i] == -1){
                    continue;
                }
                get_plane_jacobian(feature_1.Plane[i], feature_1.Plane_points[i],feature_2.Plane[plane_match[i]], feature_2.Plane_points[plane_match[i]], pre_T, J, k);
            }
            nu=2;
            d= next_d;
            if( (1- pow(2* h-1,3)) > 1/3){
                lambda = lambda * (1- pow(2* h-1,3));
            }else{
                lambda = lambda/3;
            }
        }else{
            lambda = nu * lambda;
            nu = 2;
        }
    }

    float distance2;
    distance2 = d.transpose()*d;
//    std::cout  << "delta_distance: "<< -distance + distance2 << std::endl;
//    std::cout << "odometry: " << pre_T.transpose() << std::endl;
//    pre_T(0)=0;
//    pre_T(1)=0;
//    pre_T(2)=0;
//    pre_T.setZero();
    ///////////translation////////////////
//
//    Eigen::Matrix<float, Eigen::Dynamic, 3 > J2;
//    Eigen::Matrix<float, Eigen::Dynamic, 1 > d2;
//    Eigen::Matrix<float, Eigen::Dynamic, 1 > next_d2;
//    Eigen::Matrix<float, 3, 1> pre_T2;
//    pre_T2.setZero();
//
//    J2.resize(num_d2, 3);
//    d2.resize(num_d2,1) ;
//    next_d2.resize(num_d2,1) ;
//
//    k=0;
//    for(int i=0; i<3; i++){
//        pre_T(i)=pre_T2(i);
//    }
//    T=pre_T;
//    R = get_rotation(pre_T);
//    t = get_translation(pre_T);
//    //// pre_d
//    for (int i=0;i<line_size; i++){
//        if(line_match[i] == -1){
//            continue;
//        }
//        get_point_point_distance(feature_1.Line[i], feature_1.Line_points[i], feature_2.Line[line_match[i]], feature_2.Line_points[line_match[i]], d2, k,R,t);
//    }
//
//    for (int i=0;i<plane_size; i++){
//        if(plane_match[i] == -1){
//            continue;
//        }
//        get_point_point_distance(feature_1.Plane[i], feature_1.Plane_points[i],feature_2.Plane[plane_match[i]], feature_2.Plane_points[plane_match[i]], d2, k,R,t);
//    }
//    std::cout << d2.transpose() * d2<< std::endl;
//    ////Jacobian
//    k=0;
//    ////new jaco
//    for (int i=0;i<line_size; i++){
//        if(line_match[i] == -1){
//            continue;
//        }
//        get_distance_jacobian(feature_1.Line[i], feature_1.Line_points[i], feature_2.Line[line_match[i]], feature_2.Line_points[line_match[i]], pre_T, J2, k);
//    }
//    for (int i=0;i<plane_size; i++){
//        if(plane_match[i] == -1){
//            continue;
//        }
//        get_distance_jacobian(feature_1.Plane[i], feature_1.Plane_points[i],feature_2.Plane[plane_match[i]], feature_2.Plane_points[plane_match[i]], pre_T, J2, k);
//    }
//
////    std::cout << J2 << std::endl;
//
//    Eigen::Matrix<float, 3, 3> H2;
//    Eigen::Matrix<float, 3, 1> dT2;
//    Eigen::Matrix<float, 3, 1> T2;
//
//    H2.setZero();
//    lambda=0.1;
//    h_threshold = ODOM_H_THRESHOLD;
//    nu=2;
////    std::cout << d2.transpose()*d2 << std::endl;
//////    std::cout << pre_T.transpose() << std::endl;
//    for(int iter=0; iter<100; iter++){
//        Eigen::Matrix<float, 3, 3> C2;
//        Eigen::Matrix<float, 3, 3 > K;
//        K = J2.transpose() * J2;
//        for(int i=0; i<3;i++){
//            H2(i,i) = K(i,i);
//        }
//        C2 = J2.transpose() * J2 + lambda * H2;
//        dT2 = C2.inverse() * J2.transpose() * d2 * -1;
//        T2 = pre_T2 + dT2;
////        std::cout << dT2.transpose() << std::endl;
//        for (int i=0; i<3; i++){
//            T(i) = T2(i);
//        }
//        //// y(p)
//        ////y(p+h)
//        R = get_rotation(T);
//        t = get_translation(T);
//        k=0;
//        for (int i=0; i<line_size; i++){
//            if(line_match[i] == -1){
//                continue;
//            }
//            get_point_point_distance(feature_1.Line[i], feature_1.Line_points[i], feature_2.Line[line_match[i]], feature_2.Line_points[line_match[i]], next_d2, k, R,t);
//        }
//        for (int i=0;i<plane_size; i++){
//            if(plane_match[i] == -1){
//                continue;
//            }
//            get_point_point_distance(feature_1.Plane[i], feature_1.Plane_points[i],feature_2.Plane[plane_match[i]], feature_2.Plane_points[plane_match[i]], next_d2, k,R,t);
//        }
////        std::cout << next_d2.transpose() * next_d2 << ' ' <<lambda <<  std::endl;
//
//        float jaco_numer;
//        float h;
//        float h_numer;
//        h = d2.transpose()*d2;
//        h = h- next_d2.transpose() * next_d2;
//        h_numer = dT2.transpose()*(lambda * H2 * dT2 - J2.transpose()*d2 );
//        h = h/h_numer;
////        std::cout << h << std::endl;
//        if(h > 0){
//            pre_T = T;
//            d2= next_d2;
//            ////new jaco
//            k=0;
////            std::cout << J2 << std::endl;
//            for (int i=0;i<line_size; i++){
//                if(line_match[i] == -1){
//                    continue;
//                }
//                get_distance_jacobian(feature_1.Line[i], feature_1.Line_points[i], feature_2.Line[line_match[i]], feature_2.Line_points[line_match[i]], pre_T, J2, k);
//            }
//            for (int i=0;i<plane_size; i++){
//                if(plane_match[i] == -1){
//                    continue;
//                }
//                get_distance_jacobian(feature_1.Plane[i], feature_1.Plane_points[i],feature_2.Plane[plane_match[i]], feature_2.Plane_points[plane_match[i]], pre_T, J2, k);
//            }
//            nu=2;
//
//            std::cout << iter << ' ' << d2.transpose()*d2 << ' ' << lambda << std::endl;
//            if( (1- pow(2* h-1,3)) > 1/3){
//                lambda = lambda * (1- pow(2* h-1,3));
//            }else{
//                lambda = lambda/3;
//            }
//        }else{
//            lambda = nu * lambda;
//            nu = 2;
//        }
//
//    }
//    std::cout << pre_T.transpose() << std::endl;
}

odom::odom(Eigen::Matrix<float,6,1> input_T){
    T=input_T;
}
void odom::match_update(odom &input_odom) {
    line_match.assign(input_odom.line_match.begin(), input_odom.line_match.end());
    plane_match.assign(input_odom.plane_match.begin(), input_odom.plane_match.end());

}
void odom::find_match(feature &feature_1,feature &feature_2, float &dist_threshold, float &angle_threshold){

    find_match1(feature_1.Line,feature_2.Line,line_match, dist_threshold, angle_threshold);
    find_match1(feature_1.Plane,feature_2.Plane,plane_match, dist_threshold, angle_threshold);

}

Eigen::Matrix<float,6,1> odom::example(feature &feature_1,feature &feature_2){
    Eigen::Matrix<float, 6, 1> input_T;
    input_T = T;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr check_cloud;
    check_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB check_point;
    get_SE3(feature_1, feature_2,line_match, plane_match, input_T);


//    Eigen::Matrix<float, 3, 3> R;
//    Eigen::Matrix<float ,3, 1> t;
//    R = get_rotation(input_T);
//    t = get_translation(input_T);
//
//    Eigen::Matrix<float, 3,1> p;
//
//    for( int i=0; i<feature_1.Plane.size(); i++){
//        if(plane_match[i] == -1){
//            continue;
//        }
//        for(int j=0; j<feature_1.Plane_every_points[i].size(); j=j+3){
//            check_point.x = feature_1.Plane_every_points[i][j];
//            check_point.y = feature_1.Plane_every_points[i][j+1];
//            check_point.z = feature_1.Plane_every_points[i][j+2];
//            check_point.r = 255;
//            check_point.b = 0;
//            check_point.g = 0;
//            check_cloud->push_back(check_point);
//        }
//        for(int j=0; j<feature_2.Plane_every_points[plane_match[i]].size(); j=j+3){
//            p(0) = feature_2.Plane_every_points[plane_match[i]][j];
//            p(1) = feature_2.Plane_every_points[plane_match[i]][j+1];
//            p(2) =  feature_2.Plane_every_points[plane_match[i]][j+2];
//            p = R * p + t;
//            check_point.x = p(0);
//            check_point.y = p(1);
//            check_point.z = p(2);
//            check_point.r = 0;
//            check_point.b = 0;
//            check_point.g = 255;
//            check_cloud->push_back(check_point);
//        }
//    }
//    for(int j=0; j<100; j++){
//        check_point.x = 0.02 * j;
//        check_point.y = 0;
//        check_point.z =0;
//        check_point.r = 255;
//        check_point.b = 0;
//        check_point.g = 0;
//        check_cloud->push_back(check_point);
//    }
//    for(int j=0; j<100; j++){
//        check_point.x = 0;
//        check_point.y = 0.02 * j;
//        check_point.z =0;
//        check_point.r = 0;
//        check_point.b = 0;
//        check_point.g = 255;
//        check_cloud->push_back(check_point);
//    }
//    for(int j=0; j<100; j++){
//        check_point.x = 0;
//        check_point.y = 0;
//        check_point.z = 0.02 * j;
//        check_point.r = 0;
//        check_point.b = 255;
//        check_point.g = 0;
//        check_cloud->push_back(check_point);
//    }
//
//    pcl::visualization::CloudViewer viewer("Cloud Viewer");
//    viewer.showCloud(check_cloud);
//    while (!viewer.wasStopped ())
//    {
//    }
//    check_cloud->clear();
//
//    for( int i=0; i<feature_1.Plane.size(); i++){
//        for(int j=0; j<feature_1.Plane_every_points[i].size(); j=j+3){
//            check_point.x = feature_1.Plane_every_points[i][j];
//            check_point.y = feature_1.Plane_every_points[i][j+1];
//            check_point.z = feature_1.Plane_every_points[i][j+2];
//            check_point.r = 255 ;
//            check_point.b = 255 ;
//            check_point.g = 255;
//            check_cloud->push_back(check_point);
//        }
//    }
//    for(int i=0; i<feature_2.Plane.size();i++) {
//        for (int j = 0; j < feature_2.Plane_every_points[i].size(); j = j + 3) {
//            p(0) = feature_2.Plane_every_points[i][j];
//            p(1) = feature_2.Plane_every_points[i][j + 1];
//            p(2) = feature_2.Plane_every_points[i][j + 2];
//            p= R*p +t;
//            check_point.x = p(0);
//            check_point.y = p(1);
//            check_point.z = p(2);
//            check_point.r = 0;
//            check_point.b = 0;
//            check_point.g = 255;
//            check_cloud->push_back(check_point);
//        }
//    }
//
//    for(int j=0; j<100; j++){
//        check_point.x = 0.02 * j;
//        check_point.y = 0;
//        check_point.z =0;
//        check_point.r = 255;
//        check_point.b = 0;
//        check_point.g = 0;
//        check_cloud->push_back(check_point);
//    }
//    for(int j=0; j<100; j++){
//        check_point.x = 0;
//        check_point.y = 0.02 * j;
//        check_point.z =0;
//        check_point.r = 0;
//        check_point.b = 0;
//        check_point.g = 255;
//        check_cloud->push_back(check_point);
//    }
//    for(int j=0; j<100; j++){
//        check_point.x = 0;
//        check_point.y = 0;
//        check_point.z = 0.02 * j;
//        check_point.r = 0;
//        check_point.b = 255;
//        check_point.g = 0;
//        check_cloud->push_back(check_point);
//    }
//
//    pcl::visualization::CloudViewer viewer2("Cloud Viewer");
//    viewer2.showCloud(check_cloud);
//    while (!viewer2.wasStopped ())
//    {
//    }


    return input_T;
}