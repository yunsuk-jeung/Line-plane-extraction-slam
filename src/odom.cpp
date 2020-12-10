#include "odom.h"
Eigen::Matrix <float,4,4 > get_SE3(Eigen::Matrix<float,6,1> &T);

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

int check_correspondence(feature_point &feature1, feature_point & feature2){
    float d,theta;
    d = sqrt(pow((feature1.origin_x - feature2.origin_x),2) + pow((feature1.origin_y - feature2.origin_y),2) + pow((feature1.origin_z - feature2.origin_z),2));
    theta = fabs(feature1.nx * feature2.nx + feature1.ny * feature2.ny + feature1.nz * feature2.nz);
    int found_correspondence;
    if (d < CORRESPONDENCE_DISTANCE_THRESHOLD && theta > CORRESPONDENCE_ANGLE_THRESHOLD){
        found_correspondence =1;
        return found_correspondence;
    }else{
        found_correspondence =0;
        return found_correspondence;
    }
}

void find_match(std::vector<feature_point> &feature_1, std::vector<feature_point> &feature_2, std::vector<int> &match){
    int num=0;
    for (int i=0; i<feature_1.size(); i++){
        match.push_back(-1);
        for(int j=0; j<feature_2.size(); j++){
            if(check_correspondence(feature_1[i],feature_2[j]) == 0){
                continue;
            }
            else{
                match[i]= j;
                break;
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
void get_line_feature_distance(feature_point &Line_1, std::vector <float> &Line_1_points, feature_point &Line_2, std::vector <float> &Line_2_points, Eigen::Matrix<float, Eigen::Dynamic, 1> &d, int &k){
    Eigen::Matrix<float, 3, 1 > p1;
    Eigen::Matrix<float, 3, 1 > p2;
    Eigen::Matrix<float, 3, 1 > u;

    p1(0)=Line_1.origin_x;
    p1(1)=Line_1.origin_y;
    p1(2)=Line_1.origin_z;
    u(0)=Line_1.nx;
    u(1)=Line_1.ny;
    u(2)=Line_1.nz;
    for (int i=0; i< Line_2_points.size(); i=i+3){
        p2(0)=Line_2_points[i];
        p2(1)=Line_2_points[i+1];
        p2(2)=Line_2_points[i+2];

        d(k) = line_point_distance(u,p1,p2);
        k++;
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

    for (int i=0; i< Line_2_points.size(); i=i+3){
        p2(0)=Line_2_points[i];
        p2(1)=Line_2_points[i+1];
        p2(2)=Line_2_points[i+2];
        p2 = R * p2 +t;
        d(k) = line_point_distance(u,p1,p2);
        k++;
    }
}

void get_plane_feature_distance(feature_point &Plane_1, std::vector < float > &Plane_1_points, feature_point &Plane_2, std::vector < float > &Plane_2_points, Eigen::Matrix<float, Eigen::Dynamic, 1> &d, int &k){
    Eigen::Matrix<float, 3, 1 > p1;
    Eigen::Matrix<float, 3, 1 > p2;
    Eigen::Matrix<float, 3, 1 > u;


    p1(0)=Plane_1.origin_x;
    p1(1)=Plane_1.origin_y;
    p1(2)=Plane_1.origin_z;
    u(0)=Plane_1.nx;
    u(1)=Plane_1.ny;
    u(2)=Plane_1.nz;

    for (int i=0; i< Plane_2_points.size(); i=i+3){
        p2(0)=Plane_2_points[i];
        p2(1)=Plane_2_points[i+1];
        p2(2)=Plane_2_points[i+2];
        d(k) = plane_point_distance(u,p1,p2);
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

    float x1 = Line_1.origin_x;
    float y1 = Line_1.origin_y;
    float z1 = Line_1.origin_z;
    float nx = Line_1.nx;
    float ny = Line_1.ny;
    float nz = Line_1.nz;

    float tx = T(0);
    float ty = T(1);
    float tz = T(2);
    float r = T(3);
    float p = T(4);
    float w = T(5);
    float x2;
    float y2;
    float z2;

    for (int i=0; i< Line_2_points.size(); i=i+3){
        x2=Line_2_points[i];
        y2=Line_2_points[i+1];
        z2=Line_2_points[i+2];

        J(k,0) =       -(2*nz*(nx*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w))) - 2*ny*(ny*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)) - nx*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w))))
                       /(2*pow ( (pow((ny*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)) - nx*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w))),2) +     pow(  (nx*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w))),2) +  pow(   (ny*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)))   ,2)      ),(1/2)));
        J(k,1) =  -(2*nz*(ny*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w))) + 2*nx*(ny*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)) - nx*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w))))/(2*    pow((  pow(   (ny*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)) - nx*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w))),2) + pow((nx*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w))),2) + pow((ny*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w))),2)),(1/2)));
        J(k,2) =  (2*nx*(nx*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w))) + 2*ny*(ny*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w))))/(2*    pow((   pow( (ny*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)) - nx*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w))) ,2) + pow(  (nx*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w))) ,2) + pow((ny*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w))) ,2)),(1/2)));
        J(k,3) =  (2*(nx*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)))*(nx*(y2*cos(p)*cos(r) - z2*cos(p)*sin(r)) - nz*(y2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + z2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)))) + 2*(ny*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)))*(ny*(y2*cos(p)*cos(r) - z2*cos(p)*sin(r)) + nz*(y2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + z2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)))) + 2*(nx*(y2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + z2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w))) + ny*(y2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + z2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r))))*(ny*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)) - nx*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w))))/(2*   pow((    pow((ny*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)) - nx*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w))),2) +    pow((nx*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w))),2) + pow( (ny*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w))),2)),(1/2)));
        J(k,4) =  -(2*(nx*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)))*(nx*(x2*cos(p) + z2*cos(r)*sin(p) + y2*sin(p)*sin(r)) + nz*(z2*cos(p)*cos(r)*cos(w) - x2*cos(w)*sin(p) + y2*cos(p)*cos(w)*sin(r))) - 2*(ny*(z2*cos(p)*cos(r)*cos(w) - x2*cos(w)*sin(p) + y2*cos(p)*cos(w)*sin(r)) - nx*(z2*cos(p)*cos(r)*sin(w) - x2*sin(p)*sin(w) + y2*cos(p)*sin(r)*sin(w)))*(ny*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)) - nx*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w))) + 2*(ny*(x2*cos(p) + z2*cos(r)*sin(p) + y2*sin(p)*sin(r)) + nz*(z2*cos(p)*cos(r)*sin(w) - x2*sin(p)*sin(w) + y2*cos(p)*sin(r)*sin(w)))*(ny*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w))))/(2*  pow((   pow((ny*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)) - nx*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)))  ,2) + pow((nx*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w))),2) + pow((ny*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w))),2))  ,(1/2)));
        J(k,5) =  -(2*(nx*(z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + x2*cos(p)*cos(w)) + ny*(y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)))*(ny*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)) - nx*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w))) - 2*nz*(nx*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)))*(y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)) + 2*nz*(ny*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)))*(z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + x2*cos(p)*cos(w)))/(2*   pow((pow((ny*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w)) - nx*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w))),2) + pow((nx*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(tx - x1 - y2*(cos(r)*sin(w) - cos(w)*sin(p)*sin(r)) + z2*(sin(r)*sin(w) + cos(r)*cos(w)*sin(p)) + x2*cos(p)*cos(w))) ,2) + pow((ny*(tz - z1 - x2*sin(p) + z2*cos(p)*cos(r) + y2*cos(p)*sin(r)) - nz*(ty - y1 + y2*(cos(r)*cos(w) + sin(p)*sin(r)*sin(w)) - z2*(cos(w)*sin(r) - cos(r)*sin(p)*sin(w)) + x2*cos(p)*sin(w)))    ,2)) ,(1/2)));

        k++;
    }
}

void get_plane_jacobian(feature_point &Plane_1, std::vector < float > &Plane_1_points, feature_point &Plane_2, std::vector < float > &Plane_2_points, Eigen::Matrix<float, 6, 1 > &T, Eigen::Matrix<float, Eigen::Dynamic, 6> &J, int &k){
    float x1 = Plane_1.origin_x;
    float y1 = Plane_1.origin_y;
    float z1 = Plane_1.origin_z;
    float nx = Plane_1.nx;
    float ny = Plane_1.ny;
    float nz = Plane_1.nz;

    float x2;
    float y2;
    float z2;
    float tx = T(0);
    float ty = T(1);
    float tz = T(2);
    float r = T(3);
    float p = T(4);
    float w = T(5);

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

void get_SE3(feature &feature_1, feature &feature_2, std::vector <int> &line_match, std::vector <int> &plane_match, Eigen::Matrix<float,6,1> &pre_T){
    int line_size =line_match.size();
    int plane_size = plane_match.size();
    int num_d=0;
    for (int i=0; i< line_size; i++){
        if(line_match[i] != -1){
            num_d += feature_2.Line_points[line_match[i]].size();
        }
    }

    float line_num=num_d;
    if(line_num >0){
        std::cout << "line_exsit" << ' ' ;
    }

    for (int i=0; i< plane_size; i++){
        if(plane_match[i] != -1){
            num_d += feature_2.Plane_points[plane_match[i]].size();
        }
    }
    float plane_num=num_d;
//    std::cout << plane_num << std::endl;
//    std::cout << num_d<< std::endl;
//    std::cout << num_d << std::endl;

    num_d =num_d/3;
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
//    std::cout << num_d << std::endl;
    J.resize(num_d, 6);
    d.resize(num_d,1) ;
    next_d.resize(num_d,1) ;
//    std::cout << num_d << std::endl;
     int k=0;
     //// pre_d
    for (int i=0;i<line_size; i++){
        if(line_match[i] == -1){
            continue;
        }
        get_line_feature_distance(feature_1.Line[i], feature_1.Line_points[i], feature_2.Line[line_match[i]], feature_2.Line_points[line_match[i]], d, k);
    }

    for (int i=0;i<plane_size; i++){
        if(plane_match[i] == -1){
            continue;
        }
        get_plane_feature_distance(feature_1.Plane[i], feature_1.Plane_points[i],feature_2.Plane[plane_match[i]], feature_2.Plane_points[plane_match[i]], d, k);
//        std::cout << d << "asdfasdfasdfasf" << std::endl;
    }
//    std::cout << d <<std::endl;
    std::cout << "matched feature #: " << num_d/5 << std::endl;
    float total_d = d.transpose() * d;
    std::cout << "first distance: " << total_d << std::endl;
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

//    std::cout << J.transpose() * J << std::endl;

    float distance = d.transpose()*d;

    if (d.transpose() * d == 0 ){
        std::cout << "here?" <<std::endl;
        std::cout  << "delat_distance: "<< distance << std::endl;
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
    std::cout  << "delat_distance: "<< -distance + distance2 << std::endl;
    std::cout << pre_T.transpose() << std::endl;
}

odom::odom(){
    T.setZero();
}

Eigen::Matrix<float,4,4> odom::example(feature &feature_1,feature &feature_2){
    std::vector < int > line_match;
    std::vector < int > plane_match;
    find_match(feature_1.Line,feature_2.Line,line_match);
    find_match(feature_1.Plane,feature_2.Plane,plane_match);
    get_SE3(feature_1, feature_2,line_match, plane_match, T);

    Eigen::Matrix<float,4,4> SE3;
    Eigen::Matrix<float, 3, 3 > R;
    Eigen::Matrix<float, 3, 1 > t;
    SE3.setZero();
    R=get_rotation(T);
    t=get_translation(T);
    for(int i=0;i <3; i++){
        for(int j=0;j <3; j++){
            SE3(i,j) = R(i,j);
        }
    }
    for(int i=0; i<3; i++){
        SE3(i,3) = t(i);
    }
    SE3(3,3) = 1;

    return SE3;
}