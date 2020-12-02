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
    theta = feature1.nx * feature2.nx + feature1.ny * feature2.ny + feature1.nz * feature2.nz;
    int found_correspondence;
    if (d < CORRESPONDENCE_DISTANCE_THRESHOLD && theta > CORRESPONDENCE_ANGLE_THRESHOLD){
        found_correspondence =1;
        return found_correspondence;
    }else{
        found_correspondence =0;
        return 0;
    }
}

void find_match(std::vector<feature_point> &feature_1, std::vector<feature_point> &feature_2, std::vector<int> &match){
    int num=0;
    for (int i=0; i<feature_1.size(); i++){
        match.push_back(-1);
        for(int j=0; j<feature_2.size(); j++){
            if(check_correspondence(feature_1[i],feature_2[j]) == 0){
                continue;
            }else{
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
void get_SE3(feature &feature_1, feature &feature_2, std::vector <int> &line_match, std::vector <int> &plane_match, Eigen::Matrix<float,6,1> &pre_T){
    int line_size =line_match.size();
    int plane_size = plane_match.size();
    int num_d=0;
    for (int i=0; i< line_size; i++){
        if(line_match[i] != -1){
            num_d ++;
        }
    }
    for (int i=0; i< plane_size; i++){
        if(plane_match[i] != -1){
            num_d ++;
        }
    }
//    std::cout << num_d<< std::endl;
//    std::cout << num_d << std::endl;
    Eigen::Matrix<float, Eigen::Dynamic, 6 > J;
    Eigen::Matrix<float, Eigen::Dynamic, 1 > d;
    Eigen::Matrix<float, Eigen::Dynamic, 1 > next_d;

    Eigen::Matrix<float, 6,1 > T;
    Eigen::Matrix<float, 6,1 > dT;

    Eigen::Matrix<float, 3, 1 >  p1;
    Eigen::Matrix<float, 3, 1 >  p2;
    Eigen::Matrix<float, 3, 1 >  Tp2;
    Eigen::Matrix<float, 3, 1 >  u;
    Eigen::Matrix<float, 3, 3 > R;
    Eigen::Matrix<float, 3, 1 > t;

    float initial_guess = 0.0000001;
    float lambda=100;
//    std::cout << line_size << ' ' << plane_size << std::endl;
//    std::cout << num_d << std::endl;
    J.resize(num_d, 6);
    d.resize(num_d,1) ;
    next_d.resize(num_d,1) ;
     int k=0;
     //// pre_d
    for (int i=0;i<line_size; i++){
        if(line_match[i] == -1){
            continue;
        }
        p1(0)=feature_1.Line[i].origin_x;
        p1(1)=feature_1.Line[i].origin_y;
        p1(2)=feature_1.Line[i].origin_z;
        u(0)=feature_1.Line[i].nx;
        u(1)=feature_1.Line[i].ny;
        u(2)=feature_1.Line[i].nz;
        p2(0)=feature_2.Line[line_match[i]].origin_x;
        p2(1)=feature_2.Line[line_match[i]].origin_y;
        p2(2)=feature_2.Line[line_match[i]].origin_z;
        d(k) = line_point_distance(u,p1,p2);
//        std::cout << d(k) << std::endl;
        k++;

    }
//    std::cout << "line done" << std::endl;
    for (int i=0;i<plane_size; i++){
        if(plane_match[i] == -1){
            continue;
        }
        p1(0)=feature_1.Plane[i].origin_x;
        p1(1)=feature_1.Plane[i].origin_y;
        p1(2)=feature_1.Plane[i].origin_z;
        u(0)=feature_1.Plane[i].nx;
        u(1)=feature_1.Plane[i].ny;
        u(2)=feature_1.Plane[i].nz;
        p2(0)=feature_2.Plane[plane_match[i]].origin_x;
        p2(1)=feature_2.Plane[plane_match[i]].origin_y;
        p2(2)=feature_2.Plane[plane_match[i]].origin_z;
        d(k) = plane_point_distance(u,p1,p2);
//        std::cout << d(k) << std::endl;
//        std::cout << p1.transpose() << ' ' << p2.transpose() << ' ' << std::endl;
        k++;
    }

    //// next_d
//    std::cout << d.transpose() << std::endl;
    for (int j=0; j<6; j++){
        T.setZero();
        T(j)=initial_guess;
        R = get_rotation(T);
        t = get_translation(T);
        k=0;
        for (int i=0; i<line_size; i++){
            if(line_match[i] == -1){
                continue;
            }
            p1(0)=feature_1.Line[i].origin_x;
            p1(1)=feature_1.Line[i].origin_y;
            p1(2)=feature_1.Line[i].origin_z;
            u(0)=feature_1.Line[i].nx;
            u(1)=feature_1.Line[i].ny;
            u(2)=feature_1.Line[i].nz;
            p2(0)=feature_2.Line[line_match[i]].origin_x;
            p2(1)=feature_2.Line[line_match[i]].origin_y;
            p2(2)=feature_2.Line[line_match[i]].origin_z;
            p2 = R * p2 +t;
            next_d(k) = line_point_distance(u,p1,p2);
            J(k,j) =( next_d(k) - d(k)) / initial_guess;
            k++;

        }
        for (int i=0;i<plane_size; i++){
            if(plane_match[i] == -1){
                continue;
            }
            p1(0)=feature_1.Plane[i].origin_x;
            p1(1)=feature_1.Plane[i].origin_y;
            p1(2)=feature_1.Plane[i].origin_z;
            u(0)=feature_1.Plane[i].nx;
            u(1)=feature_1.Plane[i].ny;
            u(2)=feature_1.Plane[i].nz;
            p2(0)=feature_2.Plane[plane_match[i]].origin_x;
            p2(1)=feature_2.Plane[plane_match[i]].origin_y;
            p2(2)=feature_2.Plane[plane_match[i]].origin_z;
            p2 = R * p2 +t;
            next_d(k) = plane_point_distance(u,p1,p2);
            J(k,j) = (next_d(k) - d(k)) / initial_guess;
            k++;
        }
    }
//    std::cout << J << std::endl;
    Eigen::Matrix<float , 6, 6> I;
    I.setZero();
    for(int i=0; i< 6; i++){
        I(i,i) =1;
    }
//    std::cout << "start update" << std::endl;
    T.setZero();
    for(int iter=0; iter<1000; iter++){
        Eigen::Matrix<float, 6, 6> C;
        C = J.transpose() * J +  I * lambda;
        dT = C.inverse() * J.transpose() * d * -1;
        if(fabs(dT(0)) < 0.0000001 && fabs(dT(1)) < 0.0000001 && fabs(dT(2)) < 0.0000001
        && fabs(dT(3)) < 0.0000001 && fabs(dT(4)) < 0.0000001 && fabs(dT(5)) < 0.0000001){
            return;
        }
        pre_T = T;
        T = pre_T + dT;
//        std::cout << pre_T.transpose() << std::endl;
//        std::cout << T.transpose() << std::endl;
        //// y(p)
        R = get_rotation(pre_T);
        t = get_translation(pre_T);
        k=0;
        for (int i=0; i<line_size; i++){
            if(line_match[i] == -1){
                continue;
            }
            p1(0)=feature_1.Line[i].origin_x;
            p1(1)=feature_1.Line[i].origin_y;
            p1(2)=feature_1.Line[i].origin_z;
            u(0)=feature_1.Line[i].nx;
            u(1)=feature_1.Line[i].ny;
            u(2)=feature_1.Line[i].nz;
            p2(0)=feature_2.Line[line_match[i]].origin_x;
            p2(1)=feature_2.Line[line_match[i]].origin_y;
            p2(2)=feature_2.Line[line_match[i]].origin_z;
            p2 = R * p2 +t;
            d(k) = line_point_distance(u,p1,p2);
            k++;

        }
        for (int i=0;i<plane_size; i++){
            if(plane_match[i] == -1){
                continue;
            }
            p1(0)=feature_1.Plane[i].origin_x;
            p1(1)=feature_1.Plane[i].origin_y;
            p1(2)=feature_1.Plane[i].origin_z;
            u(0)=feature_1.Plane[i].nx;
            u(1)=feature_1.Plane[i].ny;
            u(2)=feature_1.Plane[i].nz;
            p2(0)=feature_2.Plane[plane_match[i]].origin_x;
            p2(1)=feature_2.Plane[plane_match[i]].origin_y;
            p2(2)=feature_2.Plane[plane_match[i]].origin_z;
            p2 = R * p2 +t;
            d(k) = plane_point_distance(u,p1,p2);
            k++;
        }
        ////y(p+h)
        R = get_rotation(T);
        t = get_translation(T);
        k=0;
        for (int i=0; i<line_size; i++){
            if(line_match[i] == -1){
                continue;
            }
            p1(0)=feature_1.Line[i].origin_x;
            p1(1)=feature_1.Line[i].origin_y;
            p1(2)=feature_1.Line[i].origin_z;
            u(0)=feature_1.Line[i].nx;
            u(1)=feature_1.Line[i].ny;
            u(2)=feature_1.Line[i].nz;
            p2(0)=feature_2.Line[line_match[i]].origin_x;
            p2(1)=feature_2.Line[line_match[i]].origin_y;
            p2(2)=feature_2.Line[line_match[i]].origin_z;
            p2 = R * p2 +t;
            next_d(k) = line_point_distance(u,p1,p2);
            k++;

        }
        for (int i=0;i<plane_size; i++){
            if(plane_match[i] == -1){
                continue;
            }
            p1(0)=feature_1.Plane[i].origin_x;
            p1(1)=feature_1.Plane[i].origin_y;
            p1(2)=feature_1.Plane[i].origin_z;
            u(0)=feature_1.Plane[i].nx;
            u(1)=feature_1.Plane[i].ny;
            u(2)=feature_1.Plane[i].nz;
            p2(0)=feature_2.Plane[plane_match[i]].origin_x;
            p2(1)=feature_2.Plane[plane_match[i]].origin_y;
            p2(2)=feature_2.Plane[plane_match[i]].origin_z;
            p2 = R * p2 +t;
            next_d(k) = plane_point_distance(u,p1,p2);
            k++;
        }
        float numer;
        numer = dT.transpose() * dT;
        J = J + ((next_d - d) - (J * dT)) * dT.transpose() / numer;
    }
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