#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include "loader.h"
#include "image.h"
#include "odom.h"
#include <omp.h>
#include "feature.h"
#include <sophus/se3.hpp>

int main (int argc, char** argv)
{
    clock_t start,end;
    start = clock();
    std::string home_env;
    std::string file_env;
    home_env = getenv("HOME") ;
    file_env = home_env + "/workspace/line_plane/src";
    std:: string fileName;
    std:: string fileName2;
    fileName = file_env + "/bag2csv.csv";
    fileName2 = file_env + "/SE3_point.csv";
    FILE* logFp_;
    logFp_ = fopen(fileName2.c_str(), "wb");
    Eigen::Matrix<float,4,1> point;
    point.setZero();
    point(3) =1;
    Eigen::Matrix<float,4,1> origin;
    Eigen::Matrix<float,4,1> origin_check;
    origin_check = point;
    Eigen::Matrix <float,4,4> SE3;
    Eigen::Matrix <float,4,4> SE3_inte;
    Eigen::Matrix <float,4,4> SE3_check;
    Eigen::Matrix<float, 6, 1> T_key;
    Eigen::Matrix<float, 6, 1> T_temp;
    Eigen::Matrix<float, 6, 1> T_pre;
    Eigen::Matrix<float, 6, 1> T_iter;
    Eigen::Matrix<float ,3,3> R;
    Eigen::Matrix<float,3,1 > t;

    float moon_dist = CORRESPONDENCE_DISTANCE_THRESHOLD;
    float moon_angle = CORRESPONDENCE_ANGLE_THRESHOLD;
    T_key.setZero();
    T_iter.setZero();
    SE3.setZero();

    for (int i=0; i<4; i++) {
        SE3(i, i) = 1;
    }

    SE3_check=SE3;
    SE3_inte = SE3;
//    float se[16]= {0.652359, -0.749059, 0.115493, 5.88519, 0.739822, 0.662445 ,0.117595, 1.7336, -0.164594, 0.00872995, 0.986323 ,-1.49204, 0, 0, 0, 1};
//    for (int i=0; i<16;i++){
//        SE3_inte(i/4,i%4) = se[i];
//    }
    int i=0;
    while( i<300){
        feature pre;
        pre.get_feature(fileName, i);
        T_pre=T_key;

        int j=i+1;

        while(j<i+6){
            feature present;
            feature rotated_present;
            present.get_feature(fileName, j);
            rotated_present.copy(present);
            T_temp = T_pre;
            moon_dist = CORRESPONDENCE_DISTANCE_THRESHOLD + 0.11 * (j-i-1);
            moon_angle = CORRESPONDENCE_ANGLE_THRESHOLD - 0.025 * (j-i-1);
            odom A(T_temp);
            A.find_match(pre, present,moon_dist,moon_angle);
            T_iter = A.example(pre, present);
            rotated_present.rotate(T_iter);
            odom B(T_temp);
            B.find_match(pre,rotated_present, moon_dist ,moon_angle);
            odom C(T_temp);
            C.match_update(B);

            T_pre=C.example(pre,present);
            float sqrt_T = sqrt(pow(T_pre(3),2)+pow(T_pre(4),2)+pow(T_pre(5),2));
            float sqrt_T2 = sqrt(pow(T_temp(3),2)+pow(T_temp(4),2)+pow(T_temp(5),2));

            std::cout << i << ' ' << j << ' ' << sqrt_T << std::endl;
            if( sqrt_T > 0.05){
                i=j;
                break;
            }
            j++;
        }

        R=get_rotation(T_pre);
        t=get_translation(T_pre);
        for(int ii=0;ii <3; ii++){
            for(int jj=0;jj <3; jj++){
                SE3(ii,jj) = R(ii,jj);
            }
        }
        for(int ii=0; ii<3; ii++){
            SE3(ii,3) = t(ii);
        }
        SE3(3,3) = 1;


        SE3_inte = SE3_inte * SE3;
        origin = SE3_inte * point;
        fprintf(logFp_, "%f,%f,%f\n", origin(0), origin(1), origin(2));

        std::cout << i << ": " << origin(0) << ' ' << origin(1) << ' ' << origin(2) << "           "  ;

        //        for(int s=0; s<4; s++){
//            for(int ss=0; ss<4; ss++){
//                std::cout <<   SE3_inte(s,ss) << ',' << ' ';
//            }
//        }
        std::cout << std::endl;
        i=j;
    }
//    for(int i=200; i<240; i=i+2) {
//        feature pre;
//        pre.get_feature(fileName, i);
//        T_pre=T_key;
//
//
//        for (int j = i+1; j < i+3; j++) {
//            feature present;
//            feature rotated_present;
//            present.get_feature(fileName, j);
//            rotated_present.copy(present);
//            T_temp = T_pre;
//            moon_dist = CORRESPONDENCE_DISTANCE_THRESHOLD + 0.11 * (j-i-1);
//            moon_angle = CORRESPONDENCE_ANGLE_THRESHOLD - 0.025 * (j-i-1);
//            odom A(T_temp);
//            A.find_match(pre, present,moon_dist,moon_angle);
//            T_iter = A.example(pre, present);
//
////            display(pre,present);
//
//            rotated_present.rotate(T_iter);
////            std::cout << "rotated" << std::endl;
//            odom B(T_pre);
//            B.find_match(pre,rotated_present, moon_dist ,moon_angle);
//            odom C(T_pre);
//            C.match_update(B);
//            T_pre=C.example(pre,present);
//
////            display(pre,rotated_present);
//
//            float sqrt_T = sqrt(pow(T_pre(3),2)+pow(T_pre(4),2)+pow(T_pre(5),2));
//
////            std::cout << j << ": " << print_T.transpose() << ' ' << sqrt_T << std::endl;
//
//        }
//        R=get_rotation(T_pre);
//        t=get_translation(T_pre);
//        for(int ii=0;ii <3; ii++){
//            for(int jj=0;jj <3; jj++){
//                SE3(ii,jj) = R(ii,jj);
//            }
//        }
//        for(int ii=0; ii<3; ii++){
//            SE3(ii,3) = t(ii);
//        }
//        SE3(3,3) = 1;
//
//
//        SE3_inte = SE3_inte * SE3;
//        origin = SE3_inte * point;
//        fprintf(logFp_, "%f,%f,%f\n", origin(0), origin(1), origin(2));
//
//        std::cout << i << ": " << origin(0) << ' ' << origin(1) << ' ' << origin(2) << "           "  ;
//
//        //        for(int s=0; s<4; s++){
////            for(int ss=0; ss<4; ss++){
////                std::cout <<   SE3_inte(s,ss) << ',' << ' ';
////            }
////        }
//        std::cout << std::endl;
//    }
//    for(int i=240; i<280; i=i+3) {
//        feature pre;
//        pre.get_feature(fileName, i);
//        T_pre=T_key;
//        Sophus::SE3f SE3_3;
//
//        for (int j = i+1; j < i+4; j++) {
//            feature present;
//            feature rotated_present;
//            present.get_feature(fileName, j);
//            rotated_present.copy(present);
//            T_temp = T_pre;
//            moon_dist = CORRESPONDENCE_DISTANCE_THRESHOLD + 0.11 * (j-i-1);
//            moon_angle = CORRESPONDENCE_ANGLE_THRESHOLD - 0.025 * (j-i-1);
//            odom A(T_temp);
//            A.find_match(pre, present,moon_dist,moon_angle);
//            T_iter = A.example(pre, present);
////            display(pre,present);
//            rotated_present.rotate(T_iter);
////            std::cout << "rotated" << std::endl;
//            odom B(T_pre);
//            B.find_match(pre,rotated_present, moon_dist ,moon_angle);
//            odom C(T_pre);
//            C.match_update(B);
//            T_pre=C.example(pre,present);
////            display(pre,rotated_present);
//            R=get_rotation(T_pre);
//            t=get_translation(T_pre);
//            for(int ii=0;ii <3; ii++){
//                for(int jj=0;jj <3; jj++){
//                    SE3(ii,jj) = R(ii,jj);
//                }
//            }
//            for(int ii=0; ii<3; ii++){
//                SE3(ii,3) = t(ii);
//            }
//            SE3(3,3) = 1;
//            Sophus::SE3f SE3_4(R,t);
//            Eigen::Matrix<float, 6, 1> print_T;
//            print_T = SE3_4.log();
//            float sqrt_T = sqrt(pow(print_T(3),2)+pow(print_T(4),2)+pow(print_T(5),2));
//            std::cout << print_T.transpose() << ' ' << sqrt_T << std::endl;
//        }
//
//        R=get_rotation(T_pre);
//        t=get_translation(T_pre);
//        for(int ii=0;ii <3; ii++){
//            for(int jj=0;jj <3; jj++){
//                SE3(ii,jj) = R(ii,jj);
//            }
//        }
//        for(int ii=0; ii<3; ii++){
//            SE3(ii,3) = t(ii);
//        }
//        SE3(3,3) = 1;
//        SE3_inte = SE3_inte * SE3;
//        origin = SE3_inte * point;
//
//
//
//
//        fprintf(logFp_, "%f,%f,%f\n", origin(0), origin(1), origin(2));
//        std::cout << i << ':' << origin(0) << ' ' << origin(1) << ' ' << origin(2) << "           "  ;
//
////        for(int s=0; s<4; s++){
////            for(int ss=0; ss<4;ss++){
////                std::cout <<   SE3_inte(s,ss) << ',' << ' ';
////            }
////        }
//        std::cout << std::endl;
//    }
////    for(int i=220; i<230; i=i+5) {
////        feature pre;
////        pre.get_feature(fileName, i);
////        T_pre=T_key;
////        Sophus::SE3f SE3_3;
////
////        for (int j = i+1; j < i+6; j++) {
////            feature present;
////            feature rotated_present;
////            present.get_feature(fileName, j);
////            rotated_present.copy(present);
////            T_temp = T_pre;
////            moon_dist = CORRESPONDENCE_DISTANCE_THRESHOLD + 0.11 * (j-i-1);
////            moon_angle = CORRESPONDENCE_ANGLE_THRESHOLD - 0.025 * (j-i-1);
////            odom A(T_temp);
////            A.find_match(pre, present,moon_dist,moon_angle);
////            T_iter = A.example(pre, present);
//////            display(pre,present);
////            rotated_present.rotate(T_iter);
//////            std::cout << "rotated" << std::endl;
////            odom B(T_pre);
////            B.find_match(pre,rotated_present, moon_dist ,moon_angle);
////            odom C(T_pre);
////            C.match_update(B);
////            T_pre=C.example(pre,present);
//////            display(pre,rotated_present);
////        }
////        R=get_rotation(T_pre);
////        t=get_translation(T_pre);
////        for(int ii=0;ii <3; ii++){
////            for(int jj=0;jj <3; jj++){
////                SE3(ii,jj) = R(ii,jj);
////            }
////        }
////        for(int ii=0; ii<3; ii++){
////            SE3(ii,3) = t(ii);
////        }
////        SE3(3,3) = 1;
////        SE3_inte = SE3_inte * SE3;
////        origin = SE3_inte * point;
////        fprintf(logFp_, "%f,%f,%f\n", origin(0), origin(1), origin(2));
////        std::cout << i << ':' << origin(0) << ' ' << origin(1) << ' ' << origin(2) << "           "  ;
////        for(int s=0; s<4; s++){
////            for(int ss=0; ss<4;ss++){
////                std::cout <<   SE3_inte(s,ss) << ',' << ' ';
////            }
////        }
////        std::cout << std::endl;
////    }
//    end = clock();

//    std::cout << (end - start)/CLOCKS_PER_SEC << std::endl;


    return (0);
}


