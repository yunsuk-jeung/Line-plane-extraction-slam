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

    for(int i=0; i<1230; i=i+5) {
        feature pre;
        pre.get_feature(fileName, i);
        T_pre=T_key;
        Sophus::SE3f SE3_3;

        for (int j = i+1; j < i+6; j++) {
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
//                R=get_rotation(T_iter);
//                t=get_translation(T_iter);
//                Sophus::SE3f SE3_1(R,t);
            rotated_present.rotate(T_iter);
//            odom B(T_key);
//            B.find_match(pre,rotated_present, moon_dist ,moon_angle);
//            B.example(pre,rotated_present);

            odom B(T_pre);
            B.find_match(pre,rotated_present, moon_dist ,moon_angle);
            odom C(T_pre);
            C.match_update(B);
            T_pre=C.example(pre,present);
                ////loop 2
//                moon_dist = 0.55;
//                moon_angle = 0.90;
//                odom B(T_key);
//                B.find_match(pre,rotated_present,moon_dist,moon_angle);
//                T_iter = B.example(pre, rotated_present);
//                R=get_rotation(T_iter);
//                t=get_translation(T_iter);
//                Sophus::SE3f SE3_2(R,t);
//                SE3_3 = SE3_2 * SE3_1;
//                T_pre = SE3_3.log();
//                std::cout << T_pre.transpose() << std::endl;
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
        std::cout << i << ':' << origin(0) << ' ' << origin(1) << ' ' << origin(2) << "           "  ;
        for(int s=0; s<4; s++){
            for(int ss=0; ss<4;ss++){
                std::cout <<   SE3(s,ss) << ' ';
            }
        }
        std::cout << std::endl;
    }

    end = clock();

//    std::cout << (end - start)/CLOCKS_PER_SEC << std::endl;


    return (0);
}


