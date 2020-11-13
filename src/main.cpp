#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include "loader.h"
#include "image.h"

int main (int argc, char** argv)
{
//    std::string home_env;
//    std::string file_env;
//    home_env = getenv("HOME") ;
//    file_env = home_env + "/workspace/line_plane/src/extract";
//    std:: string fileName;
//    fileName = file_env + "/0000000000.txt";
//
//    //
//    loader luck;
//    luck.txt2pcl(fileName);
//    luck.create_depth_image();
//    luck.remove_flat_region();
//    luck.create_image();
//    luck.clusterizer();
//    luck.viewer();
//    luck.viewer2();
    Eigen::Matrix3f A;
    A(0,0)=6;
    A(0,1)=0;
    A(0,2)=0;
    A(1,0)=0;
    A(1,1)=6;
    A(1,2)=6;
    A(2,0) = 0;
    A(2,1) =0;
    A(2,2)=1;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> s(A);
    std::cout << s.eigenvectors() << std::endl;
    std::cout << s.eigenvalues() << std::endl;

    double x,y,z;
    x=s.eigenvectors().col(0)[0];
    y=s.eigenvectors().col(0)[1];
    z=s.eigenvectors().col(0)[2];
    std::cout << x << y << z <<std::endl;
    return (0);
}