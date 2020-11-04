#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include "loader.h"

int main (int argc, char** argv)
{
    std::string home_env;
    std::string file_env;
    home_env = getenv("HOME") ;
    file_env = home_env + "/workspace/line_plane/src/extract";
    std:: string fileName;
    fileName = file_env + "/0000000000.txt";

    //
//    loader luck;
//    luck.txt2pcl(fileName);
//    luck.create_depth_image();
//    luck.remove_flat_region();
//    luck.create_image();
//    luck.viewer();
//    luck.get_normal();
    Eigen::Matrix3f cc;
    Eigen::Vector3f c;
    Eigen::Matrix3f cov_matrix;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

    float x[3][3] ={
            {0,1,2},
            {0,1,2},
            {0,1,2}
    };
    float y[3][3] ={
            {0,0,0},
            {1,1,1},
            {2,2,2}
    };
    float z[3][3] ={
            {1,1,1},
            {1,1,1},
            {1,1,1}
    };

    float xx[3][3];
    float xy[3][3];
    float xz[3][3];
    float yy[3][3];
    float yz[3][3];
    float zz[3][3];

    for(int i=0; i<3; i++){
        for (int j=0; j<3;j++){
            xx[i][j] = x[i][j]*x[i][j];
            xy[i][j] = x[i][j]*y[i][j];
            xz[i][j] = x[i][j]*z[i][j];
            yy[i][j] = y[i][j]*y[i][j];
            yz[i][j] = y[i][j]*z[i][j];
            zz[i][j] = z[i][j]*z[i][j];
        }
    }
    for(int i=0; i<3; i++){
        for (int j=1; j<3;j++){
            x[i][j] += x[i][j-1];
            y[i][j] += y[i][j-1];
            z[i][j] += z[i][j-1];
            xx[i][j] += xx[i][j-1];
            xy[i][j] += xy[i][j-1];
            xz[i][j] += xz[i][j-1];
            yy[i][j] += yy[i][j-1];
            yz[i][j] += yz[i][j-1];
            zz[i][j] += zz[i][j-1];

        }
    }
    for(int j=0; j<3; j++){
        for (int i=1; i<3;i++){
            x[i][j] += x[i-1][j];
            y[i][j] += y[i-1][j];
            z[i][j] += z[i-1][j];
            xx[i][j] += xx[i-1][j];
            xy[i][j] += xy[i-1][j];
            xz[i][j] += xz[i-1][j];
            yy[i][j] += yy[i-1][j];
            yz[i][j] += yz[i-1][j];
            zz[i][j] += zz[i-1][j];

        }
    }
    cc(0,0)=xx[2][2];
    cc(0,1)=xy[2][2];
    cc(0,2)=xz[2][2];
    cc(1,0)=xy[2][2];
    cc(1,1)=yy[2][2];
    cc(1,2)=yz[2][2];
    cc(2,0)=xz[2][2];
    cc(2,1)=yz[2][2];
    cc(2,2)=zz[2][2];
    c(0)=x[2][2];
    c(1)=y[2][2];
    c(2)=z[2][2];
    cov_matrix = cc - c*c.transpose()/9 ;
//    cov_matrix(0,0)= 6;
//    cov_matrix(0,1)= 0;
//    cov_matrix(0,2)= 0;
//    cov_matrix(1,0)= 0;
//    cov_matrix(1,1)= -6;
//    cov_matrix(1,2)= 0;
//    cov_matrix(2,0)= 0;
//    cov_matrix(2,1)= 0;
//    cov_matrix(2,2)= 0;
    std::cout<< cov_matrix << std::endl;

    double a;
    double aa;
    double aaa;
    Eigen::EigenSolver<Eigen::Matrix3f> s(cov_matrix);
    a = s.eigenvalues().col(0)[0].real();
    aa = s.eigenvalues().col(0)[1].real();
    aaa = s.eigenvalues().col(0)[2].real();

//    std::cout << a << std::endl;
//    std::cout << aa << std::endl;
//    std::cout << aaa << std::endl;
    std::cout << s.eigenvalues() << std::endl;
    std::cout << s.eigenvectors().col(0) << std::endl;


    return (0);
}