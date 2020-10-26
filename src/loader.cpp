#include "loader.h"


loader::loader(){
    cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    vertical_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
//    cloud3 = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
}

void loader::txt2pcl(std::string fileName) {
    pcl::PointXYZRGB temp;
    std:: ifstream file(fileName.c_str());
    std::string line, val;
    int row=0;
    while(std::getline(file,line)) {

        std::stringstream ss(line);

        std::getline(ss, val, ' ');
        temp.x = atof(val.c_str());
        std::getline(ss, val, ' ');
        temp.y = atof(val.c_str());
        std::getline(ss, val, ' ');
        temp.z = atof(val.c_str());
        std::getline(ss, val, ' ');
        temp.r=0;
        temp.g=0;
        temp.b=255;
//        temp.intensity=1;
        cloud->push_back(temp);
        row++;
    }
//    pcl::copyPointCloud(*cloud,*cloud2);
}

void loader::viewer()
{
    pcl::visualization::CloudViewer viewer("Cloud Viewer");

    viewer.showCloud(vertical_cloud);

    while (!viewer.wasStopped ())
    {
    }

}

void loader::create_depth_image(){
    spherical_point temp;
    int size=cloud->points.size();
    float col_resolution = 2*M_PI/4500;
    float row_resolution = 0.3 * M_PI/180;

    int row_num;
    int col_num;

    for (int i=0; i< size; i++) {

        temp.r = sqrt(pow(cloud->points[i].x, 2) + pow(cloud->points[i].y, 2) + pow(cloud->points[i].z, 2));

        if (temp.r == 0) {
            temp.theta = 0;
            temp.pi = 0;
        }
        //// atan2 returns -pi ~ pi --> -a --> 2pi-a
        if (atan2(cloud->points[i].y, cloud->points[i].x) >= 0) {
            temp.pi = atan2(cloud->points[i].y, cloud->points[i].x);
        } else {
            temp.pi = 2 * M_PI + atan2(cloud->points[i].y, cloud->points[i].x);
        }
        temp.theta = acos(cloud->points[i].z / temp.r);
        temp.index = i;

        row_num = (temp.theta - 85 * M_PI / 180) / row_resolution;
        col_num = temp.pi / col_resolution;

        depth_image[row_num][col_num] = temp;
    }
}
////make vector depth_image
//void loader::create_depth_image_2() {
//    spherical_point temp;
//    spherical_point zero_temp;
//    std::vector <spherical_point> row_temp;
//
//    float pre_pi=0;
//    int size=cloud->points.size();
//    float resolution = 2*M_PI/4500;
//    int delta;
//    int pre_delta=-1;
//
//
//    for (int i=0; i< size; i++) {
//        temp.r = sqrt(pow(cloud->points[i].x, 2) + pow(cloud->points[i].y, 2) + pow(cloud->points[i].z, 2));
//        if(temp.r == 0) {
//            temp.theta = 0;
//            temp.pi = 0;
//        }
//        //// atan2 returns -pi ~ pi --> -a --> 2pi-a
//        if( atan2(cloud->points[i].y , cloud->points[i].x) >=0 ) {
//            temp.theta = acos(cloud->points[i].z / temp.r);
//            temp.pi = atan2(cloud->points[i].y , cloud->points[i].x);
//        }
//        else {
//            temp.theta = acos(cloud->points[i].z / temp.r);
//            temp.pi = 2 * M_PI + atan2(cloud->points[i].y , cloud->points[i].x);
//        }
//        temp.index = i;
//        ////
//        delta = temp.pi/resolution;
//
//        if (delta == pre_delta){
//            row_temp.pop_back();
//        }
//
//        int add =0;
//        while(add < delta-(pre_delta+1)) {
//            row_temp.push_back(zero_temp);
//            add++;
//        }
//        row_temp.push_back(temp);
//        //// last
//            if (  temp.pi-pre_pi < 0){
//                row_temp.pop_back();
//                int add2=0;
//                int add3=0;
//                while(add2 < 4500-(pre_delta+1)) {
//                    row_temp.push_back(zero_temp);
//                    add2++;
//                }
//                spherical_depth_image.push_back(row_temp);
//                row_temp.clear();
//
//                while(add3 < delta){
//                    row_temp.push_back(zero_temp);
//                    add3++;
//                }
//                row_temp.push_back(temp);
//            }
//
//        ////Change pre_pi
//        if(atan2(cloud->points[i].y , cloud->points[i].x) >= 0) {
//            pre_pi = atan2(cloud->points[i].y, cloud->points[i].x);
//        }
//        else{
//            pre_pi = 2*M_PI + atan2(cloud->points[i].y, cloud->points[i].x);
//        }
//
//        pre_delta = delta;
//    }
//
//    int add2=0;
//    while(add2 < 4500-(pre_delta+1)) {
//        row_temp.push_back(zero_temp);
//        add2++;
//    }
//    spherical_depth_image.push_back(row_temp);
//}

 void loader::remove_flat_region(){
    clock_t begin, end;
    float a;
    float b;
    float epsilone=0.05;
    int height = 100;
    int length = 4500;
    int num;
    float close_area = 10;
     begin = clock();
    int k=0;

    for (int i=0; i< length; i++) {
        for (int j =0; j<height; j++){
            if(depth_image[j][i].index != -1) {
                num = 0;
                a = depth_image[j][i].r * sin(depth_image[j][i].theta);

                if (a < close_area) {
                    epsilone = 0.002;
                    for (int jj = j + 1; jj < height; jj++) {
                        if (depth_image[jj][i].index != -1) {
                           b = depth_image[jj][i].r * sin(depth_image[jj][i].theta);
                            if (fabs(a - b) < epsilone) {
                            num++;
                         }
                        }
                    }
                }
                if (a >= close_area) {
                    epsilone = 0.4;
                    for (int jj = j + 1; jj < height; jj++) {
                        if (depth_image[jj][i].index != -1) {
                            b = depth_image[jj][i].r * sin(depth_image[jj][i].theta);
                            if (fabs(a - b) < epsilone) {
                                num++;
                            }
                        }
                    }
                }

                if(num > 1){
                    vertical_cloud->points.push_back(cloud->points[depth_image[j][i].index]);
                    depth_image[j][i].index = k;
                    k++;
                }
            }
        }
    }
    end = clock();
//    std::cout << (end-begin) << std::endl;

}
