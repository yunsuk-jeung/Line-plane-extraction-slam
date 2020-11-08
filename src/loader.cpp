#include "loader.h"


loader::loader(){
    cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    vertical_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    vertical_cloud2 = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
//    cloud3 = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    normal_cloud = pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);
//    interval_image(64, std::vector< interval_point>(4500));
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
        temp.r=255;
        temp.g=0;
        temp.b=0;
//        temp.intensity=1;
        cloud->push_back(temp);
        row++;
    }
//    pcl::copyPointCloud(*cloud,*cloud2);
}

void loader::viewer()
{
    pcl::visualization::CloudViewer viewer("Cloud Viewer");


    viewer.showCloud(vertical_cloud2);

    while (!viewer.wasStopped ())
    {
    }

}

void loader::create_depth_image() {
    spherical_point temp;
    int size = cloud->points.size();
    float col_resolution = 2 * M_PI / 4500;
    float min_theta = M_PI;
    float max_theta = 0;
    float r_temp;
    float theta_temp;

    for (int i = 0; i < size; i++) {
        r_temp = sqrt(pow(cloud->points[i].x, 2) + pow(cloud->points[i].y, 2) + pow(cloud->points[i].z, 2));
        theta_temp = acos(cloud->points[i].z / r_temp);
        if (theta_temp >= max_theta) {
            max_theta = theta_temp;
        }
        if (theta_temp <= min_theta) {
            min_theta = theta_temp;
        }
    }
    float row_resolution = (max_theta + 0.0001 - min_theta) / 64;
    int row_num;
    int col_num;
    for (int i = 0; i < size; i++) {
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
        row_num = (temp.theta - min_theta) / row_resolution;
        col_num = temp.pi / col_resolution;
        depth_image[row_num][col_num] = temp;
    }
}

void loader::remove_flat_region(){
    clock_t begin, end;
    float a;
    float b;
    float epsilone;
    int height = 64;
    int length = 4500;
    int num;
    int num_vertical;
    float close_area = 10;
    std::vector<int> temp_vertical_checker;
    spherical_point zero_point;
    for (int i=0; i< length; i++) {
        int vertical_checker[64]={0,};
        for (int j =0; j<height; j++){
            if (vertical_checker[j] == 0){
                if(depth_image[j][i].index != -1 ) {
                    num = 0;
                    a = depth_image[j][i].r * sin(depth_image[j][i].theta);

                    if (a < close_area) {
                        epsilone = 0.002;
                        num_vertical = 2;
                    } else {
                        epsilone = 0.2;
                        num_vertical = 1;
                    }

                    for (int jj = j + 1; jj < height; jj++) {
                        if (depth_image[jj][i].index != -1) {
                            b = depth_image[jj][i].r * sin(depth_image[jj][i].theta);
                            if (fabs(a - b) < epsilone) {
                                num++;
                                temp_vertical_checker.push_back(jj);
                            }
                        }
                    }
                    if (num > num_vertical) {
                        vertical_checker[j] = 1;
                        for (int k=0; k<temp_vertical_checker.size(); k++){
                            vertical_checker[temp_vertical_checker[k]]=1;
                        }
                        temp_vertical_checker.clear();
                    }else{
                        depth_image[j][i] = zero_point;
                    }
                }
            }
        }
    }
    int k=0;
    int index;

    for (int i=0; i<ROW; i++){
        for (int j=0; j<COL; j++){
            index = depth_image[i][j].index;
            if( index != -1){
                vertical_cloud->points.push_back(cloud->points[index]);
                depth_image[i][j].index=k;
                k++;
            }
        }
    }
}

//// call integral_image
void loader::create_image() {
    image integral(ROW+1,COL+1);
    integral.set_boundary(5,5);
    integral.create_integral_image(depth_image, vertical_cloud);
    integral.create_interval_image(depth_image);
    normal_cloud=integral.get_normal(depth_image,vertical_cloud);
    int k=0;
    for (int i=0; i<ROW; i++){
        for (int j=0; j<COL; j++){
            if(depth_image[i][j].index != -1){
                vertical_cloud2->points.push_back(vertical_cloud->points[depth_image[i][j].index]);
                depth_image[i][j].index=k;
                k++;
            }
        }
    }
//    std::cout << vertical_cloud2->points.size() << ' '<< normal_cloud -> points.size() << std::endl;
}
void loader::viewer2() {
    pcl::visualization::PCLVisualizer viewer("pcl viewer");
    viewer.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(vertical_cloud2,normal_cloud,1,1);
    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }
}

void loader::clusterizer(){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);

    std::vector < std::vector < int > > visit(ROW, std::vector <int> (COL));
    std::vector < int > surface_num;
    int neighbor_row= 30;
    int neighbor_col = 50;
    float reference_depth;
    float depth;
    float distance = 4;
    int surface=0;
    int max_surface=0;
    std::cout << 's' << std::endl;

    for (int i = 0; i< ROW; i++){
//        for (int j=0; j<COL; j++){
    int j=4250;
            if (depth_image[i][j].index != -1 && visit[i][j] == 0){
                reference_depth = pow(vertical_cloud2->points[depth_image[i][j].index].x,2)
                                + pow(vertical_cloud2->points[depth_image[i][j].index].y,2)
                                + pow(vertical_cloud2->points[depth_image[i][j].index].z,2);

                if(visit[i][j] == 0){
                    surface++;
                } else{
                    surface = visit[i][j];
                }
                if (surface > max_surface){
                    max_surface=surface;
                }
                for (int ii= i-neighbor_row; ii < i+neighbor_row; ii++){
                    for (int jj = j-neighbor_col; jj < j+neighbor_col; jj++ ){
                        if (ii < 0 || ii >=COL || jj < 0 || jj >= COL ) {
                            continue;
                        }
                        if(depth_image[ii][jj].index <0){
                            continue;
                        }
                        depth = pow(vertical_cloud2->points[depth_image[ii][jj].index].x,2)
                        + pow(vertical_cloud2->points[depth_image[ii][jj].index].y,2)
                        + pow(vertical_cloud2->points[depth_image[ii][jj].index].z,2);

                        if(fabs(reference_depth - depth)< distance){
                            visit[ii][jj] = surface;
                        }
                    }
                }
//            }
        }
    }

    std::cout << max_surface << std::endl;
    pcl::PointXYZRGB temp_point;

    for (int i=0; i<ROW; i++){
        for (int j=4200; j<4300; j++){
            if(depth_image[i][j].index == -1){
                continue;
            }
            else if (visit[i][j] == 1){
                temp_point.x = vertical_cloud2->points[depth_image[i][j].index].x;
                temp_point.y = vertical_cloud2->points[depth_image[i][j].index].y;
                temp_point.z = vertical_cloud2->points[depth_image[i][j].index].z;
                temp_point.r=255;
                temp_point.g=0;
                temp_point.b=0;
                temp->points.push_back(temp_point);
            } else if (visit[i][j] ==2){
                temp_point.x = vertical_cloud2->points[depth_image[i][j].index].x;
                temp_point.y = vertical_cloud2->points[depth_image[i][j].index].y;
                temp_point.z = vertical_cloud2->points[depth_image[i][j].index].z;
                temp_point.r=0;
                temp_point.g=255;
                temp_point.b=0;
                temp->points.push_back(temp_point);
            } else if (visit[i][j] ==3){
                temp_point.x = vertical_cloud2->points[depth_image[i][j].index].x;
                temp_point.y = vertical_cloud2->points[depth_image[i][j].index].y;
                temp_point.z = vertical_cloud2->points[depth_image[i][j].index].z;
                temp_point.r=255;
                temp_point.g=0;
                temp_point.b=0;
                temp->points.push_back(temp_point);
            }
            else{
                temp_point.x = vertical_cloud2->points[depth_image[i][j].index].x;
                temp_point.y = vertical_cloud2->points[depth_image[i][j].index].y;
                temp_point.z = vertical_cloud2->points[depth_image[i][j].index].z;
                temp_point.r=255;
                temp_point.g=255;
                temp_point.b=255;
                temp->points.push_back(temp_point);
            }
        }
    }
//    for (int i=0; i<ROW; i++){
//        for (int j=4200; j<4300; j++){
//            if(visit[i][j] == 0){
//                continue;
//            }
////            else if (visit[i][j] == 1){
////                temp_point.x = vertical_cloud2->points[depth_image[i][j].index].x;
////                temp_point.y = vertical_cloud2->points[depth_image[i][j].index].y;
////                temp_point.z = vertical_cloud2->points[depth_image[i][j].index].z;
////                temp_point.r=255;
////                temp_point.g=0;
////                temp_point.b=0;
////                temp->points.push_back(temp_point);
////            } else if (visit[i][j] ==2){
////                temp_point.x = vertical_cloud2->points[depth_image[i][j].index].x;
////                temp_point.y = vertical_cloud2->points[depth_image[i][j].index].y;
////                temp_point.z = vertical_cloud2->points[depth_image[i][j].index].z;
////                temp_point.r=0;
////                temp_point.g=255;
////                temp_point.b=0;
////                temp->points.push_back(temp_point);
////            }
//            else{
//                temp_point.x = vertical_cloud2->points[depth_image[i][j].index].x;
//                temp_point.y = vertical_cloud2->points[depth_image[i][j].index].y;
//                temp_point.z = vertical_cloud2->points[depth_image[i][j].index].z;
//                temp_point.r=255;
//                temp_point.g=255;
//                temp_point.b=255;
//                temp->points.push_back(temp_point);
//            }
//        }
//    }

    pcl::visualization::CloudViewer viewer("Cloud Viewer");


    viewer.showCloud(temp);

    while (!viewer.wasStopped ())
    {
    }
}
