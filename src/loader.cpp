#include "loader.h"


loader::loader(){
    cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    vertical_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
//    cloud3 = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    normals = pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);
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
        temp.g=255;
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
    for (int i=0; i<height; i++){
        for (int j=0; j<length; j++){
            index = depth_image[i][j].index;
            if( index != -1){
                vertical_cloud->points.push_back(cloud->points[index]);
                depth_image[i][j].index=k;
                k++;
            }
        }
    }
    cloud->clear();
}

//// call integral_image
void loader::create_integral_image() {
    int row=sizeof(depth_image) / sizeof(depth_image[0]);
    int col= sizeof(depth_image[0])/sizeof(spherical_point);
    image integral(row,col);
    integral.set_boundary(10,20);
    integral.create_integral_image(depth_image, vertical_cloud);
}

image::image(int row, int col){
    itg_x = std::vector<std::vector<double> > (row, std::vector < double > (col));
    itg_y = std::vector<std::vector<double> > (row, std::vector < double > (col));
    itg_z = std::vector<std::vector<double> > (row, std::vector < double > (col));
    itg_xx = std::vector<std::vector<double> > (row, std::vector < double > (col));
    itg_xy = std::vector<std::vector<double> > (row, std::vector < double > (col));
    itg_xz = std::vector<std::vector<double> > (row, std::vector < double > (col));
    itg_yy = std::vector<std::vector<double> > (row, std::vector < double > (col));
    itg_yz = std::vector<std::vector<double> > (row, std::vector < double > (col));
    itg_zz = std::vector<std::vector<double> > (row, std::vector < double > (col));
    itg_num = std::vector<std::vector<int> > (row, std::vector < int > (col));
    interval_image = std::vector<std::vector<interval_point> > (row, std::vector < interval_point > (col));
}

void image::set_boundary(int max_row, int max_col) {
    boundary_col = max_col;
    boundary_row = max_row;
}

void image::create_integral_image(spherical_point (&depth_image)[64][4500], pcl::PointCloud<pcl::PointXYZRGB>::Ptr vertical_cloud){
    int skipper;
    interval_point checked;
    std::vector <interval_point > row_checked;
    float x,y,z;

    int index=0;
    for (int i=0; i< ROW; i++){
        for (int j=0; j<COL; j++){
            index = depth_image[i][j].index;
            if (index != -1) {
                itg_x[i][j] = vertical_cloud->points[index].x;
                itg_y[i][j] = vertical_cloud->points[index].y;
                itg_z[i][j] = vertical_cloud->points[index].z;
                itg_xx[i][j] = vertical_cloud->points[index].x * vertical_cloud->points[index].x;
                itg_xy[i][j] = vertical_cloud->points[index].x * vertical_cloud->points[index].y;
                itg_xz[i][j] = vertical_cloud->points[index].x * vertical_cloud->points[index].z;
                itg_yy[i][j] = vertical_cloud->points[index].y * vertical_cloud->points[index].y;
                itg_yz[i][j] = vertical_cloud->points[index].y * vertical_cloud->points[index].z;
                itg_zz[i][j] = vertical_cloud->points[index].z * vertical_cloud->points[index].z;
                itg_num[i][j] =1;
            }
        }
    }
    //// add through col
    for (int i=0; i<ROW; i++){
        for (int j=1; j< COL; j++){
            itg_x[i][j] += itg_x[i][j-1];
            itg_y[i][j] += itg_y[i][j-1];
            itg_z[i][j] += itg_z[i][j-1];
            itg_xx[i][j] += itg_xx[i][j-1];
            itg_xy[i][j] += itg_xy[i][j-1];
            itg_xz[i][j] += itg_xz[i][j-1];
            itg_yy[i][j] += itg_yy[i][j-1];
            itg_yz[i][j] += itg_yz[i][j-1];
            itg_zz[i][j] += itg_zz[i][j-1];
            itg_num[i][j] += itg_num[i][j-1];
        }
    }
    //// add through row
    for (int j=0; j<COL; j++){
        for (int i=1; i< ROW; i++){
            itg_x[i][j] += itg_x[i-1][j];
            itg_y[i][j] += itg_y[i-1][j];
            itg_z[i][j] += itg_z[i-1][j];
            itg_xx[i][j] += itg_xx[i-1][j];
            itg_xy[i][j] += itg_xy[i-1][j];
            itg_xz[i][j] += itg_xz[i-1][j];
            itg_yy[i][j] += itg_yy[i-1][j];
            itg_yz[i][j] += itg_yz[i-1][j];
            itg_zz[i][j] += itg_zz[i-1][j];
            itg_num[i][j] += itg_num[i-1][j];
        }
    }
    for (int i= 0 ; i < 10; i++){
}   create_interval_image(depth_image, itg_num);

}

interval_point check_vertical(int i, int j, const spherical_point (&depth_image)[ROW][COL], std::vector < std::vector < int > > itg_num, int boundary_row, int boundary_col){
    interval_point output;
    int left_col=j-1;
    int right_col=j+1;
    int up_row=i-1;
    int bottom_row=i+1;
    int num=1;
    int checker;
    int ver_checker;
    float pre_depth;
    float depth;
    float ver_pre_depth;
    float ver_depth;
    for (int k=0; k< 63;k++){
//       if(depth_image[k][8].index != -1){
           std::cout << k<<' '<<  depth_image[k][8].index - pre_depth<< std::endl;
//       }
    }

    ver_checker =0;
    ver_pre_depth = depth_image[i][j].r;
    //// up row check
    while (up_row >= 0 && ver_checker < boundary_row){
        if(depth_image[up_row][j].index == -1){
            up_row --;
        }else{
            ver_depth = depth_image[up_row][j].r;
            if(fabs(ver_depth-ver_pre_depth) < DEPTH_THRESHOLD){
               ver_checker++;
               up_row --;
               ver_pre_depth = ver_depth;
            }else{
                up_row ++;
                break;
            }

        }
    }
    if (up_row <0){
        up_row = 0;
    }
    std::cout << up_row << std::endl;

    //// down row check
    ver_checker =0;
    ver_pre_depth = depth_image[i][j].r;
    while (bottom_row >= 0 && ver_checker < boundary_row){
        if(depth_image[bottom_row][j].index == -1){
            bottom_row ++;
        }else{
            ver_depth = depth_image[bottom_row][j].r;
            if(fabs(ver_depth-ver_pre_depth) < DEPTH_THRESHOLD){
                ver_checker++;
                bottom_row ++;
                ver_pre_depth = ver_depth;
            }else{
                bottom_row --;
                break;
            }

        }
    }
    if (bottom_row >= ROW){
        bottom_row = ROW-1;
    }

    std::cout << bottom_row << std::endl;
    ///// right column check
    checker=0;
    pre_depth = depth_image[i][j].r;
    while (right_col < 4500 && checker < boundary_col){
        if(depth_image[i][right_col].index == -1){
            right_col ++;
        }
        else{
            depth = depth_image[i][right_col].r;
            if(fabs(depth-pre_depth)< DEPTH_THRESHOLD){
                checker++;
                right_col++;
                pre_depth = depth;
            }else{
                right_col --;

                break;
            }
        }
    }
    if (right_col > COL-1){
        right_col = COL-1;
    }
//// left col check
    checker =0;
    pre_depth = depth_image[i][j].r;
    while (left_col >= 0 && checker < boundary_col){
        if(depth_image[i][left_col].index == -1){
            left_col --;
        }
        else{
            depth = depth_image[i][left_col].r;
            if(fabs(depth-pre_depth)< DEPTH_THRESHOLD){
                checker++;
                left_col--;
                pre_depth = depth;
            }else{
                left_col ++;

                break;
            }
        }
    }
    if (left_col < 0){
        left_col = 0;
    }


    output.left_col = left_col;
    output.right_col = right_col;
    output.up_row = up_row;
    output.bottom_row = bottom_row;
    output.num  = num;
    return output;
}

void image::create_interval_image(const spherical_point (&depth_image)[ROW][COL], std::vector < std::vector < int > > itg_num) {
    interval_point ver;
    interval_point hor;
    int num1;
    int num2;
//    for (int i=0; i<ROW; i++ ){
//        for (int j=0; j<COL; j++){
int i=6;
int j=8;

           ver = check_vertical(i,j,depth_image,itg_num,boundary_row,boundary_col);

            if (ver.num > hor.num){
                interval_image[i][j] = ver;
            }else{
                interval_image[i][j] = hor;
            }

//        }
//    }
}


