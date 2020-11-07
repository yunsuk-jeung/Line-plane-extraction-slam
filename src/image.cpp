#include "image.h"

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
    normals = pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);
}

void image::set_boundary(int max_row, int max_col) {
    boundary_col = max_col;
    boundary_row = max_row;
}

void image::create_integral_image(const spherical_point (&depth_image)[64][4500], pcl::PointCloud<pcl::PointXYZRGB>::Ptr vertical_cloud){
    std::vector <interval_point > row_checked;
    int index;
    for (int i=0; i< ROW; i++){
        for (int j=0; j<COL; j++){
            index = depth_image[i][j].index;
            if (index != -1) {
                itg_x[i+1][j+1] = vertical_cloud->points[index].x;
                itg_y[i+1][j+1] = vertical_cloud->points[index].y;
                itg_z[i+1][j+1] = vertical_cloud->points[index].z;
                itg_xx[i+1][j+1] = vertical_cloud->points[index].x * vertical_cloud->points[index].x;
                itg_xy[i+1][j+1] = vertical_cloud->points[index].x * vertical_cloud->points[index].y;
                itg_xz[i+1][j+1] = vertical_cloud->points[index].x * vertical_cloud->points[index].z;
                itg_yy[i+1][j+1] = vertical_cloud->points[index].y * vertical_cloud->points[index].y;
                itg_yz[i+1][j+1] = vertical_cloud->points[index].y * vertical_cloud->points[index].z;
                itg_zz[i+1][j+1] = vertical_cloud->points[index].z * vertical_cloud->points[index].z;
                itg_num[i+1][j+1] =1;
            }
        }
    }
    //// add through col
    for (int i=1; i<ROW+1; i++){
        for (int j=2; j< COL+1; j++){
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
    for (int j=1; j<COL+1; j++){
        for (int i=2; i< ROW+1; i++){
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
}

interval_point check_horizontal(int i, int j, const spherical_point (&depth_image)[ROW][COL], int boundary_col, float input_depth){
    interval_point output;
    int left_col=j-1;
    int right_col=j+1;
    int up_row=i;
    int bottom_row=i;
    int checker;
    int double_checker;
    float depth;
    float pre_depth;
    ///// right column check
    checker=0;
    double_checker=0;
    pre_depth = input_depth;
    while (right_col < COL && checker < boundary_col && double_checker < 10){
        if(depth_image[i][right_col].index == -1){
            right_col ++;
            double_checker++;
        }
        else{
            depth = depth_image[i][right_col].r;
            if(fabs(depth-pre_depth)< DEPTH_THRESHOLD){
                checker++;
                double_checker++;
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
    double_checker=0;
    pre_depth = input_depth;
    while (left_col >= 0 && checker < boundary_col && double_checker < 10){
        if(depth_image[i][left_col].index == -1){
            left_col --;
            double_checker++;
        }
        else{
            depth = depth_image[i][left_col].r;
            if(fabs(depth-pre_depth)< DEPTH_THRESHOLD){
                checker++;
                double_checker++;
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
    return output;
}
interval_point check_vertical(int i, int j, const spherical_point (&depth_image)[ROW][COL], int boundary_row){
    interval_point output;
    int left_col=j;
    int right_col=j;
    int up_row=i-1;
    int bottom_row=i+1;

    int ver_checker;
    float ver_pre_depth;
    float ver_depth;
    int double_checker=0;
    ver_checker =0;
    ver_pre_depth = depth_image[i][j].r;
    //// up row check
    while (up_row >= 0 && ver_checker < boundary_row && double_checker < 10){
        if(depth_image[up_row][j].index == -1){
            up_row --;
            double_checker ++;
        }else{
            ver_depth = depth_image[up_row][j].r;
            if(fabs(ver_depth-ver_pre_depth) < DEPTH_THRESHOLD){
                ver_checker++;
                up_row --;
                double_checker ++;
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
    //// down row check
    ver_checker =0;
    double_checker=0;
    ver_pre_depth = depth_image[i][j].r;
    while (bottom_row < ROW && ver_checker < boundary_row && double_checker < 10){
        if(depth_image[bottom_row][j].index == -1){
            bottom_row ++;
            double_checker ++;
        }else{
            ver_depth = depth_image[bottom_row][j].r;
            if(fabs(ver_depth-ver_pre_depth) < DEPTH_THRESHOLD){
                ver_checker++;
                bottom_row ++;
                double_checker ++;
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

    output.left_col = left_col;
    output.right_col = right_col;
    output.up_row = up_row;
    output.bottom_row = bottom_row;
    return output;
}

void image::create_interval_image(const spherical_point (&depth_image)[ROW][COL]) {
    interval_point ver;
    interval_point hor;
    interval_point input_interval_point;
    float input_depth;
    float pre_depth;
    int left_col;
    int right_col;
    int up_row;
    int bottom_row;

    for( int i=0 ; i< ROW; i ++){
        for (int j=0; j<COL; j++ ){
            if(depth_image[i][j].index != -1){
                left_col=0;
                right_col =4999;
                pre_depth=depth_image[i][j].r;
                ver = check_vertical(i,j,depth_image,boundary_row);
                up_row = ver.up_row;
                bottom_row = ver.bottom_row;
                for (int ii = i; ii >= up_row; ii--){
                    if (depth_image[ii][j].index == -1){
                        input_depth = pre_depth;
                    }else{
                        input_depth = depth_image[ii][j].r;
                        pre_depth = input_depth;
                    }
                    hor = check_horizontal(ii,j,depth_image,boundary_col,input_depth);
                    if(left_col < hor.left_col){
                        left_col = hor.left_col;
                    }
                    if(right_col > hor.left_col){
                        right_col = hor.right_col;
                    }
                }
                for (int ii = i; ii <= bottom_row; ii++){
                    if (depth_image[ii][j].index == -1){
                        input_depth = pre_depth;
                    }else{
                        input_depth = depth_image[ii][j].r;
                        pre_depth = input_depth;
                    }
                    hor = check_horizontal(ii,j,depth_image,boundary_col,input_depth);
                    if(left_col < hor.left_col){
                        left_col = hor.left_col;
                    }
                    if(right_col > hor.right_col){
                        right_col = hor.right_col;
                    }
                }
                input_interval_point.up_row = up_row;
                input_interval_point.bottom_row = bottom_row;
                input_interval_point.left_col = left_col;
                input_interval_point.right_col = right_col;
                interval_image[i][j] = input_interval_point;

//                std::cout << interval_image[i][j].right_col << std::endl;
//                std::cout << interval_image[i][j].left_col << std::endl;
//                std::cout <<  interval_image[i][j].up_row<<std::endl;
//                std::cout<<   interval_image[i][j].bottom_row<<std::endl;
//                std::cout << 111 << std::endl;
            }
        }
    }

}

pcl::PointCloud<pcl::Normal>::Ptr image::get_normal(spherical_point (&depth_image)[ROW][COL], pcl::PointCloud<pcl::PointXYZRGB>::Ptr vertical_cloud) {
    int left_col;
    int right_col;
    int up_row;
    int bottom_row;
    int num;
    interval_point checked;
    double cxx;
    double cxy;
    double cxz;
    double cyy;
    double cyz;
    double czz;
    double cx;
    double cy;
    double cz;
    double eigen1;
    double eigen2;
    double eigen3;

    pcl::Normal point_normal;
    Eigen::Matrix3f cc;
    Eigen::Vector3f c;
    Eigen::Matrix3f cov_matrix;

    for(int i=0; i<ROW; i++) {
        for (int j=0; j<COL;j++) {
            if(depth_image[i][j].index != -1) {
                left_col = interval_image[i][j].left_col + 1;
                right_col = interval_image[i][j].right_col + 1;
                up_row = interval_image[i][j].up_row + 1;
                bottom_row = interval_image[i][j].bottom_row + 1;
                num = itg_num[bottom_row][right_col] - itg_num[up_row - 1][right_col] -
                      itg_num[bottom_row][left_col - 1] + itg_num[up_row - 1][left_col - 1];

                if (num > 5) {
                    cxx = itg_xx[bottom_row][right_col] - itg_xx[up_row - 1][right_col] -
                          itg_xx[bottom_row][left_col - 1] +
                          itg_xx[up_row - 1][left_col - 1];
                    cxy = itg_xy[bottom_row][right_col] - itg_xy[up_row - 1][right_col] -
                          itg_xy[bottom_row][left_col - 1] +
                          itg_xy[up_row - 1][left_col - 1];
                    cxz = itg_xz[bottom_row][right_col] - itg_xz[up_row - 1][right_col] -
                          itg_xz[bottom_row][left_col - 1] +
                          itg_xz[up_row - 1][left_col - 1];
                    cyy = itg_yy[bottom_row][right_col] - itg_yy[up_row - 1][right_col] -
                          itg_yy[bottom_row][left_col - 1] +
                          itg_yy[up_row - 1][left_col - 1];
                    cyz = itg_yz[bottom_row][right_col] - itg_yz[up_row - 1][right_col] -
                          itg_yz[bottom_row][left_col - 1] +
                          itg_yz[up_row - 1][left_col - 1];
                    czz = itg_zz[bottom_row][right_col] - itg_zz[up_row - 1][right_col] -
                          itg_zz[bottom_row][left_col - 1] +
                          itg_zz[up_row - 1][left_col - 1];
                    cx = itg_x[bottom_row][right_col] - itg_x[up_row - 1][right_col] - itg_x[bottom_row][left_col - 1] +
                         itg_x[up_row - 1][left_col - 1];
                    cy = itg_y[bottom_row][right_col] - itg_y[up_row - 1][right_col] - itg_y[bottom_row][left_col - 1] +
                         itg_y[up_row - 1][left_col - 1];
                    cz = itg_z[bottom_row][right_col] - itg_z[up_row - 1][right_col] - itg_z[bottom_row][left_col - 1] +
                         itg_z[up_row - 1][left_col - 1];

                    cc(0, 0) = cxx;
                    cc(0, 1) = cxy;
                    cc(0, 2) = cxz;
                    cc(1, 0) = cxy;
                    cc(1, 1) = cyy;
                    cc(1, 2) = cyz;
                    cc(2, 0) = cxz;
                    cc(2, 1) = cyz;
                    cc(2, 2) = czz;
                    c(0) = cx;
                    c(1) = cy;
                    c(2) = cz;

                    cov_matrix = cc ;
                    cov_matrix -= c * c.transpose() / num;

                    Eigen::EigenSolver<Eigen::Matrix3f> s(cov_matrix);


                    eigen1 = s.eigenvalues().col(0)[0].real();
                    eigen2 = s.eigenvalues().col(0)[1].real();
                    eigen3 = s.eigenvalues().col(0)[2].real();
//                    eigen1 = fabs(s.eigenvalues().col(0)[0].real());
//                    eigen2 = fabs(s.eigenvalues().col(0)[1].real());
//                    eigen3 = fabs(s.eigenvalues().col(0)[2].real());
                    if (eigen1 < eigen2 && eigen1 < eigen3) {
                        point_normal.normal_x=s.eigenvectors().col(0)[0].real();
                        point_normal.normal_y = s.eigenvectors().col(0)[1].real();
                        point_normal.normal_z = s.eigenvectors().col(0)[2].real();

                    } else if (eigen2 < eigen1 && eigen2 < eigen3) {
                        point_normal.normal_x = s.eigenvectors().col(1)[0].real();
                        point_normal.normal_y = s.eigenvectors().col(1)[1].real();
                        point_normal.normal_z = s.eigenvectors().col(1)[2].real();

                    } else {
                        point_normal.normal_x = s.eigenvectors().col(2)[0].real();
                        point_normal.normal_y = s.eigenvectors().col(2)[1].real();
                        point_normal.normal_z = s.eigenvectors().col(2)[2].real();
                    }
                    if(point_normal.normal_x * vertical_cloud->points[depth_image[i][j].index].x+point_normal.normal_y * vertical_cloud->points[depth_image[i][j].index].y+point_normal.normal_z * vertical_cloud->points[depth_image[i][j].index].z > 0){
                        point_normal.normal_x = -point_normal.normal_x;
                        point_normal.normal_y = -point_normal.normal_y;
                        point_normal.normal_z = -point_normal.normal_z;
                        normals->points.push_back(point_normal);
                    }else if(point_normal.normal_x * vertical_cloud->points[depth_image[i][j].index].x+point_normal.normal_y * vertical_cloud->points[depth_image[i][j].index].y+point_normal.normal_z * vertical_cloud->points[depth_image[i][j].index].z == 0) {
                        depth_image[i][j].index = -1;
                    }else{
                        normals->points.push_back(point_normal);
                    }

                }else{
                    depth_image[i][j].index = -1;
                }
            }
        }
    }
    return normals;
}