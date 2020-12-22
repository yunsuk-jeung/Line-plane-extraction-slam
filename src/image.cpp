#include "image.h"

image::image(int row, int col){
//    integral_image = std::vector<std::vector<integral_point> > (row+1, std::vector < integral_point> (col+1));
    itg_xx = std::vector<std::vector < double > > (row+1, std::vector < double> (col+1));
    itg_xy =std::vector<std::vector < double > > (row+1, std::vector < double> (col+1));
    itg_xz =std::vector<std::vector < double > > (row+1, std::vector < double> (col+1));
    itg_yy =std::vector<std::vector < double > > (row+1, std::vector < double> (col+1));
    itg_yz =std::vector<std::vector < double > > (row+1, std::vector < double> (col+1));
    itg_zz =std::vector<std::vector < double > > (row+1, std::vector < double> (col+1));
    itg_x =std::vector<std::vector < double > > (row+1, std::vector < double> (col+1));
    itg_y = std::vector<std::vector < double > > (row+1, std::vector < double> (col+1));
    itg_z = std::vector<std::vector < double > > (row+1, std::vector < double> (col+1));
    itg_num = std::vector<std::vector < int > > (row+1, std::vector < int> (col+1));
    interval_image = std::vector<std::vector<interval_point> > (row, std::vector < interval_point > (col));
    normals = pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);
}

//// integral image with vector <itegral_point>
//void image::create_integral_image(const spherical_point (&depth_image)[64][4500], pcl::PointCloud<pcl::PointXYZRGB>::Ptr vertical_cloud){
//    std::vector <interval_point > row_checked;
//    int index;
//    for (int i=0; i< ROW; i++){
//        for (int j=0; j<COL; j++){
//            index = depth_image[i][j].index;
//            if (index != -1) {
//                integral_image[i+1][j+1].itg_x = vertical_cloud->points[index].x;
//                integral_image[i+1][j+1].itg_y = vertical_cloud->points[index].y;
//                integral_image[i+1][j+1].itg_z = vertical_cloud->points[index].z;
//                integral_image[i+1][j+1].itg_xx = vertical_cloud->points[index].x * vertical_cloud->points[index].x;
//                integral_image[i+1][j+1].itg_xy = vertical_cloud->points[index].x * vertical_cloud->points[index].y;
//                integral_image[i+1][j+1].itg_xz = vertical_cloud->points[index].x * vertical_cloud->points[index].z;
//                integral_image[i+1][j+1].itg_yy = vertical_cloud->points[index].y * vertical_cloud->points[index].y;
//                integral_image[i+1][j+1].itg_yz = vertical_cloud->points[index].y * vertical_cloud->points[index].z;
//                integral_image[i+1][j+1].itg_zz = vertical_cloud->points[index].z * vertical_cloud->points[index].z;
//                integral_image[i+1][j+1].itg_num =1;
//            }
//        }
//    }
//    //// add through col
//    for (int i=1; i<ROW+1; i++){
//        for (int j=2; j< COL+1; j++){
//            integral_image[i][j].itg_x += integral_image[i][j-1].itg_x;
//            integral_image[i][j].itg_y += integral_image[i][j-1].itg_y;
//            integral_image[i][j].itg_z += integral_image[i][j-1].itg_z;
//            integral_image[i][j].itg_xx += integral_image[i][j-1].itg_xx;
//            integral_image[i][j].itg_xy += integral_image[i][j-1].itg_xy;
//            integral_image[i][j].itg_xz += integral_image[i][j-1].itg_xz;
//            integral_image[i][j].itg_yy += integral_image[i][j-1].itg_yy;
//            integral_image[i][j].itg_yz += integral_image[i][j-1].itg_yz;
//            integral_image[i][j].itg_zz += integral_image[i][j-1].itg_zz;
//            integral_image[i][j].itg_num += integral_image[i][j-1].itg_num;
//        }
//    }
//    //// add through row
//    for (int j=1; j<COL+1; j++){
//        for (int i=2; i< ROW+1; i++){
//            integral_image[i][j].itg_x += integral_image[i-1][j].itg_x;
//            integral_image[i][j].itg_y += integral_image[i-1][j].itg_y;
//            integral_image[i][j].itg_z += integral_image[i-1][j].itg_z;
//            integral_image[i][j].itg_xx += integral_image[i-1][j].itg_xx;
//            integral_image[i][j].itg_xy += integral_image[i-1][j].itg_xy;
//            integral_image[i][j].itg_xz += integral_image[i-1][j].itg_xz;
//            integral_image[i][j].itg_yy += integral_image[i-1][j].itg_yy;
//            integral_image[i][j].itg_yz += integral_image[i-1][j].itg_yz;
//            integral_image[i][j].itg_zz += integral_image[i-1][j].itg_zz;
//            integral_image[i][j].itg_num += integral_image[i-1][j].itg_num;
//        }
//    }
//}

////  integral image with 9 vector
void image::create_integral_image(const std::vector < std::vector < spherical_point > > &depth_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr vertical_cloud){
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

int check_horizontal(int &left_col, int &right_col, const std::vector < std::vector < spherical_point > > &depth_image, int row,int central_col,int start_col,int end_col, float reference_depth){
    int found_valid_element =0;
    left_col = -1;
    right_col = -1;
    int index = depth_image[row][central_col].index;
    if(index >=0){
        if(fabs(depth_image[row][central_col].r -reference_depth ) < NORMAL_DEPTH_THRESHOLD){
            left_col ++;
            right_col ++;
            found_valid_element=1;
        }else{
            return found_valid_element;
        }
    }else{
        left_col ++;
        right_col ++;
//        found_valid_element=1;
    }
    for (int j= central_col -1; j >= start_col; j--){
        if(depth_image[row][j].index <0){
            left_col ++;
        }else if (fabs(depth_image[row][j].r - reference_depth) < NORMAL_DEPTH_THRESHOLD ){
            left_col ++;
            found_valid_element =1;
        }else{
            break;
        }
    }

    for (int j=central_col+1; j<=end_col; j++){
        if(depth_image[row][j].index <0){
            right_col ++;
        }else if(fabs(depth_image[row][j].r - reference_depth < NORMAL_DEPTH_THRESHOLD)){
            right_col ++;
            found_valid_element =1;
        }else{
            break;
        }
    }
    return found_valid_element;
}
int check_vertical(int &up_row, int &bottom_row, const std::vector < std::vector < spherical_point > > &depth_image, int central_row,int col,int start_row,int end_row, float reference_depth){
    up_row = -1;
    bottom_row =-1;
    int found_valid_element =0;
    int index = depth_image[central_row][col].index;
    if(index >= 0){
        if(fabs(depth_image[central_row][col].r - reference_depth)<NORMAL_DEPTH_THRESHOLD){
            up_row ++;
            bottom_row ++;
            found_valid_element =1 ;
        }else{
            return found_valid_element;
        }
    }else{
        up_row ++;
        bottom_row ++;
//        found_valid_element=1;
    }

    for (int i = central_row-1 ; i>=start_row; i--){
        if(depth_image[i][col].index < 0){
            up_row ++;
        }else if(fabs(depth_image[i][col].r - reference_depth)<NORMAL_DEPTH_THRESHOLD){
            up_row ++;
            found_valid_element =1;
        }else{
            break;
        }
    }
    for (int i = central_row+1 ; i <= end_row; i++){
        if(depth_image[i][col].index < 0){
            bottom_row ++;
        }else if(fabs(depth_image[i][col].r - reference_depth)<NORMAL_DEPTH_THRESHOLD){
            bottom_row ++;
            found_valid_element =1;
        }else{
            break;
        }
    }
    return found_valid_element;
}

int imin(int a, int b){
    if (a<0 && b<0){
        return 0;
    }else if (a<0 && b>= 0){
        return b;
    }else if (a>=0 && b<0){
        return a;
    }else if(a<b){
        return a;
    }else {
        return b;
    }

}
void image::create_interval_image(const std::vector < std::vector < spherical_point > > &depth_image) {

    for(int i=0; i<ROW; i++){
        for (int j=0; j<COL; j++){
            int index = depth_image[i][j].index;
            if(index <0){
                continue;
            }
            float reference_depth = depth_image[i][j].r;
            int keep_going =1;
            int rows_crown_radius = 1;
            int cols_crown_radius =1;
            for (int k=0; k<INTERVAL_CIRCULAR_LEVELS && keep_going >0; k++){
                keep_going =0;
                int start_row = i -rows_crown_radius;
                if (start_row < 0){
                    start_row =0;
                }
                int start_col = j - cols_crown_radius;
                if (start_col < 0){
                    start_col =0;
                }
                int end_row = i + rows_crown_radius;
                if (end_row >= ROW ){
                    end_row =ROW-1;
                }
                int end_col = j + cols_crown_radius;
                if (end_col >= COL ){
                    end_col =COL-1;
                }
                int up_left_col, up_right_col;
                keep_going+= check_horizontal(up_left_col, up_right_col, depth_image, start_row,j,start_col, end_col,reference_depth);
                int bottom_left_col,bottom_right_col;
                keep_going += check_horizontal(bottom_left_col, bottom_right_col, depth_image, end_row,j,start_col, end_col,reference_depth);
                int left_up_row, left_bottom_row;
                keep_going+= check_vertical(left_up_row, left_bottom_row, depth_image, i, start_col, start_row, end_row,reference_depth);
                int right_up_row, right_bottom_row;
                keep_going+= check_vertical(right_up_row, right_bottom_row, depth_image, i, end_col, start_row, end_row,reference_depth);

                interval_image[i][j].up_row = i-imin(left_up_row,right_up_row);
                interval_image[i][j].bottom_row = i+imin(left_bottom_row,right_bottom_row);
                interval_image[i][j].left_col = j-imin(up_left_col,bottom_left_col);
                interval_image[i][j].right_col = j + imin(up_left_col,up_right_col);
                rows_crown_radius ++;
                cols_crown_radius ++;
            }
        }
    }
}
pcl::PointCloud<pcl::Normal>::Ptr image::get_normal(std::vector < std::vector < spherical_point > > &depth_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr vertical_cloud) {
//    clock_t begin, end;
//    begin = clock();
//    std::cout << "star_normal" << std:: endl;
    int left_col;
    int right_col;
    int up_row;
    int bottom_row;
    int num;
    interval_point checked;

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
//                if (num > 2 && right_col - left_col > 0 && bottom_row-up_row >0) {
                if (num > 4 && right_col - left_col > 0 && bottom_row-up_row >0 ) {

                    cc(0, 0) = itg_xx[bottom_row][right_col] - itg_xx[up_row - 1][right_col] -
                               itg_xx[bottom_row][left_col - 1] + itg_xx[up_row - 1][left_col - 1];

                    cc(0, 1) = itg_xy[bottom_row][right_col] - itg_xy[up_row - 1][right_col] -
                               itg_xy[bottom_row][left_col - 1] + itg_xy[up_row - 1][left_col - 1];

                    cc(0, 2) = itg_xz[bottom_row][right_col] - itg_xz[up_row - 1][right_col] -
                               itg_xz[bottom_row][left_col - 1] + itg_xz[up_row - 1][left_col - 1];

                    cc(1, 1)  = itg_yy[bottom_row][right_col] - itg_yy[up_row - 1][right_col] -
                                itg_yy[bottom_row][left_col - 1] + itg_yy[up_row - 1][left_col - 1];

                    cc(1, 2)  = itg_yz[bottom_row][right_col] - itg_yz[up_row - 1][right_col] -
                                itg_yz[bottom_row][left_col - 1] + itg_yz[up_row - 1][left_col - 1];


                    cc(2, 2) = itg_zz[bottom_row][right_col] - itg_zz[up_row - 1][right_col] -
                               itg_zz[bottom_row][left_col - 1] + itg_zz[up_row - 1][left_col - 1];
                    cc(1, 0) = cc(0,1);
                    cc(2, 0) = cc(0,2);
                    cc(2, 1) = cc(1,2);

                    c(0) = itg_x[bottom_row][right_col] - itg_x[up_row - 1][right_col] -
                           itg_x[bottom_row][left_col - 1] + itg_x[up_row - 1][left_col - 1];

                    c(1) = itg_y[bottom_row][right_col] - itg_y[up_row - 1][right_col] -
                           itg_y[bottom_row][left_col - 1] + itg_y[up_row - 1][left_col - 1];

                    c(2) = itg_z[bottom_row][right_col] - itg_z[up_row - 1][right_col] -
                           itg_z[bottom_row][left_col - 1] + itg_z[up_row - 1][left_col - 1];

                    cov_matrix = cc ;
                    cov_matrix -= c * c.transpose() / num;
                    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> s(cov_matrix);
                    point_normal.normal_x = s.eigenvectors().col(0)[0];
                    point_normal.normal_y = s.eigenvectors().col(0)[1];
                    point_normal.normal_z = s.eigenvectors().col(0)[2];

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
//    end = clock();

////    std::cout << "end_normal" << std:: endl;
////    std::cout<<"수행시간 : "<<((end-begin))<<std::endl;
    return normals;

}