#include "image.h"

image::image(int row, int col){
    integral_image = std::vector<std::vector<integral_point> > (row+1, std::vector < integral_point> (col+1));
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
                integral_image[i+1][j+1].itg_x = vertical_cloud->points[index].x;
                integral_image[i+1][j+1].itg_y = vertical_cloud->points[index].y;
                integral_image[i+1][j+1].itg_z = vertical_cloud->points[index].z;
                integral_image[i+1][j+1].itg_xx = vertical_cloud->points[index].x * vertical_cloud->points[index].x;
                integral_image[i+1][j+1].itg_xy = vertical_cloud->points[index].x * vertical_cloud->points[index].y;
                integral_image[i+1][j+1].itg_xz = vertical_cloud->points[index].x * vertical_cloud->points[index].z;
                integral_image[i+1][j+1].itg_yy = vertical_cloud->points[index].y * vertical_cloud->points[index].y;
                integral_image[i+1][j+1].itg_yz = vertical_cloud->points[index].y * vertical_cloud->points[index].z;
                integral_image[i+1][j+1].itg_zz = vertical_cloud->points[index].z * vertical_cloud->points[index].z;
                integral_image[i+1][j+1].itg_num =1;
            }
        }
    }
    //// add through col
    for (int i=1; i<ROW+1; i++){
        for (int j=2; j< COL+1; j++){
            integral_image[i][j].itg_x += integral_image[i][j-1].itg_x;
            integral_image[i][j].itg_y += integral_image[i][j-1].itg_y;
            integral_image[i][j].itg_z += integral_image[i][j-1].itg_z;
            integral_image[i][j].itg_xx += integral_image[i][j-1].itg_xx;
            integral_image[i][j].itg_xy += integral_image[i][j-1].itg_xy;
            integral_image[i][j].itg_xz += integral_image[i][j-1].itg_xz;
            integral_image[i][j].itg_yy += integral_image[i][j-1].itg_yy;
            integral_image[i][j].itg_yz += integral_image[i][j-1].itg_yz;
            integral_image[i][j].itg_zz += integral_image[i][j-1].itg_zz;
            integral_image[i][j].itg_num += integral_image[i][j-1].itg_num;
        }
    }
    //// add through row
    for (int j=1; j<COL+1; j++){
        for (int i=2; i< ROW+1; i++){
            integral_image[i][j].itg_x += integral_image[i-1][j].itg_x;
            integral_image[i][j].itg_y += integral_image[i-1][j].itg_y;
            integral_image[i][j].itg_z += integral_image[i-1][j].itg_z;
            integral_image[i][j].itg_xx += integral_image[i-1][j].itg_xx;
            integral_image[i][j].itg_xy += integral_image[i-1][j].itg_xy;
            integral_image[i][j].itg_xz += integral_image[i-1][j].itg_xz;
            integral_image[i][j].itg_yy += integral_image[i-1][j].itg_yy;
            integral_image[i][j].itg_yz += integral_image[i-1][j].itg_yz;
            integral_image[i][j].itg_zz += integral_image[i-1][j].itg_zz;
            integral_image[i][j].itg_num += integral_image[i-1][j].itg_num;
        }
    }
}

//interval_point check_horizontal(int i, int j, const spherical_point (&depth_image)[ROW][COL], int boundary_col, float input_depth){
//    interval_point output;
//    int left_col=j-1;
//    int right_col=j+1;
//    int up_row=i;
//    int bottom_row=i;
//    int checker;
//    int double_checker;
//    float depth;
//    float pre_depth;
//    ///// right column check
//    checker=0;
//    double_checker=0;
//    pre_depth = input_depth;
//    while (right_col < COL && checker < boundary_col && double_checker < 10){
//        if(depth_image[i][right_col].index == -1){
//            right_col ++;
//            double_checker++;
//        }
//        else{
//            depth = depth_image[i][right_col].r;
//            if(fabs(depth-pre_depth)< DEPTH_THRESHOLD){
//                checker++;
//                double_checker++;
//                right_col++;
//                pre_depth = depth;
//            }else{
//                right_col --;
//                break;
//            }
//        }
//    }
//    if (right_col > COL-1){
//        right_col = COL-1;
//    }
////// left col check
//    checker =0;
//    double_checker=0;
//    pre_depth = input_depth;
//    while (left_col >= 0 && checker < boundary_col && double_checker < 10){
//        if(depth_image[i][left_col].index == -1){
//            left_col --;
//            double_checker++;
//        }
//        else{
//            depth = depth_image[i][left_col].r;
//            if(fabs(depth-pre_depth)< DEPTH_THRESHOLD){
//                checker++;
//                double_checker++;
//                left_col--;
//                pre_depth = depth;
//            }else{
//                left_col ++;
//
//                break;
//            }
//        }
//    }
//    if (left_col < 0){
//        left_col = 0;
//    }
//    output.left_col = left_col;
//    output.right_col = right_col;
//    output.up_row = up_row;
//    output.bottom_row = bottom_row;
//    return output;
//}
//interval_point check_vertical(int i, int j, const spherical_point (&depth_image)[ROW][COL], int boundary_row){
//    interval_point output;
//    int left_col=j;
//    int right_col=j;
//    int up_row=i-1;
//    int bottom_row=i+1;
//
//    int ver_checker;
//    float ver_pre_depth;
//    float ver_depth;
//    int double_checker=0;
//    ver_checker =0;
//    ver_pre_depth = depth_image[i][j].r;
//    //// up row check
//    while (up_row >= 0 && ver_checker < boundary_row && double_checker < 10){
//        if(depth_image[up_row][j].index == -1){
//            up_row --;
//            double_checker ++;
//        }else{
//            ver_depth = depth_image[up_row][j].r;
//            if(fabs(ver_depth-ver_pre_depth) < DEPTH_THRESHOLD){
//                ver_checker++;
//                up_row --;
//                double_checker ++;
//                ver_pre_depth = ver_depth;
//            }else{
//                up_row ++;
//                break;
//            }
//
//        }
//    }
//    if (up_row <0){
//        up_row = 0;
//    }
//    //// down row check
//    ver_checker =0;
//    double_checker=0;
//    ver_pre_depth = depth_image[i][j].r;
//    while (bottom_row < ROW && ver_checker < boundary_row && double_checker < 10){
//        if(depth_image[bottom_row][j].index == -1){
//            bottom_row ++;
//            double_checker ++;
//        }else{
//            ver_depth = depth_image[bottom_row][j].r;
//            if(fabs(ver_depth-ver_pre_depth) < DEPTH_THRESHOLD){
//                ver_checker++;
//                bottom_row ++;
//                double_checker ++;
//                ver_pre_depth = ver_depth;
//            }else{
//                bottom_row --;
//                break;
//            }
//        }
//    }
//    if (bottom_row >= ROW){
//        bottom_row = ROW-1;
//    }
//
//    output.left_col = left_col;
//    output.right_col = right_col;
//    output.up_row = up_row;
//    output.bottom_row = bottom_row;
//    return output;
//}

int check_horizontal(int &left_col, int &right_col, const spherical_point (&depth_image)[ROW][COL], int row,int central_col,int start_col,int end_col, float reference_depth){
    int found_valid_element =0;
    left_col = -1;
    right_col = -1;
    int index = depth_image[row][central_col].index;
    if(index >=0){
        if(fabs(depth_image[row][central_col].r -reference_depth ) < DEPTH_THRESHOLD){
            left_col ++;
            right_col ++;
            found_valid_element=1;
        }else{
            return found_valid_element;
        }
    }else{
        left_col ++;
        right_col ++;
        found_valid_element=1;
    }
    for (int j= central_col -1; j >= start_col; j--){
        if(depth_image[row][j].index <0){
            left_col ++;
        }else if (fabs(depth_image[row][j].r - reference_depth) < DEPTH_THRESHOLD ){
            left_col ++;
            found_valid_element =1;
        }else{
            break;
        }
    }

    for (int j=central_col+1; j<=end_col; j++){
        if(depth_image[row][j].index <0){
            right_col ++;
        }else if(fabs(depth_image[row][j].r - reference_depth < DEPTH_THRESHOLD)){
            right_col ++;
            found_valid_element =1;
        }else{
            break;
        }
    }
    return found_valid_element;
}
int check_vertical(int &up_row, int &bottom_row, const spherical_point (&depth_image)[ROW][COL], int central_row,int col,int start_row,int end_row, float reference_depth){
    up_row = -1;
    bottom_row =-1;
    int found_valid_element =0;
    int index = depth_image[central_row][col].index;
    if(index >= 0){
        if(fabs(depth_image[central_row][col].r - reference_depth)<DEPTH_THRESHOLD){
            up_row ++;
            bottom_row ++;
            found_valid_element =1 ;
        }else{
            return found_valid_element;
        }
    }else{
        up_row ++;
        bottom_row ++;
        found_valid_element=1;
    }

    for (int i = central_row-1 ; i>=start_row; i--){
        if(depth_image[i][col].index < 0){
            up_row ++;
        }else if(fabs(depth_image[i][col].r - reference_depth)<DEPTH_THRESHOLD){
            up_row ++;
            found_valid_element =1;
        }else{
            break;
        }
    }
    for (int i = central_row+1 ; i <= end_row; i++){
        if(depth_image[i][col].index < 0){
            bottom_row ++;
        }else if(fabs(depth_image[i][col].r - reference_depth)<DEPTH_THRESHOLD){
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
void image::create_interval_image(const spherical_point (&depth_image)[ROW][COL]) {

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
//    for (int i=0; i < 10; i++){
//        for (int j=0; j<10; j++){
//            if(depth_image[i][j].index < 0){
//                continue;
//            }
//            int num=0;
//
//            for (int ii=interval_image[i][j].up_row; ii <= interval_image[i][j].bottom_row; ii++){
//                for (int jj=interval_image[i][j].left_col; jj <= interval_image[i][j].right_col; jj++){
//                    if(depth_image[ii][jj].index != -1){
//                        num++;
//                    }
//                }
//            }
//            std::cout << i << ' ' << interval_image[i][j].up_row<< ' ' << interval_image[i][j].bottom_row << std::endl;
//            std::cout << j << ' ' << interval_image[i][j].left_col<< ' ' << interval_image[i][j].right_col << std::endl;
//            std::cout << num << std::endl;
//
//        }
//    }


}
//void image::create_interval_image(const spherical_point (&depth_image)[ROW][COL]) {
//    interval_point ver;
//    interval_point hor;
//    interval_point input_interval_point;
//    float input_depth;
//    float pre_depth;
//    int left_col;
//    int right_col;
//    int up_row;
//    int bottom_row;
//
//    for( int i=0 ; i< ROW; i ++){
//        for (int j=0; j<COL; j++ ){
//            if(depth_image[i][j].index != -1){
//                left_col=0;
//                right_col =4999;
//                pre_depth=depth_image[i][j].r;
//                ver = check_vertical(i,j,depth_image,boundary_row);
//                up_row = ver.up_row;
//                bottom_row = ver.bottom_row;
//                for (int ii = i; ii >= up_row; ii--){
//                    if (depth_image[ii][j].index == -1){
//                        input_depth = pre_depth;
//                    }else{
//                        input_depth = depth_image[ii][j].r;
//                        pre_depth = input_depth;
//                    }
//                    hor = check_horizontal(ii,j,depth_image,boundary_col,input_depth);
//                    if(left_col < hor.left_col){
//                        left_col = hor.left_col;
//                    }
//                    if(right_col > hor.left_col){
//                        right_col = hor.right_col;
//                    }
//                }
//                for (int ii = i; ii <= bottom_row; ii++){
//                    if (depth_image[ii][j].index == -1){
//                        input_depth = pre_depth;
//                    }else{
//                        input_depth = depth_image[ii][j].r;
//                        pre_depth = input_depth;
//                    }
//                    hor = check_horizontal(ii,j,depth_image,boundary_col,input_depth);
//                    if(left_col < hor.left_col){
//                        left_col = hor.left_col;
//                    }
//                    if(right_col > hor.right_col){
//                        right_col = hor.right_col;
//                    }
//                }
//                input_interval_point.up_row = up_row;
//                input_interval_point.bottom_row = bottom_row;
//                input_interval_point.left_col = left_col;
//                input_interval_point.right_col = right_col;
//                interval_image[i][j] = input_interval_point;
//
////                std::cout << interval_image[i][j].right_col << std::endl;
////                std::cout << interval_image[i][j].left_col << std::endl;
////                std::cout <<  interval_image[i][j].up_row<<std::endl;
////                std::cout<<   interval_image[i][j].bottom_row<<std::endl;
////                std::cout << 111 << std::endl;
//            }
//        }
//    }
//
//}

pcl::PointCloud<pcl::Normal>::Ptr image::get_normal(spherical_point (&depth_image)[ROW][COL], pcl::PointCloud<pcl::PointXYZRGB>::Ptr vertical_cloud) {
    clock_t begin, end;
    begin = clock();
    std::cout << "star_normal" << std:: endl;
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
                num = integral_image[bottom_row][right_col].itg_num - integral_image[up_row - 1][right_col].itg_num -
                      integral_image[bottom_row][left_col - 1].itg_num + integral_image[up_row - 1][left_col - 1].itg_num;
//                if (num > 2 && right_col - left_col > 0 && bottom_row-up_row >0) {
                if (num > 4 ) {

                    cc(0, 0) = integral_image[bottom_row][right_col].itg_xx - integral_image[up_row - 1][right_col].itg_xx -
                          integral_image[bottom_row][left_col - 1].itg_xx + integral_image[up_row - 1][left_col - 1].itg_xx;

                    cc(0, 1) = integral_image[bottom_row][right_col].itg_xy - integral_image[up_row - 1][right_col].itg_xy -
                          integral_image[bottom_row][left_col - 1].itg_xy + integral_image[up_row - 1][left_col - 1].itg_xy;

                    cc(0, 2) = integral_image[bottom_row][right_col].itg_xz - integral_image[up_row - 1][right_col].itg_xz -
                          integral_image[bottom_row][left_col - 1].itg_xz + integral_image[up_row - 1][left_col - 1].itg_xz;


                    cc(1, 1)  = integral_image[bottom_row][right_col].itg_yy - integral_image[up_row - 1][right_col].itg_yy -
                          integral_image[bottom_row][left_col - 1].itg_yy + integral_image[up_row - 1][left_col - 1].itg_yy;

                    cc(1, 2)  = integral_image[bottom_row][right_col].itg_yz - integral_image[up_row - 1][right_col].itg_yz -
                          integral_image[bottom_row][left_col - 1].itg_yz + integral_image[up_row - 1][left_col - 1].itg_yz;


                    cc(2, 2) = integral_image[bottom_row][right_col].itg_zz - integral_image[up_row - 1][right_col].itg_zz -
                          integral_image[bottom_row][left_col - 1].itg_zz + integral_image[up_row - 1][left_col - 1].itg_zz;
                    cc(1, 0) = cc(0,1);
                    cc(2, 0) = cc(0,2);
                    cc(2, 1) = cc(1,2);

                    c(0) = integral_image[bottom_row][right_col].itg_x - integral_image[up_row - 1][right_col].itg_x -
                         integral_image[bottom_row][left_col - 1].itg_x + integral_image[up_row - 1][left_col - 1].itg_x;

                    c(1) = integral_image[bottom_row][right_col].itg_y - integral_image[up_row - 1][right_col].itg_y -
                         integral_image[bottom_row][left_col - 1].itg_y + integral_image[up_row - 1][left_col - 1].itg_y;

                    c(2) = integral_image[bottom_row][right_col].itg_z - integral_image[up_row - 1][right_col].itg_z -
                         integral_image[bottom_row][left_col - 1].itg_z + integral_image[up_row - 1][left_col - 1].itg_z;

                    cov_matrix = cc ;
                    cov_matrix -= c * c.transpose() / num;

                    Eigen::EigenSolver<Eigen::Matrix3f> s(cov_matrix);

//
//                    eigen1 = s.eigenvalues().col(0)[0].real();
//                    eigen2 = s.eigenvalues().col(0)[1].real();
//                    eigen3 = s.eigenvalues().col(0)[2].real();
                    eigen1 = fabs(s.eigenvalues().col(0)[0].real());
                    eigen2 = fabs(s.eigenvalues().col(0)[1].real());
                    eigen3 = fabs(s.eigenvalues().col(0)[2].real());
                    if (eigen1 < eigen2 && eigen1 < eigen3) {
                        point_normal.normal_x=s.eigenvectors().col(0)[0].real();
                        point_normal.normal_y = s.eigenvectors().col(0)[1].real();
                        point_normal.normal_z = s.eigenvectors().col(0)[2].real();

                    } else if (eigen2 < eigen1 && eigen2 < eigen3) {
                        point_normal.normal_x = s.eigenvectors().col(1)[0].real();
                        point_normal.normal_y = s.eigenvectors().col(1)[1].real();
                        point_normal.normal_z = s.eigenvectors().col(1)[2].real();

                    } else if (eigen3 < eigen1 && eigen3 < eigen2){
                        point_normal.normal_x = s.eigenvectors().col(2)[0].real();
                        point_normal.normal_y = s.eigenvectors().col(2)[1].real();
                        point_normal.normal_z = s.eigenvectors().col(2)[2].real();
                    }else {
                        point_normal.normal_x =0;
                        point_normal.normal_y =0;
                        point_normal.normal_z=0;
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
    end = clock();
    std::cout << "end_normal" << std:: endl;
    std::cout<<"수행시간 : "<<((end-begin))<<std::endl;
    return normals;

}