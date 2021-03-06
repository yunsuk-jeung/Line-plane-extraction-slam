#include "loader.h"


loader::loader(){
    cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    vertical_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    vertical_cloud2 = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    normal_cloud = pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);
    depth_image = std::vector < std::vector < spherical_point > > (ROW, std::vector < spherical_point > (COL));
}

void loader::txt2pcl(const std::string &fileName, const int &num) {
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
}

void loader::csv2pcl(const std::string &fileName, const int &num) {
//    std::cout << "csv readed" << std::endl;
    pcl::PointXYZRGB temp;
    std:: ifstream file(fileName.c_str());
    std::string line, val;
    for (int i=0; i< num+1; i++){
        std::getline(file,line);
    }

    std::stringstream ss(line);
    std::getline(ss, val, ',');
    std::getline(ss, val, ',');
        while(std::getline(ss,val,',')){
            temp.x = atof(val.c_str());
            std::getline(ss, val, ',');
            temp.y = atof(val.c_str());
            std::getline(ss, val, ',');
            temp.z = atof(val.c_str());
            temp.r=255;
            temp.g=255;
            temp.b=255;
            cloud->push_back(temp);
        }
//    pcl::visualization::CloudViewer viewer("Cloud Viewer");
//
//
//    viewer.showCloud(cloud);
//
//    while (!viewer.wasStopped ())
//    {
//    }
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
//    std::cout << "depth_image created" << std::endl;
    spherical_point temp;
    int size = cloud->points.size();
    float col_resolution = 2 * M_PI / COL;
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
    float row_resolution = (max_theta + 0.0001 - min_theta) / ROW;
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
//    std::cout << "flat region removed" << std::endl;
    clock_t begin, end;
    float a;
    float b;
    float epsilone;
    int height = ROW;
    int length = COL;
    int num;
    int num_vertical;
    float close_area = 10;
    std::vector<int> temp_vertical_checker;
    spherical_point zero_point;
    for (int i=0; i< length; i++) {
        int vertical_checker[ROW]={0,};
        for (int j =0; j<height; j++){
            if (vertical_checker[j] == 0){
                if(depth_image[j][i].index != -1 ) {
                    num = 0;
                    a = depth_image[j][i].r * sin(depth_image[j][i].theta);

                    if (a < close_area) {
                        epsilone = CLOSE_FLAT_REGION_RANGE;
                        num_vertical = CLOSE_VERTICAL_THRESHOLD;
                    } else {
                        epsilone = FAR_FLAT_REGION_RANGE;
                        num_vertical = FAR_VERTICAL_THRESHOLD;
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
//    std::cout << "start creating image" << std::endl;
    image integral(ROW+1,COL+1);

    integral.create_integral_image(depth_image, vertical_cloud);
    integral.create_interval_image(depth_image);
    normal_cloud=integral.get_normal(depth_image,vertical_cloud);
//    std::cout << normal_cloud->points.size() << std::endl;
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

}
void loader::viewer2() {
    pcl::visualization::PCLVisualizer viewer("pcl viewer");
    viewer.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(vertical_cloud2,normal_cloud,1,0.5);
    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }
}

//// compute distance between line & point
float line_to_point(float &origin_x,float &origin_y, float &origin_z,float &nx, float &ny, float &nz, float &x, float &y, float &z){
    float distance;
    float new_nx = x-origin_x;
    float new_ny = y-origin_y;
    float new_nz = z-origin_z;
    distance = pow(nx * (ny * new_nz - nz * new_ny),2) + pow (ny * (nz* new_nx - nx * new_nz),2) + pow(nz * (nx * new_ny - ny* new_nx),2);
    return distance;
}
//// compute distance between plane & point
float plane_to_point(float &origin_x,float &origin_y, float &origin_z,float &nx, float &ny, float &nz, float &x, float &y, float &z){
    float distance;
    float new_nx = x-origin_x;
    float new_ny = y-origin_y;
    float new_nz = z-origin_z;
    distance = fabs(new_nx * nx + new_ny * ny + new_nz * nz);
    return distance;
}

void loader::clusterizer(std::vector < feature_point > &Line, std::vector <  feature_point  > &Plane, std::vector < std::vector <float > > &Line_points, std::vector < std::vector <float> > &Plane_points, std::vector < std::vector <float> > &Line_every_points, std::vector < std::vector <float> > &Plane_every_points){

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Normal>::Ptr temp_normal (new pcl::PointCloud<pcl::Normal>);
    pcl::PointXYZRGB temp_point;
    pcl::Normal temp_normal_point;
    std::vector < std::vector < int > > visit(ROW, std::vector <int> (COL));

    std::vector < int > visit_row;
    std::vector < int > visit_col;
    float reference_depth;
    float depth;
    int surface=0;
    int max_surface=0;

    float x1;
    float x2;
    float y1;
    float y2;
    float z1;
    float z2;
    float nx1;
    float ny1;
    float nz1;
    float nx2;
    float ny2;
    float nz2;
//    std::cout << "start clusterize" << std::endl;
    //cluster
    for (int i = 0; i< ROW; i++){
        for (int j=0; j<COL; j++){
            if (depth_image[i][j].index != -1){
                x1= vertical_cloud2->points[depth_image[i][j].index].x;
                y1 = vertical_cloud2->points[depth_image[i][j].index].y;
                z1 = vertical_cloud2->points[depth_image[i][j].index].z;
                nx1 = normal_cloud->points[depth_image[i][j].index].normal_x;
                ny1 = normal_cloud->points[depth_image[i][j].index].normal_y;
                nz1 = normal_cloud->points[depth_image[i][j].index].normal_z;
                if(visit[i][j] == 0){
                    max_surface++;
                    surface = max_surface;
                } else{
                    surface = visit[i][j];
//                    continue;
                }
                int row_found_valid;
                int col_found_valid;

                row_found_valid=0;
                int ii = i-CLUSTER_NEIGHBOR_ROW;
                while (ii < i+CLUSTER_NEIGHBOR_ROW && row_found_valid < CLUSTER_NEIGHBOR_ROW_VALID){
                    int jj = j -CLUSTER_NEIGHBOR_COL;
                    col_found_valid=0;
                    while (jj < j+CLUSTER_NEIGHBOR_COL && col_found_valid < CLUSTER_NEIGHBOR_COL_VALID){
                        if (ii < 0 || ii >=ROW || jj < 0 || jj >= COL ) {
                            jj++;
                            continue;
                        }
                        if(depth_image[ii][jj].index <0){
                            jj++;
                            continue;
                        }else{
                            col_found_valid++;
                        }
                        if(visit[ii][jj] != 0){
                            jj++;
                            continue;
                        }
                        x2 = vertical_cloud2->points[depth_image[ii][jj].index].x;
                        y2 = vertical_cloud2->points[depth_image[ii][jj].index].y;
                        z2 = vertical_cloud2->points[depth_image[ii][jj].index].z;
                        if ( +pow(x2-x1,2)+pow(y2-y1,2)+pow(z2-z1,2)< CLUSTER_NEIGHBOR_DISTANCE){
                            nx2 = normal_cloud->points[depth_image[ii][jj].index].normal_x;
                            ny2 = normal_cloud->points[depth_image[ii][jj].index].normal_y;
                            nz2 = normal_cloud->points[depth_image[ii][jj].index].normal_z;
                            if (nx1 * nx2 + ny1 * ny2 + nz1 * nz2 > CLUSTER_NEIGHBOR_ANGLE  ){
//                                    if(visit[ii][jj] !=0 && visit[i][j] != visit[ii][jj]){
//                                        if(surface == max_surface){
//                                            max_surface --;
//                                        }
//                                        surface = visit[ii][jj];
//                                    }
                                    visit_row.push_back(ii);
                                    visit_col.push_back(jj);
                            }
                        }
                        if(col_found_valid >0){
                            row_found_valid++;
                        }
                        jj++;
                    }
                    ii++;
                }
                for(int k=0; k < visit_row.size(); k++){
                    visit[visit_row[k]][visit_col[k]] = surface;
                }
                visit_row.clear();
                visit_col.clear();
            }
        }
    }

//    std::cout << max_surface << std::endl;
    std::vector < std::vector < int > > cloud_index(max_surface+1);

    for(int i=0; i<ROW; i++) {
        for (int j = 0; j < COL; j++) {
            if (depth_image[i][j].index < 0) {

                continue;
            }

            if (visit[i][j] ==0 ){
                continue;
            }
            surface = visit[i][j];
            cloud_index[surface - 1].push_back(depth_image[i][j].index);
        }
    }
//
//    std::cout << "color check" << std::endl;
//
//    int color_line =0;
//    for(int i=0; i< cloud_index.size(); i++){
//        if (cloud_index[i].size() > 0){
//            if (color_line % 5 == 0 ){
//                for (int j=0; j< cloud_index[i].size(); j++){
//                    temp_point.x = vertical_cloud2->points[cloud_index[i][j]].x;
//                    temp_point.y = vertical_cloud2->points[cloud_index[i][j]].y;
//                    temp_point.z = vertical_cloud2->points[cloud_index[i][j]].z;
//                    temp_point.r = 255;
//                    temp_point.g = 0;
//                    temp_point.b = 0;
//                    temp->points.push_back(temp_point);
////                    temp_normal_point.normal_x = normal_cloud->points[cloud_index[i][j]].normal_x;
////                    temp_normal_point.normal_y = normal_cloud->points[cloud_index[i][j]].normal_y;
////                    temp_normal_point.normal_z = normal_cloud->points[cloud_index[i][j]].normal_z;
////                    temp_normal->points.push_back(temp_normal_point);
//                }
//                color_line ++;
//            }else if(color_line % 5 ==1) {
//                for (int j = 0; j < cloud_index[i].size(); j++) {
//                    temp_point.x = vertical_cloud2->points[cloud_index[i][j]].x;
//                    temp_point.y = vertical_cloud2->points[cloud_index[i][j]].y;
//                    temp_point.z = vertical_cloud2->points[cloud_index[i][j]].z;
//                    temp_point.r = 0;
//                    temp_point.g = 255;
//                    temp_point.b = 0;
//                    temp->points.push_back(temp_point);
////                    temp_normal_point.normal_x = normal_cloud->points[cloud_index[i][j]].normal_x;
////                    temp_normal_point.normal_y = normal_cloud->points[cloud_index[i][j]].normal_y;
////                    temp_normal_point.normal_z = normal_cloud->points[cloud_index[i][j]].normal_z;
////                    temp_normal->points.push_back(temp_normal_point);
//                }
//                color_line ++;
//            }else if (color_line % 5 == 2 ){
//                for (int j=0; j< cloud_index[i].size(); j++){
//                    temp_point.x = vertical_cloud2->points[cloud_index[i][j]].x;
//                    temp_point.y = vertical_cloud2->points[cloud_index[i][j]].y;
//                    temp_point.z = vertical_cloud2->points[cloud_index[i][j]].z;
//                    temp_point.r = 255;
//                    temp_point.g = 255;
//                    temp_point.b = 0;
//                    temp->points.push_back(temp_point);
////                        temp_normal_point.normal_x = normal_cloud->points[cloud_index[i][j]].normal_x;
////                        temp_normal_point.normal_y = normal_cloud->points[cloud_index[i][j]].normal_y;
////                        temp_normal_point.normal_z = normal_cloud->points[cloud_index[i][j]].normal_z;
////                        temp_normal->points.push_back(temp_normal_point);
//                }
//                color_line ++;
//            }else if(color_line % 5 ==3){
//                for (int j=0; j< cloud_index[i].size(); j++){
//                    temp_point.x = vertical_cloud2->points[cloud_index[i][j]].x;
//                    temp_point.y = vertical_cloud2->points[cloud_index[i][j]].y;
//                    temp_point.z = vertical_cloud2->points[cloud_index[i][j]].z;
//                    temp_point.r = 255;
//                    temp_point.g = 255;
//                    temp_point.b = 255;
//                    temp->points.push_back(temp_point);
////                    temp_normal_point.normal_x = normal_cloud->points[cloud_index[i][j]].normal_x;
////                    temp_normal_point.normal_y = normal_cloud->points[cloud_index[i][j]].normal_y;
////                    temp_normal_point.normal_z = normal_cloud->points[cloud_index[i][j]].normal_z;
////                    temp_normal->points.push_back(temp_normal_point);
//                }
//                color_line ++;
//            }else{
//                for (int j=0; j< cloud_index[i].size(); j++){
//                    temp_point.x = vertical_cloud2->points[cloud_index[i][j]].x;
//                    temp_point.y = vertical_cloud2->points[cloud_index[i][j]].y;
//                    temp_point.z = vertical_cloud2->points[cloud_index[i][j]].z;
//                    temp_point.r = 0;
//                    temp_point.g = 0;
//                    temp_point.b = 255;
//                    temp->points.push_back(temp_point);
////                    temp_normal_point.normal_x = normal_cloud->points[cloud_index[i][j]].normal_x;
////                    temp_normal_point.normal_y = normal_cloud->points[cloud_index[i][j]].normal_y;
////                    temp_normal_point.normal_z = normal_cloud->points[cloud_index[i][j]].normal_z;
////                    temp_normal->points.push_back(temp_normal_point);
//                }
//                color_line ++;
//            }
//        }
//
//    }

////find valid element
    float cx;
    float cy;
    float cz;
    float cxx;
    float cxy;
    float cxz;
    float cyy;
    float cyz;
    float czz;
    feature_point origin;

    std::vector < int> found_valid(cloud_index.size());
    Eigen::Matrix3f cc;
    Eigen::Vector3f c;
    Eigen::Matrix3f cov_matrix;

    for (int i=0; i< cloud_index.size(); i++){
        int num = cloud_index[i].size();

        if (num > 30){
            cx=0;
            cy=0;
            cz=0;
            cxx=0;
            cxy=0;
            cxz=0;
            cyy=0;
            cyz=0;
            czz=0;
            for (int j=0; j< num; j++){
                cx += vertical_cloud2->points[cloud_index[i][j]].x;
                cy += vertical_cloud2->points[cloud_index[i][j]].y;
                cz += vertical_cloud2->points[cloud_index[i][j]].z;
                cxx += vertical_cloud2->points[cloud_index[i][j]].x * vertical_cloud2->points[cloud_index[i][j]].x ;
                cxy += vertical_cloud2->points[cloud_index[i][j]].x * vertical_cloud2->points[cloud_index[i][j]].y ;
                cxz += vertical_cloud2->points[cloud_index[i][j]].x * vertical_cloud2->points[cloud_index[i][j]].z ;
                cyy += vertical_cloud2->points[cloud_index[i][j]].y * vertical_cloud2->points[cloud_index[i][j]].y ;
                cyz += vertical_cloud2->points[cloud_index[i][j]].y * vertical_cloud2->points[cloud_index[i][j]].z ;
                czz += vertical_cloud2->points[cloud_index[i][j]].z * vertical_cloud2->points[cloud_index[i][j]].z ;
            }
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

            float origin_x = cx/cloud_index[i].size();
            float origin_y = cy/cloud_index[i].size();
            float origin_z = cz/cloud_index[i].size();
            origin.origin_x = origin_x;
            origin.origin_y = origin_y;
            origin.origin_z = origin_z;


            cov_matrix = cc ;
            cov_matrix -= c * c.transpose() / cloud_index[i].size();

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> s(cov_matrix);
            float eigen1 = s.eigenvalues().col(0)[0];
            float eigen2 = s.eigenvalues().col(0)[1];
            float eigen3 = s.eigenvalues().col(0)[2];

            float error =0;

            if((eigen1+eigen2)/(eigen1+eigen2+eigen3) < LINE_EIGENVALUE_THRESHOLD){
                float nx = s.eigenvectors().col(2)[0];
                float ny = s.eigenvectors().col(2)[1];
                float nz = s.eigenvectors().col(2)[2];

                for (int j =0; j < num; j++){
                error += line_to_point(origin_x, origin_y, origin_z, nx, ny, nz, vertical_cloud2->points[cloud_index[i][j]].x,vertical_cloud2->points[cloud_index[i][j]].y, vertical_cloud2->points[cloud_index[i][j]].z);
                }
                error = error/num;
                if (error < 0.04){

                    found_valid[i] = 1;
                    origin.nx=nx;
                    origin.ny=ny;
                    origin.nz=nz;
                    Line.push_back(origin);
                    std::vector <float > points;
                    for (int j =0; j < num; j++){
                        points.push_back(vertical_cloud2->points[cloud_index[i][j]].x);
                        points.push_back(vertical_cloud2->points[cloud_index[i][j]].y);
                        points.push_back(vertical_cloud2->points[cloud_index[i][j]].z);
                    }
                    for ( int j=0; j<100; j++){
                        points.push_back(origin_x + nx * j/50);
                        points.push_back(origin_y + ny * j/50);
                        points.push_back(origin_z + nz * j/50);
                    }
                    Line_every_points.push_back(points);
                    points.clear();

//                    for (int w=0; w<100; w++){
//                        for(int h=0; h<100; h++){
//                            points.push_back(origin_x - (w)/50 * nx1 - (h)/50 * nx2);
//                            points.push_back(origin_y - (w)/50 * ny1 - (h)/50 * ny2 );
//                            points.push_back(origin_z - (w)/50 * nz1 - (h)/50 * nz2 );
//                        }
//                    }
                    points.push_back(origin.origin_x);
                    points.push_back(origin.origin_y);
                    points.push_back(origin.origin_z);
                    points.push_back(origin.origin_x + nx);
                    points.push_back(origin.origin_y + ny);
                    points.push_back(origin.origin_z + nz);
                    points.push_back(origin.origin_x - nx);
                    points.push_back(origin.origin_y - ny);
                    points.push_back(origin.origin_z - nz);
                    Line_points.push_back(points);
                    points.clear();

                }else{
                    if(eigen1/(eigen1+eigen2+eigen3) < PLANE_EIGENVALUE_THRESHOLD){
                        float nx = s.eigenvectors().col(0)[0];
                        float ny = s.eigenvectors().col(0)[1];
                        float nz = s.eigenvectors().col(0)[2];
                        error =0;
                        for (int j =0; j < num; j++){
                            error += plane_to_point(origin_x, origin_y, origin_z, nx, ny, nz, vertical_cloud2->points[cloud_index[i][j]].x,vertical_cloud2->points[cloud_index[i][j]].y, vertical_cloud2->points[cloud_index[i][j]].z);
                        }
                        error = error/num;
                        if(error < 0.1){
                            if(nx * origin_x + ny*origin_y + nz*origin_z >= 0){
                                nx = -nx;
                                ny = -ny;
                                nz = -nz;
                            }
                            found_valid[i] = 2;
                            float nx1 = s.eigenvectors().col(1)[0];
                            float ny1 = s.eigenvectors().col(1)[1];
                            float nz1 = s.eigenvectors().col(1)[2];
                            float nx2 = s.eigenvectors().col(2)[0];
                            float ny2 = s.eigenvectors().col(2)[1];
                            float nz2 = s.eigenvectors().col(2)[2];
                            origin.nx=nx;
                            origin.ny=ny;
                            origin.nz=nz;

                            Plane.push_back(origin);

                            std::vector <float > points;
                            for (int j =0; j < num; j++){
                                points.push_back(vertical_cloud2->points[cloud_index[i][j]].x);
                                points.push_back(vertical_cloud2->points[cloud_index[i][j]].y);
                                points.push_back(vertical_cloud2->points[cloud_index[i][j]].z);
                            }
                            for ( int j=0; j<100; j++){
                                points.push_back(origin_x + nx * j/50);
                                points.push_back(origin_y + ny * j/50);
                                points.push_back(origin_z + nz * j/50);
                            }
                            Plane_every_points.push_back(points);
                            points.clear();

//                            for (int w=0; w<100; w++){
//                                float h=0;
////                                for(int h=0; h<100; h++){
//                                    points.push_back(origin_x - nx1 * (w)/50 - (h)/50 * nx2);
//                                    points.push_back(origin_y - ny1 * (w)/50  - (h)/50 * ny2 );
//                                    points.push_back(origin_z - nz1 * (w)/50  - (h)/50 * nz2 );
////                                }
//                            }
                            points.push_back(origin.origin_x);
                            points.push_back(origin.origin_y);
                            points.push_back(origin.origin_z);
                            points.push_back(origin.origin_x + 2 * nx1);
                            points.push_back(origin.origin_y + 2 * ny1);
                            points.push_back(origin.origin_z + 2 * nz1);
                            points.push_back(origin.origin_x + 2 * nx2);
                            points.push_back(origin.origin_y + 2 * ny2);
                            points.push_back(origin.origin_z + 2 * nz2);
                            points.push_back(origin.origin_x - 2 * nx1);
                            points.push_back(origin.origin_y - 2 * ny1);
                            points.push_back(origin.origin_z - 2 * nz1);
                            points.push_back(origin.origin_x - 2 * nx2);
                            points.push_back(origin.origin_y - 2 * ny2);
                            points.push_back(origin.origin_z - 2 * nz2);
                            Plane_points.push_back(points);
                            points.clear();
                        }
                    }
                }
            }else{
                if( eigen1/(eigen1+eigen2+eigen3) < PLANE_EIGENVALUE_THRESHOLD){
                    float nx = s.eigenvectors().col(0)[0];
                    float ny = s.eigenvectors().col(0)[1];
                    float nz = s.eigenvectors().col(0)[2];
                    error =0;
                    for (int j =0; j < num; j++){
                        error += plane_to_point(origin_x, origin_y, origin_z, nx, ny, nz, vertical_cloud2->points[cloud_index[i][j]].x,vertical_cloud2->points[cloud_index[i][j]].y, vertical_cloud2->points[cloud_index[i][j]].z);
                    }
                    error = error/num;
                    if(error < 0.1){
                        if(nx * origin_x + ny*origin_y + nz*origin_z >= 0){
                            nx = -nx;
                            ny = -ny;
                            nz = -nz;
                        }
                        float nx1 = s.eigenvectors().col(1)[0];
                        float ny1 = s.eigenvectors().col(1)[1];
                        float nz1 = s.eigenvectors().col(1)[2];
                        float nx2 = s.eigenvectors().col(2)[0];
                        float ny2 = s.eigenvectors().col(2)[1];
                        float nz2 = s.eigenvectors().col(2)[2];
                        origin.nx=nx;
                        origin.ny=ny;
                        origin.nz=nz;
                        found_valid[i] = 2;
                        Plane.push_back(origin);
                        std::vector <float > points;
                        for (int j =0; j < num; j++){
                            points.push_back(vertical_cloud2->points[cloud_index[i][j]].x);
                            points.push_back(vertical_cloud2->points[cloud_index[i][j]].y);
                            points.push_back(vertical_cloud2->points[cloud_index[i][j]].z);
                        }
                        for ( int j=0; j<100; j++){
                            points.push_back(origin_x + nx * j/50);
                            points.push_back(origin_y + ny * j/50);
                            points.push_back(origin_z + nz * j/50);
                        }
                        Plane_every_points.push_back(points);
                        points.clear();

//                        for (int w=0; w<100; w++){
//                            float h = 0;
//                            for(int h=0; h<100; h++){
//                                points.push_back(origin_x - 2 * nx1 * (w-50)/50 +  2* nx2* (h-50)/50);
//                                points.push_back(origin_y - 2 * ny1 * (w-50)/50  + 2* ny2* (h-50)/50 );
//                                points.push_back(origin_z - 2 * nz1 * (w-50)/50  + 2* nz2* (h-50)/50 );
//                            }
//                        }
                        points.push_back(origin.origin_x);
                        points.push_back(origin.origin_y);
                        points.push_back(origin.origin_z);
                        points.push_back(origin.origin_x + 2 * nx1);
                        points.push_back(origin.origin_y + 2 * ny1);
                        points.push_back(origin.origin_z + 2 * nz1);
                        points.push_back(origin.origin_x + 2 * nx2);
                        points.push_back(origin.origin_y + 2 * ny2);
                        points.push_back(origin.origin_z + 2 * nz2);
                        points.push_back(origin.origin_x - 2 * nx1);
                        points.push_back(origin.origin_y - 2 * ny1);
                        points.push_back(origin.origin_z - 2 * nz1);
                        points.push_back(origin.origin_x - 2 * nx2);
                        points.push_back(origin.origin_y - 2 * ny2);
                        points.push_back(origin.origin_z - 2 * nz2);
                        Plane_points.push_back(points);
                        points.clear();
//                      float e1= plane_to_point(origin_x, origin_y, origin_z, nx, ny, nz, vertical_cloud2->points[cloud_index[i][0]].x,vertical_cloud2->points[cloud_index[i][0]].y, vertical_cloud2->points[cloud_index[i][0]].z);
//                        std::cout << i << ' ' << "error: " << e1 << std::endl;
                    }
                }
            }
        }
    }


//    std::cout << Line.size() << ' ' <<Plane.size() << std::endl;
//// check with color
//    int color_line =0;
//    int color_plane =0;
//    for (int i=0; i < cloud_index.size(); i++){
//        if (found_valid[i] == 1){
//            if (color_line % 3 == 0 ){
//               for (int j=0; j< cloud_index[i].size(); j++){
//                    temp_point.x = vertical_cloud2->points[cloud_index[i][j]].x;
//                    temp_point.y = vertical_cloud2->points[cloud_index[i][j]].y;
//                    temp_point.z = vertical_cloud2->points[cloud_index[i][j]].z;
//                    temp_point.r = 255;
//                    temp_point.g = 0;
//                    temp_point.b = 0;
//                    temp->points.push_back(temp_point);
//                    temp_normal_point.normal_x = normal_cloud->points[cloud_index[i][j]].normal_x;
//                    temp_normal_point.normal_y = normal_cloud->points[cloud_index[i][j]].normal_y;
//                    temp_normal_point.normal_z = normal_cloud->points[cloud_index[i][j]].normal_z;
//                    temp_normal->points.push_back(temp_normal_point);
//                }
//               color_line ++;
//            }else if (color_line % 3 == 1 ){
//                    for (int j=0; j< cloud_index[i].size(); j++){
//                        temp_point.x = vertical_cloud2->points[cloud_index[i][j]].x;
//                        temp_point.y = vertical_cloud2->points[cloud_index[i][j]].y;
//                        temp_point.z = vertical_cloud2->points[cloud_index[i][j]].z;
//                        temp_point.r = 255;
//                        temp_point.g = 255;
//                        temp_point.b = 0;
//                        temp->points.push_back(temp_point);
//                        temp_normal_point.normal_x = normal_cloud->points[cloud_index[i][j]].normal_x;
//                        temp_normal_point.normal_y = normal_cloud->points[cloud_index[i][j]].normal_y;
//                        temp_normal_point.normal_z = normal_cloud->points[cloud_index[i][j]].normal_z;
//                        temp_normal->points.push_back(temp_normal_point);
//                    }
//                    color_line ++;
//            }else{
//                for (int j=0; j< cloud_index[i].size(); j++){
//                    temp_point.x = vertical_cloud2->points[cloud_index[i][j]].x;
//                    temp_point.y = vertical_cloud2->points[cloud_index[i][j]].y;
//                    temp_point.z = vertical_cloud2->points[cloud_index[i][j]].z;
//                    temp_point.r = 0;
//                    temp_point.g = 255;
//                    temp_point.b = 0;
//                    temp->points.push_back(temp_point);
//                    temp_normal_point.normal_x = normal_cloud->points[cloud_index[i][j]].normal_x;
//                    temp_normal_point.normal_y = normal_cloud->points[cloud_index[i][j]].normal_y;
//                    temp_normal_point.normal_z = normal_cloud->points[cloud_index[i][j]].normal_z;
//                    temp_normal->points.push_back(temp_normal_point);
//                }
//                color_line ++;
//            }
//
//
//        } else if (found_valid[i] == 2){
//            if (color_plane % 3 == 0 ){
//                for (int j=0; j< cloud_index[i].size(); j++){
//                    temp_point.x = vertical_cloud2->points[cloud_index[i][j]].x;
//                    temp_point.y = vertical_cloud2->points[cloud_index[i][j]].y;
//                    temp_point.z = vertical_cloud2->points[cloud_index[i][j]].z;
//                    temp_point.r = 0;
//                    temp_point.g = 0;
//                    temp_point.b = 255;
//                    temp->points.push_back(temp_point);
//                    temp_normal_point.normal_x = normal_cloud->points[cloud_index[i][j]].normal_x;
//                    temp_normal_point.normal_y = normal_cloud->points[cloud_index[i][j]].normal_y;
//                    temp_normal_point.normal_z = normal_cloud->points[cloud_index[i][j]].normal_z;
//                    temp_normal->points.push_back(temp_normal_point);
//                }
//                color_plane ++;
//            }else if(color_plane % 3 == 1){
//                for (int j=0; j< cloud_index[i].size(); j++){
//                    temp_point.x = vertical_cloud2->points[cloud_index[i][j]].x;
//                    temp_point.y = vertical_cloud2->points[cloud_index[i][j]].y;
//                    temp_point.z = vertical_cloud2->points[cloud_index[i][j]].z;
//                    temp_point.r = 0;
//                    temp_point.g = 255;
//                    temp_point.b = 255;
//                    temp->points.push_back(temp_point);
//                    temp_normal_point.normal_x = normal_cloud->points[cloud_index[i][j]].normal_x;
//                    temp_normal_point.normal_y = normal_cloud->points[cloud_index[i][j]].normal_y;
//                    temp_normal_point.normal_z = normal_cloud->points[cloud_index[i][j]].normal_z;
//                    temp_normal->points.push_back(temp_normal_point);
//                }
//                color_plane ++;
//            }
//            else{
//                for (int j=0; j< cloud_index[i].size(); j++){
//                    temp_point.x = vertical_cloud2->points[cloud_index[i][j]].x;
//                    temp_point.y = vertical_cloud2->points[cloud_index[i][j]].y;
//                    temp_point.z = vertical_cloud2->points[cloud_index[i][j]].z;
//                    temp_point.r = 255;
//                    temp_point.g = 255;
//                    temp_point.b = 255;
//                    temp->points.push_back(temp_point);
//                    temp_normal_point.normal_x = normal_cloud->points[cloud_index[i][j]].normal_x;
//                    temp_normal_point.normal_y = normal_cloud->points[cloud_index[i][j]].normal_y;
//                    temp_normal_point.normal_z = normal_cloud->points[cloud_index[i][j]].normal_z;
//                    temp_normal->points.push_back(temp_normal_point);
//                }
//                color_plane ++;
//            }
//        }
//    }
//
//////// color check;
////
//    pcl::visualization::CloudViewer viewer3("Cloud Viewer");
//
//
//    viewer3.showCloud(temp);
//
//    while (!viewer3.wasStopped ())
//    {
//
//    }
//
//    pcl::visualization::PCLVisualizer viewer2("pcl viewer");
//
//    viewer2.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(temp,temp_normal,5,1);
//    while (!viewer2.wasStopped ())
//    {
//        viewer2.spinOnce ();
//    }

}
