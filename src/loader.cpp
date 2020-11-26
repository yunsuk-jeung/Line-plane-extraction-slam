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
    viewer.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(vertical_cloud2,normal_cloud,10,1);
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

void loader::clusterizer(std::vector < feature_point  > &Line, std::vector <  feature_point  > &Plane){

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Normal>::Ptr temp_normal (new pcl::PointCloud<pcl::Normal>);
    pcl::PointXYZRGB temp_point;
    pcl::Normal temp_normal_point;
    std::vector < std::vector < int > > visit(ROW, std::vector <int> (COL));
    std::vector < std::vector < pcl::PointXYZRGB > > surface_point;
    std::vector < std::vector < int > > cloud_index;

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
    ////cluster
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
                    visit[i][j] = max_surface;
                    surface = max_surface;
                    surface_point.push_back(std::vector<pcl::PointXYZRGB>());
                    cloud_index.push_back(std::vector<int>());
                } else{
                    surface = visit[i][j];

                }
                for (int ii= i-CLUSTER_NEIGHBOR_ROW; ii < i+CLUSTER_NEIGHBOR_ROW; ii++){
                    for (int jj = j-CLUSTER_NEIGHBOR_COL; jj < j+CLUSTER_NEIGHBOR_COL; jj++ ){

                        if (ii < 0 || ii >=ROW || jj < 0 || jj >= COL ) {
                            continue;
                        }
                        if(depth_image[ii][jj].index <0){
                            continue;
                        }
//                        if(visit[ii][jj] != 0 ){
//                            continue;
//                        }
                        x2 = vertical_cloud2->points[depth_image[ii][jj].index].x;
                        y2 = vertical_cloud2->points[depth_image[ii][jj].index].y;
                        z2 = vertical_cloud2->points[depth_image[ii][jj].index].z;
                        nx2 = normal_cloud->points[depth_image[ii][jj].index].normal_x;
                        ny2 = normal_cloud->points[depth_image[ii][jj].index].normal_y;
                        nz2 = normal_cloud->points[depth_image[ii][jj].index].normal_z;

                        if(pow(x2-x1,2)+pow(y2-y1,2)+pow(z2-z1,2)< CLUSTER_NEIGHBOR_DISTANCE && nx1 * nx2 + ny1 * ny2 + nz1 * nz2 > CLUSTER_NEIGHBOR_ANGLE ){
                            visit[ii][jj] = surface;
                        }
                    }
                }
            }
        }
    }
    for(int i=0; i<ROW; i++){
        for (int j=0; j<COL; j++){
            if(depth_image[i][j].index < 0){
                continue;
            }
            surface = visit[i][j];
            surface_point[surface-1].push_back(vertical_cloud2->points[depth_image[i][j].index]);
            cloud_index[surface-1].push_back(depth_image[i][j].index);
        }
    }

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

    std::vector < int> found_valid(surface_point.size());
    Eigen::Matrix3f cc;
    Eigen::Vector3f c;
    Eigen::Matrix3f cov_matrix;

    for (int i=0; i< surface_point.size(); i++){
        int num = surface_point[i].size();

        if (num > 10){
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

            float origin_x = cx/surface_point[i].size();
            float origin_y = cy/surface_point[i].size();
            float origin_z = cz/surface_point[i].size();
            origin.origin_x = origin_x;
            origin.origin_y = origin_y;
            origin.origin_z = origin_z;

            cov_matrix = cc ;
            cov_matrix -= c * c.transpose() / surface_point[i].size();

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
                if (error < 0.025){
                    found_valid[i] = 1;
                    origin.nx=nx;
                    origin.ny=ny;
                    origin.nz=nz;
                    Line.push_back(origin);
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
                        if(error < 0.05){
                            origin.nx=nx;
                            origin.ny=ny;
                            origin.nz=nz;
                            found_valid[i] = 2;
                            Plane.push_back(origin);
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
                    if(error < 0.05){
                        origin.nx=nx;
                        origin.ny=ny;
                        origin.nz=nz;
                        found_valid[i] = 2;
                        Plane.push_back(origin);
                    }
                }
            }
        }
    }


//    std::cout << Line.size() << ' ' <<Plane.size() << std::endl;

//    int color_line =0;
//    int color_plane =0;
//    for (int i=0; i < surface_point.size(); i++){
//        if (found_valid[i] == 1){
//            if (color_line % 3 == 0 ){
//               for (int j=0; j< surface_point[i].size(); j++){
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
//                    for (int j=0; j< surface_point[i].size(); j++){
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
//                for (int j=0; j< surface_point[i].size(); j++){
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
//        } else if (found_valid[i] == 2){
//            if (color_plane % 3 == 0 ){
//                for (int j=0; j< surface_point[i].size(); j++){
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
//                for (int j=0; j< surface_point[i].size(); j++){
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
//                for (int j=0; j< surface_point[i].size(); j++){
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

//// color check;
//    pcl::visualization::CloudViewer viewer("Cloud Viewer");
//
//
//    viewer.showCloud(temp);
//
//    while (!viewer.wasStopped ())
//    {
//
//    }

//    pcl::visualization::PCLVisualizer viewer2("pcl viewer");
//
//    viewer2.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(temp,temp_normal,5,1);
//    while (!viewer2.wasStopped ())
//    {
//        viewer2.spinOnce ();
//    }

}
