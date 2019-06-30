#include "read_binFile.h"


void vins_PoseGraph_reader::testFunc() {
    std::cout << "TEST FUNCTION RUNING -- sample namespace" << std::endl;
    
    cv::Mat image;
    image = cv::imread("/home/shu/Pictures/1.png", 1);
    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
    cv::imshow("Display Image", image);
    cv::waitKey(0);

    Eigen::MatrixXd m(2,2);
    m(0,0) = 3;
    m(1,0) = 2.5;
    m(0,1) = -1;
    m(1,1) = m(1,0) + m(0,1);
    std::cout << m << std::endl;

    std::cout << "TEST PASS!" << std::endl;

}

void vins_PoseGraph_reader::saveImage_txt_in_COLMAP_format() {

}

void vins_PoseGraph_reader::saveCamera_txt_in_COLMAP_format() {
    
}

void vins_PoseGraph_reader::savePoints3D_txt_in_COLMAP_format() {
    
}

void vins_PoseGraph_reader::loadPoseGraph_from_vins() {
    FILE * pFile;
    std::string POSE_GRAPH_SAVE_PATH = "/home/shu/catkin_ws/src/VINS-Mono/pose_graph/TEST/";
    std::string file_path = POSE_GRAPH_SAVE_PATH + "pose_graph.txt";
    printf("lode pose graph from: %s \n", file_path.c_str());
    printf("pose graph loading...\n");
    pFile = fopen (file_path.c_str(),"r");
    if (pFile == NULL)
    {
        printf("lode previous pose graph error: wrong previous pose graph path or no previous pose graph \n the system will start with new pose graph \n");
        return;
    }
    int index;
    double time_stamp;
    double VIO_Tx, VIO_Ty, VIO_Tz;
    double PG_Tx, PG_Ty, PG_Tz;
    double VIO_Qw, VIO_Qx, VIO_Qy, VIO_Qz;
    double PG_Qw, PG_Qx, PG_Qy, PG_Qz;
    double loop_info_0, loop_info_1, loop_info_2, loop_info_3;
    double loop_info_4, loop_info_5, loop_info_6, loop_info_7;
    int loop_index;
    int keypoints_num;
    Eigen::Matrix<double, 8, 1 > loop_info;
    int cnt = 0;
    while (fscanf(pFile,"%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %d", &index, &time_stamp, 
                                    &VIO_Tx, &VIO_Ty, &VIO_Tz, 
                                    &PG_Tx, &PG_Ty, &PG_Tz, 
                                    &VIO_Qw, &VIO_Qx, &VIO_Qy, &VIO_Qz, 
                                    &PG_Qw, &PG_Qx, &PG_Qy, &PG_Qz, 
                                    &loop_index,
                                    &loop_info_0, &loop_info_1, &loop_info_2, &loop_info_3, 
                                    &loop_info_4, &loop_info_5, &loop_info_6, &loop_info_7,
                                    &keypoints_num) != EOF) 
    {
        
        // printf("I read: %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %d\n", index, time_stamp, 
        //                             VIO_Tx, VIO_Ty, VIO_Tz, 
        //                             PG_Tx, PG_Ty, PG_Tz, 
        //                             VIO_Qw, VIO_Qx, VIO_Qy, VIO_Qz, 
        //                             PG_Qw, PG_Qx, PG_Qy, PG_Qz, 
        //                             loop_index,
        //                             loop_info_0, loop_info_1, loop_info_2, loop_info_3, 
        //                             loop_info_4, loop_info_5, loop_info_6, loop_info_7,
        //                             keypoints_num);

        cv::Mat image;
        std::string image_path, descriptor_path;
        Eigen::Vector3d VIO_T(VIO_Tx, VIO_Ty, VIO_Tz);
        Eigen::Vector3d PG_T(PG_Tx, PG_Ty, PG_Tz);
        Eigen::Quaterniond VIO_Q;
        VIO_Q.w() = VIO_Qw;
        VIO_Q.x() = VIO_Qx;
        VIO_Q.y() = VIO_Qy;
        VIO_Q.z() = VIO_Qz;
        Eigen::Quaterniond PG_Q;
        PG_Q.w() = PG_Qw;
        PG_Q.x() = PG_Qx;
        PG_Q.y() = PG_Qy;
        PG_Q.z() = PG_Qz;
        Eigen::Matrix3d VIO_R, PG_R;
        VIO_R = VIO_Q.toRotationMatrix();
        PG_R = PG_Q.toRotationMatrix();
        Eigen::Matrix<double, 8, 1 > loop_info;
        loop_info << loop_info_0, loop_info_1, loop_info_2, loop_info_3, loop_info_4, loop_info_5, loop_info_6, loop_info_7;
    }
    fclose (pFile);


}
