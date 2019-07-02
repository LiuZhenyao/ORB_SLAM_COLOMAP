#include "pose_graph.h"


void vins_PoseGraph_reader::testFunc() {
    // Test Function, make sure your pakege has been installed properly

    std::cout << "TEST FUNCTION RUNING -- sample namespace" << std::endl;
    
    cv::Mat image;
    image = cv::imread("/home/shu/Pictures/1.png", 1);
    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
    cv::imshow("Display Image", image);
    cv::waitKey(0); // if Call OpenCV sucess!

    Eigen::MatrixXd m(2,2);
    m(0,0) = 3;
    m(1,0) = 2.5;
    m(0,1) = -1;
    m(1,1) = m(1,0) + m(0,1);
    std::cout << m << std::endl; // if Call Eigen sucess!

    std::cout << "TEST PASS!" << std::endl;

}

void vins_PoseGraph_reader::loadPoseGraph() {
    // Read pose graph data from *.bin, which was generated from VINS-Mono

    FILE * pFile;
    std::string file_path = POSE_GRAPH_SAVE_PATH + "pose_graph.txt";
    printf("lode pose graph from: %s \n", file_path.c_str());
    printf("pose graph loading...\n");
    std::printf("images.txt path: %s\n", IMAGES_TXT_SAVE_PATH.c_str());
    std::printf("images.txt saving... \n");
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
    while (fscanf(pFile,"%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %d", 
                        &index, &time_stamp, 
                        &VIO_Tx, &VIO_Ty, &VIO_Tz, 
                        &PG_Tx, &PG_Ty, &PG_Tz, 
                        &VIO_Qw, &VIO_Qx, &VIO_Qy, &VIO_Qz, 
                        &PG_Qw, &PG_Qx, &PG_Qy, &PG_Qz, 
                        &loop_index,
                        &loop_info_0, &loop_info_1, &loop_info_2, &loop_info_3, 
                        &loop_info_4, &loop_info_5, &loop_info_6, &loop_info_7,
                        &keypoints_num) != EOF) 
    {
        
        // printf("I read: %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %d\n", 
        //                 index, time_stamp, 
        //                 VIO_Tx, VIO_Ty, VIO_Tz, 
        //                 PG_Tx, PG_Ty, PG_Tz, 
        //                 VIO_Qw, VIO_Qx, VIO_Qy, VIO_Qz, 
        //                 PG_Qw, PG_Qx, PG_Qy, PG_Qz, 
        //                 loop_index,
        //                 loop_info_0, loop_info_1, loop_info_2, loop_info_3, 
        //                 loop_info_4, loop_info_5, loop_info_6, loop_info_7,
        //                 keypoints_num);

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

        vins_PoseGraph_reader::saveImages_txt_in_COLMAP_format(index, VIO_T, PG_T, VIO_Q, PG_Q);
    }
    fclose (pFile);

    vins_PoseGraph_reader::saveCameras_txt_in_COLMAP_format();
    vins_PoseGraph_reader::savePoints3D_txt_in_COLMAP_format();

}

void vins_PoseGraph_reader::saveImages_txt_in_COLMAP_format(int index, Eigen::Vector3d VIO_T, Eigen::Vector3d PG_T, Eigen::Quaterniond VIO_Q, Eigen::Quaterniond PG_Q) {
    // Save the images' pose in a txt file
    // Image list with two lines of data per image: 
    //      IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
    //      POINTS2D[] as (X, Y, POINT3D_ID) <-- this line will be empty

    FILE *pFile;
    std::string file_path = IMAGES_TXT_SAVE_PATH + "images.txt";
    int index_num = index+1;
    std::string tmp;

    if (index_num < 10)
    {
        tmp = "00000" + std::to_string(index_num);
    }
    else if (index_num > 9 && index_num < 100)
    {
        tmp = "0000" + std::to_string(index_num);
    }
    else if (index_num > 99 && index_num < 1000)
    {
        tmp = "000" + std::to_string(index_num);
    }
    else if (index_num > 999 && index_num < 10000)
    {
        tmp = "00" + std::to_string(index_num);
    }
    else if (index_num > 9999 && index_num < 100000)
    {
        tmp = "0" + std::to_string(index_num);
    }

    std::string image_name = "IMG" + tmp + ".png";
    pFile = fopen(file_path.c_str(), "a");
    fprintf (pFile, "%d %f %f %f %f %f %f %f %d %s \n\n",
                    index_num,
                    // VIO_Q.w(), VIO_Q.x(), VIO_Q.y(), VIO_Q.z(), 
                    // VIO_T.x(), VIO_T.y(), VIO_T.z(),
                    PG_Q.w(), PG_Q.x(), PG_Q.y(), PG_Q.z(), 
                    PG_T.x(), PG_T.y(), PG_T.z(),
                    CAMERA_ID, image_name.c_str()
                    );
    fclose(pFile);
}

void vins_PoseGraph_reader::saveCameras_txt_in_COLMAP_format() {
    // Camera list with one line of data per camera: 
    //      CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[focal length in pixel, principal point at pixel location]

    FILE *pFile;
    std::printf("cameras.txt path: %s\n", CAMERAS_TXT_SAVE_PATH.c_str());
    std::printf("cameras.txt saving... \n");
    std::string file_path = CAMERAS_TXT_SAVE_PATH + "cameras.txt";
    pFile = fopen(file_path.c_str(), "w");
    if (CAMERA_MODEL == "PINHOLE") {
        fprintf (pFile, "%d %s %d %d %.2f %.2f %d %d \n",
            CAMERA_ID, CAMERA_MODEL.c_str(), IMG_WIDTH, IMG_HEIGHT, fx, fy, cx, cy);
    }
    else if (CAMERA_MODEL == "SIMPLE_RADIAL")
    {   
        const float some_param = 0.0177572;
        fprintf (pFile, "%d %s %d %d %.2f %d %d %f\n",
            CAMERA_ID, CAMERA_MODEL.c_str(), IMG_WIDTH, IMG_HEIGHT, fx, cx, cy, some_param);
    }
    else if (CAMERA_MODEL == "SIMPLE_PINHOLE")
    {
           fprintf (pFile, "%d %s %d %d %.2f %d %d \n",
            CAMERA_ID, CAMERA_MODEL.c_str(), IMG_WIDTH, IMG_HEIGHT, fx, cx, cy);
    }
    fclose(pFile);
}

void vins_PoseGraph_reader::savePoints3D_txt_in_COLMAP_format() {
    // 3D point list with one line of data per point:
    // POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)

    // This file should be empty
    FILE *pFile;
    std::printf("points3D.txt path: %s\n", POINTS3D_TXT_SAVE_PATH.c_str());
    std::printf("points3D.txt saving... \n");
    std::string file_path = POINTS3D_TXT_SAVE_PATH + "points3D.txt";
    pFile = fopen(file_path.c_str(), "w");
    // fprintf (pFile, "\n");
    fclose(pFile);
}
