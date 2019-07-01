#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <eigen3/Eigen/Dense>

namespace vins_PoseGraph_reader {
    // This namespace define the functions used to read pose graph generated from VINS-Mono, MYNT EYE supported

    void testFunc();
    void loadPoseGraph();
    void saveImages_txt_in_COLMAP_format(int index, Eigen::Vector3d VIO_T, Eigen::Vector3d PG_T, Eigen::Quaterniond VIO_Q, Eigen::Quaterniond PG_Q);
    void saveCameras_txt_in_COLMAP_format();
    void savePoints3D_txt_in_COLMAP_format();

    // Define camera info for MYNT EYE S1030, calibration should be done beforehand
    const int CAMERA_ID = 1; // just set index 1 to mynteye camera
    const float fx = 440.85; // in pixel
    const float fy = 440.68; // in pixel
    const int cx = 354; // principle point position in pixel
    const int cy = 233;
    const int IMG_WIDTH = 752;
    const int IMG_HEIGHT = 480;
    const std::string CAMERA_MODEL = "SIMPLE_RADIAL"; //SIMPLE_RADIAL, SIMPLE_PINHOLE, PINHOLE 


    // Global path
    const std::string POSE_GRAPH_SAVE_PATH = "/home/shu/catkin_ws/src/VINS-Mono/pose_graph/TEST_2/";
    const std::string IMAGES_TXT_SAVE_PATH = "/home/shu/fangwenSHU/Monocular-SLAM-based-on-MYNTEYE/output/";
    const std::string CAMERAS_TXT_SAVE_PATH = "/home/shu/fangwenSHU/Monocular-SLAM-based-on-MYNTEYE/output/";
    const std::string POINTS3D_TXT_SAVE_PATH = "/home/shu/fangwenSHU/Monocular-SLAM-based-on-MYNTEYE/output/";
}

// class BinFile_reader {
//     public:
//         BinFile_reader();                                                  // save_file_path{save_file_path_} {}
//         ~BinFile_reader();

//         // custom member function
//         void testFunc();                                                                                    
//         // bool reader();
//         // bool saver();

//     private:
//         std::string input_file_path = "";
//         std::string save_file_path = "";
// };

