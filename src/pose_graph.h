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

    // Define camera info for MYNT EYE S1030
    // Calibration should be done beforehand
    const int CAMERA_ID = 1; // just set index 1 to mynteye camera
    const int IMG_WIDTH = 752;
    const int IMG_HEIGHT = 480;
    const std::string CAMERA_MODEL = "SIMPLE_RADIAL"; //SIMPLE_RADIAL, SIMPLE_PINHOLE, PINHOLE, RADIAL, OPENCV
    // Projection parameters
    const float fx = 440.8503716053656; // in pixel
    const float fy = 440.67694216951423;
    const float cx = 354.173182391866; // principle point position in pixel
    const float cy = 233.00424421090963;
    // Distortion parameters
    const float p1 = -0.00027894166759601926;
    const float p2 = 0.0007775925641113786;
    const float k1 = -0.3205125886805923;
    const float k2 = 0.10861100855243255;

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

namespace orbslam2_PoseGraph_reader {
    
}