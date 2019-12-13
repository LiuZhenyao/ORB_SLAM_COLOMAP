#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <eigen3/Eigen/Dense>

namespace vins_PoseGraph_reader
{
// This namespace define the functions used to process pose graph generated from VINS-Mono
void loadPoseGraph();
void saveImages_txt_in_COLMAP_format(int index, Eigen::Vector3d PG_T, Eigen::Quaterniond PG_Q);
void saveCameras_txt_in_COLMAP_format();
void savePoints3D_txt_in_COLMAP_format();

// This function used for reconstruction from GPS position and VINS pose (rotation)
void test_pg();
} // namespace vins_PoseGraph_reader

// Global function utilities
void testFunc();
std::string modify_img_name(int index_num);
void write_camera_model_mynteye(FILE *pFile, const int &camera_id, const std::string &camera_model);
void write_camera_model_JD(FILE *pFile, const int &camera_id, const std::string &camera_model);

// Global path
// const std::string POSE_GRAPH_SAVE_PATH = "/home/shu/catkin_ws/src/VINS-Mono/pose_graph/TEST_3/";
const std::string POSE_GRAPH_SAVE_PATH = "/home/shu/dense_orbslam_ws/ORB_SLAM2_REMODE/backup_trajectory/";
const std::string IMAGES_TXT_SAVE_PATH = "/home/shu/Downloads/Rosario_2019/ORB_SLAM_COLOMAP/output/";
const std::string CAMERAS_TXT_SAVE_PATH = "/home/shu/Downloads/Rosario_2019/ORB_SLAM_COLOMAP/output/";
const std::string POINTS3D_TXT_SAVE_PATH = "/home/shu/Downloads/Rosario_2019/ORB_SLAM_COLOMAP/output/";

namespace orbslam2_PoseGraph_reader
{
// This namespace define the functions used to process key-frames/ camera-frames generated from ORB-SLAM2

// Those function used for reconstruction from agricultral data of John Deere
// Camera: crossed-view
void loadPoseGraph();
void saveImages_txt_in_COLMAP_format(int index, Eigen::Vector3d PG_T, Eigen::Quaterniond PG_Q);
void saveCameras_txt_in_COLMAP_format();
void savePoints3D_txt_in_COLMAP_format();

// This function used for reconstruction from field data and ORB-SLAM2, maybe with GPS position
// Camera: mynt-eye
void test_pg();
} // namespace orbslam2_PoseGraph_reader