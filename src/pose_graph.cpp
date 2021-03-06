#include "pose_graph.h"
#include "mynteye.h"
#include "JohnDeere_Mono.h"

// Functions for reading pose graph from VINS-Mono
void vins_PoseGraph_reader::loadPoseGraph()
{
    // Read pose graph data from *.txt, which was generated from VINS-Mono
    FILE *pFile;
    std::string file_path = POSE_GRAPH_SAVE_PATH + "pose_graph.txt";
    printf("lode pose graph from: %s \n", file_path.c_str());
    printf("pose graph loading...\n");
    std::printf("images.txt path: %s\n", IMAGES_TXT_SAVE_PATH.c_str());
    std::printf("images.txt saving... \n");
    pFile = fopen(file_path.c_str(), "r");
    if (pFile == NULL)
    {
        printf("load pose graph error: wrong pose graph path or no pose graph available \n");
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
    Eigen::Matrix<double, 8, 1> loop_info;
    int cnt = 0;
    while (fscanf(pFile, "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %d",
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
        Eigen::Vector3d VIO_T(VIO_Tx, VIO_Ty, VIO_Tz);
        Eigen::Vector3d PG_T(PG_Tx, PG_Ty, PG_Tz);
        Eigen::Quaterniond PG_Q;
        PG_Q.w() = PG_Qw;
        PG_Q.x() = PG_Qx;
        PG_Q.y() = PG_Qy;
        PG_Q.z() = PG_Qz;
        Eigen::Matrix3d PG_R;
        PG_R = PG_Q.toRotationMatrix();

        // the coordinate axis of COLMAP is defined in a way differ from the one from VINS, rotate -90 deg around z axis
        // write the Quaternion parameters of transpose(R) and -transpose(R) * T
        // https://github.com/colmap/colmap/issues/434
        Eigen::Matrix3d R_z_;
        R_z_.row(0) << 0, 1, 0;
        R_z_.row(1) << -1, 0, 0;
        R_z_.row(2) << 0, 0, 1;

        Eigen::Quaterniond PG_Q_(R_z_ * PG_R.transpose());
        Eigen::Vector3d PG_T_;
        PG_T_ = -R_z_ * PG_R.transpose() * PG_T;

        // save images.txt for COLMAP
        vins_PoseGraph_reader::saveImages_txt_in_COLMAP_format(index, PG_T_, PG_Q_);
    }
    fclose(pFile);

    // save cameras.txt and points3D.txt for COLMAP
    vins_PoseGraph_reader::saveCameras_txt_in_COLMAP_format();
    vins_PoseGraph_reader::savePoints3D_txt_in_COLMAP_format();
}

void vins_PoseGraph_reader::saveImages_txt_in_COLMAP_format(int index, Eigen::Vector3d PG_T, Eigen::Quaterniond PG_Q)
{
    // Save the images' pose in a txt file
    // Image list with two lines of data per image:
    //      IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
    //      POINTS2D[] as (X, Y, POINT3D_ID) !!!!!!attention <-- this line will be empty
    FILE *pFile;
    std::string file_path = IMAGES_TXT_SAVE_PATH + "images.txt";
    int index_num = index + 1;
    std::string tmp = modify_img_name(index_num);
    std::string image_name = "IMG" + tmp + ".png";
    pFile = fopen(file_path.c_str(), "a");
    fprintf(pFile, "%d %f %f %f %f %f %f %f %d %s \n\n",
            index_num,
            PG_Q.w(), PG_Q.x(), PG_Q.y(), PG_Q.z(),
            PG_T.x(), PG_T.y(), PG_T.z(),
            mynteye::CAMERA_ID, image_name.c_str());
    fclose(pFile);
}

void vins_PoseGraph_reader::saveCameras_txt_in_COLMAP_format()
{
    // Camera list with one line of data per camera:
    //      CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[focal length in pixel, principal point at pixel location]
    FILE *pFile;
    std::printf("cameras.txt path: %s\n", CAMERAS_TXT_SAVE_PATH.c_str());
    std::printf("cameras.txt saving... \n");
    std::string file_path = CAMERAS_TXT_SAVE_PATH + "cameras.txt";
    pFile = fopen(file_path.c_str(), "w");
    write_camera_model_mynteye(pFile, mynteye::CAMERA_ID, mynteye::CAMERA_MODEL);
    fclose(pFile);
}

void vins_PoseGraph_reader::savePoints3D_txt_in_COLMAP_format()
{
    // 3D point list with one line of data per point:
    // POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)
    FILE *pFile; // !!!!!!attention, this file should be empty in case no 3D points record from SLAM
    std::printf("points3D.txt path: %s\n", POINTS3D_TXT_SAVE_PATH.c_str());
    std::printf("points3D.txt should be empty! \n");
    std::string file_path = POINTS3D_TXT_SAVE_PATH + "points3D.txt";
    pFile = fopen(file_path.c_str(), "w");
    fclose(pFile);
}

void testFunc()
{
    // Test Function, make sure your pakege has been installed properly
    std::cout << "TEST FUNCTION RUNING -- sample namespace" << std::endl;
    cv::Mat image;
    image = cv::imread("/home/shu/Pictures/1.png", 1);
    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
    cv::imshow("Display Image", image);
    cv::waitKey(0); // if Call OpenCV sucess!

    Eigen::MatrixXd m(2, 2);
    m(0, 0) = 3;
    m(1, 0) = 2.5;
    m(0, 1) = -1;
    m(1, 1) = m(1, 0) + m(0, 1);
    std::cout << m << std::endl; // if Call Eigen sucess!
    std::cout << "TEST PASS!" << std::endl;
}

std::string modify_img_name(int index_num)
{
    // This is necessary for COLMAP commandline "Reconstruct sparse/dense model from known camera poses"
    // Fix the bug when running "colmap point_triangulator":
    //      Check failed: existing_image.Name() == image.second.Name() (17.JPG vs. 16.JPG)
    // SO, the image will be saved like IMG000001, IMG000002 ...
    std::string temp;

    if (index_num < 10)
    {
        temp = "00000" + std::to_string(index_num);
    }
    else if (index_num > 9 && index_num < 100)
    {
        temp = "0000" + std::to_string(index_num);
    }
    else if (index_num > 99 && index_num < 1000)
    {
        temp = "000" + std::to_string(index_num);
    }
    else if (index_num > 999 && index_num < 10000)
    {
        temp = "00" + std::to_string(index_num);
    }
    else if (index_num > 9999 && index_num < 100000)
    {
        temp = "0" + std::to_string(index_num);
    }

    return temp;
}

void write_camera_model_mynteye(FILE *pFile, const int &camera_id, const std::string &camera_model)
{
    if (camera_model == "PINHOLE")
    {
        // Pinhole camera model.
        // No Distortion is assumed. Only focal length and principal point is modeled.
        // Parameter list is expected in the following order:
        //    fx, fy, cx, cy
        // See https://en.wikipedia.org/wiki/Pinhole_camera_model
        fprintf(pFile, "%d %s %d %d %f %f %f %f \n", camera_id, camera_model.c_str(), mynteye::IMG_WIDTH, mynteye::IMG_HEIGHT,
                mynteye::fx, mynteye::fy, mynteye::cx, mynteye::cy);
    }
    else if (camera_model == "SIMPLE_PINHOLE")
    {
        // Simple Pinhole camera model.
        // No Distortion is assumed. Only focal length and principal point is modeled.
        // Parameter list is expected in the following order:
        //   f, cx, cy
        fprintf(pFile, "%d %s %d %d %f %f %f \n",
                camera_id, camera_model.c_str(), mynteye::IMG_WIDTH, mynteye::IMG_HEIGHT, mynteye::fx, mynteye::cx, mynteye::cy);
    }
    else if (camera_model == "SIMPLE_RADIAL")
    {
        // Simple camera model with one focal length and one radial distortion parameter.
        //
        // This model is similar to the camera model that VisualSfM uses with the
        // difference that the distortion here is applied to the projections and
        // not to the measurements.
        //
        // Parameter list is expected in the following order:
        //    f, cx, cy, k
        const float k = 0.0177572;
        fprintf(pFile, "%d %s %d %d %f %f %f %f\n",
                camera_id, camera_model.c_str(), mynteye::IMG_WIDTH, mynteye::IMG_HEIGHT, mynteye::fx, mynteye::cx, mynteye::cy, k);
    }
    else if (camera_model == "RADIAL")
    {
        // Simple camera model with one focal length and two radial distortion
        // parameters.
        //
        // This model is equivalent to the camera model that Bundler uses
        // (except for an inverse z-axis in the camera coordinate system).
        //
        // Parameter list is expected in the following order:
        //
        //    f, cx, cy, k1, k2
        fprintf(pFile, "%d %s %d %d %f %f %f %f %f\n",
                camera_id, camera_model.c_str(), mynteye::IMG_WIDTH, mynteye::IMG_HEIGHT, mynteye::fx, mynteye::cx, mynteye::cy, mynteye::k1, mynteye::k2);
    }
    else if (camera_model == "OPENCV")
    {
        // OpenCV camera model.
        //
        // Based on the pinhole camera model. Additionally models radial and
        // tangential distortion (up to 2nd degree of coefficients). Not suitable for
        // large radial distortions of fish-eye cameras.
        //
        // Parameter list is expected in the following order:
        //
        //    fx, fy, cx, cy, k1, k2, p1, p2
        //
        // See
        // http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
        fprintf(pFile, "%d %s %d %d %f %f %f %f %f %f %f %f\n",
                camera_id, camera_model.c_str(), mynteye::IMG_WIDTH, mynteye::IMG_HEIGHT,
                mynteye::fx, mynteye::fy, mynteye::cx, mynteye::cy, mynteye::k1, mynteye::k2, mynteye::p1, mynteye::p2);
    }
}

void write_camera_model_JD(FILE *pFile, const int &camera_id, const std::string &camera_model)
{
    if (camera_model == "OPENCV")
    {
        // OpenCV camera model.
        //
        // Based on the pinhole camera model. Additionally models radial and
        // tangential distortion (up to 2nd degree of coefficients). Not suitable for
        // large radial distortions of fish-eye cameras.
        //
        // Parameter list is expected in the following order:
        //
        //    fx, fy, cx, cy, k1, k2, p1, p2
        //
        // See
        // http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
        fprintf(pFile, "%d %s %d %d %f %f %f %f %f %f %f %f\n",
                camera_id, camera_model.c_str(), JohnDeere_Mono::IMG_WIDTH, JohnDeere_Mono::IMG_HEIGHT,
                JohnDeere_Mono::fx, JohnDeere_Mono::fy, JohnDeere_Mono::cx, JohnDeere_Mono::cy,
                JohnDeere_Mono::k1, JohnDeere_Mono::k2, JohnDeere_Mono::p1, JohnDeere_Mono::p2);
    }
}

void orbslam2_PoseGraph_reader::loadPoseGraph()
{
    // Read pose graph data from *.txt, which was generated from ORB-SLAM2
    FILE *pFile;
    std::string file_path = POSE_GRAPH_SAVE_PATH + "KeyFrameTrajectory_04.txt";
    // std::string file_path = "../utility_python/pose_graph_test.txt";
    printf("lode pose graph from: %s \n", file_path.c_str());
    printf("pose graph loading...\n");
    std::printf("images.txt path: %s\n", IMAGES_TXT_SAVE_PATH.c_str());
    std::printf("images.txt saving... \n");
    pFile = fopen(file_path.c_str(), "r");
    if (pFile == NULL)
    {
        printf("load pose graph error: wrong pose graph path or no pose graph available \n");
        return;
    }

    int index = 0;
    double time_stamp;
    double PG_Tx, PG_Ty, PG_Tz;
    double PG_Qw, PG_Qx, PG_Qy, PG_Qz;
    // TUM format:
    // The format of each line is 'timestamp tx ty tz qx qy qz qw'

    while (fscanf(pFile, "%lf %lf %lf %lf %lf %lf %lf %lf",
                  &time_stamp,
                  &PG_Tx, &PG_Ty, &PG_Tz,
                  &PG_Qx, &PG_Qy, &PG_Qz, &PG_Qw) != EOF)
    {
        cv::Mat image;
        std::string image_path, descriptor_path;
        Eigen::Vector3d PG_T(PG_Tx, PG_Ty, PG_Tz);
        Eigen::Quaterniond PG_Q;
        PG_Q.w() = PG_Qw;
        PG_Q.x() = PG_Qx;
        PG_Q.y() = PG_Qy;
        PG_Q.z() = PG_Qz;
        Eigen::Matrix3d VIO_R, PG_R;
        PG_R = PG_Q.toRotationMatrix();

        Eigen::Quaterniond PG_Q_(PG_R.transpose());
        Eigen::Vector3d PG_T_;
        PG_T_ = -PG_R.transpose() * PG_T;

        // save images.txt for COLMAP
        orbslam2_PoseGraph_reader::saveImages_txt_in_COLMAP_format(index, PG_T_, PG_Q_);
        index += 1;
    }
    fclose(pFile);

    // save cameras.txt and points3D.txt for COLMAP
    orbslam2_PoseGraph_reader::saveCameras_txt_in_COLMAP_format();
    orbslam2_PoseGraph_reader::savePoints3D_txt_in_COLMAP_format();
}

void orbslam2_PoseGraph_reader::saveImages_txt_in_COLMAP_format(int index, Eigen::Vector3d PG_T, Eigen::Quaterniond PG_Q)
{
    // Save the images' pose in a txt file
    // Image list with two lines of data per image:
    //      IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
    //      POINTS2D[] as (X, Y, POINT3D_ID) !!!!!!attention <-- this line will be empty
    FILE *pFile;
    std::string file_path = IMAGES_TXT_SAVE_PATH + "images.txt";
    int index_num = index + 1;
    std::string tmp = modify_img_name(index_num);
    std::string image_name = "IMG" + tmp + ".png";
    pFile = fopen(file_path.c_str(), "a");
    fprintf(pFile, "%d %f %f %f %f %f %f %f %d %s \n\n",
            index_num,
            PG_Q.w(), PG_Q.x(), PG_Q.y(), PG_Q.z(),
            PG_T.x(), PG_T.y(), PG_T.z(),
            JohnDeere_Mono::CAMERA_ID, image_name.c_str());
    fclose(pFile);
}

void orbslam2_PoseGraph_reader::saveCameras_txt_in_COLMAP_format()
{
    // Camera list with one line of data per camera:
    //      CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[focal length in pixel, principal point at pixel location]
    FILE *pFile;
    std::printf("cameras.txt path: %s\n", CAMERAS_TXT_SAVE_PATH.c_str());
    std::printf("cameras.txt saving... \n");
    std::string file_path = CAMERAS_TXT_SAVE_PATH + "cameras.txt";
    pFile = fopen(file_path.c_str(), "w");
    write_camera_model_JD(pFile, JohnDeere_Mono::CAMERA_ID, JohnDeere_Mono::CAMERA_MODEL);
    fclose(pFile);
}

void orbslam2_PoseGraph_reader::savePoints3D_txt_in_COLMAP_format()
{
    // 3D point list with one line of data per point:
    // POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)
    FILE *pFile; // !!!!!!attention, this file should be empty in case no 3D points record from SLAM
    std::printf("points3D.txt path: %s\n", POINTS3D_TXT_SAVE_PATH.c_str());
    std::printf("points3D.txt should be empty! \n");
    std::string file_path = POINTS3D_TXT_SAVE_PATH + "points3D.txt";
    pFile = fopen(file_path.c_str(), "w");
    fclose(pFile);
}

void vins_PoseGraph_reader::test_pg()
{
    // Read pose graph data from *.txt, which was generated from ORB-SLAM2
    FILE *pFile;
    std::string file_path = "../utility_python/pose_graph_test.txt";
    printf("lode pose graph from: %s \n", file_path.c_str());
    printf("pose graph loading...\n");
    std::printf("images.txt path: %s\n", IMAGES_TXT_SAVE_PATH.c_str());
    std::printf("images.txt saving... \n");
    pFile = fopen(file_path.c_str(), "r");
    if (pFile == NULL)
    {
        printf("load pose graph error: wrong pose graph path or no pose graph available \n");
        return;
    }

    int index = 0;
    double time_stamp;
    double PG_Tx, PG_Ty, PG_Tz;
    double PG_Qw, PG_Qx, PG_Qy, PG_Qz;

    while (fscanf(pFile, "%lf %lf %lf %lf %lf %lf %lf %lf",
                  &time_stamp,
                  &PG_Tx, &PG_Ty, &PG_Tz,
                  &PG_Qx, &PG_Qy, &PG_Qz, &PG_Qw) != EOF)
    {
        cv::Mat image;
        std::string image_path, descriptor_path;
        Eigen::Vector3d PG_T(PG_Tx, PG_Ty, PG_Tz);
        Eigen::Quaterniond PG_Q;
        PG_Q.w() = PG_Qw;
        PG_Q.x() = PG_Qx;
        PG_Q.y() = PG_Qy;
        PG_Q.z() = PG_Qz;
        Eigen::Matrix3d VIO_R, PG_R;
        PG_R = PG_Q.toRotationMatrix();

        Eigen::Matrix3d R_z_;
        R_z_.row(0) << 0, 1, 0;
        R_z_.row(1) << -1, 0, 0;
        R_z_.row(2) << 0, 0, 1;

        Eigen::Quaterniond PG_Q_(R_z_ * PG_R.transpose());
        Eigen::Vector3d PG_T_;
        PG_T_ = -R_z_ * PG_R.transpose() * PG_T;

        // save images.txt for COLMAP
        vins_PoseGraph_reader::saveImages_txt_in_COLMAP_format(index, PG_T_, PG_Q_);
        index += 1;
    }
    fclose(pFile);

    // save cameras.txt and points3D.txt for COLMAP
    vins_PoseGraph_reader::saveCameras_txt_in_COLMAP_format();
    vins_PoseGraph_reader::savePoints3D_txt_in_COLMAP_format();
}

void orbslam2_PoseGraph_reader::test_pg()
{
    // Read pose graph data from *.txt, which was generated from ORB-SLAM2
    FILE *pFile;
    // std::string file_path = POSE_GRAPH_SAVE_PATH + "KeyFrameTrajectory.txt";
    std::string file_path = "../utility_python/pose_graph_test_orb.txt";
    printf("lode pose graph from: %s \n", file_path.c_str());
    printf("pose graph loading...\n");
    std::printf("images.txt path: %s\n", IMAGES_TXT_SAVE_PATH.c_str());
    std::printf("images.txt saving... \n");
    pFile = fopen(file_path.c_str(), "r");
    if (pFile == NULL)
    {
        printf("load pose graph error: wrong pose graph path or no pose graph available \n");
        return;
    }

    int index = 0;
    double time_stamp;
    double PG_Tx, PG_Ty, PG_Tz;
    double PG_Qw, PG_Qx, PG_Qy, PG_Qz;

    while (fscanf(pFile, "%lf %lf %lf %lf %lf %lf %lf %lf",
                  &time_stamp,
                  &PG_Tx, &PG_Ty, &PG_Tz,
                  &PG_Qx, &PG_Qy, &PG_Qz, &PG_Qw) != EOF)
    {
        cv::Mat image;
        std::string image_path, descriptor_path;
        Eigen::Vector3d PG_T(PG_Tx, PG_Ty, PG_Tz);
        Eigen::Quaterniond PG_Q;
        PG_Q.w() = PG_Qw;
        PG_Q.x() = PG_Qx;
        PG_Q.y() = PG_Qy;
        PG_Q.z() = PG_Qz;
        Eigen::Matrix3d VIO_R, PG_R;
        PG_R = PG_Q.toRotationMatrix();

        Eigen::Matrix3d R_3d_transform;
        R_3d_transform.row(0) << -0.39517691, -0.40363569, 0.82517479;
        R_3d_transform.row(1) << -0.91840957, 0.19213422, -0.34584434;
        R_3d_transform.row(2) << -0.01894919, -0.89451812, -0.44662989;

        Eigen::Quaterniond PG_Q_((R_3d_transform * PG_R).transpose());
        Eigen::Vector3d PG_T_;
        PG_T_ = -(R_3d_transform * PG_R).transpose() * PG_T;

        // save images.txt for COLMAP
        vins_PoseGraph_reader::saveImages_txt_in_COLMAP_format(index, PG_T_, PG_Q_);
        index += 1;
    }
    fclose(pFile);

    // save cameras.txt and points3D.txt for COLMAP
    vins_PoseGraph_reader::saveCameras_txt_in_COLMAP_format();
    vins_PoseGraph_reader::savePoints3D_txt_in_COLMAP_format();
}
