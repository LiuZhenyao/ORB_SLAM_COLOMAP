#pragma once

#include <iostream>

namespace JohnDeere_Mono 
{
    // Define camera info for John Deere crossed view stereo camera pair
    // Calibration should be done beforehand
    const int CAMERA_ID = 1; // this ID will be written into cameras.txt file for COLMAP use, leave it always = 1 for monocular case// just set index 1 to mynteye camera
    const int IMG_WIDTH = 672;
    const int IMG_HEIGHT = 376;
    const std::string CAMERA_MODEL = "OPENCV"; //SIMPLE_RADIAL, SIMPLE_PINHOLE, PINHOLE, RADIAL, OPENCV
    // Projection parameters
    const float fx = 348.522264; // in pixel
    const float fy = 348.449870;
    const float cx = 344.483596; // principle point position in pixel
    const float cy = 188.808062;
    // Distortion parameters, here, the image is already distorted!
    // see ./config/mynteye_s_config.yaml
    const float p1 = -0.000359;
    const float p2 = 0.000457;
    const float k1 = -0.174497;
    const float k2 = 0.027127;   
}