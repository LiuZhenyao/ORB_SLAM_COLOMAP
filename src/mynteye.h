#pragma once

#include <iostream>

namespace mynteye 
{
    // Define camera info for MYNT EYE S1030, only left one (monocular case)
    // Calibration should be done beforehand
    const int CAMERA_ID = 1; // just set index 1 to mynteye camera
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
    const float p1 = 0;
    const float p2 = 0;
    const float k1 = 0;
    const float k2 = 0;   
}