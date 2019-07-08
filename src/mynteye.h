#pragma once

#include <iostream>

namespace mynteye 
{
    // Define camera info for MYNT EYE S1030, only left one (monocular case)
    // Calibration should be done beforehand
    const int CAMERA_ID = 1; // just set index 1 to mynteye camera
    const int IMG_WIDTH = 752;
    const int IMG_HEIGHT = 480;
    const std::string CAMERA_MODEL = "OPENCV"; //SIMPLE_RADIAL, SIMPLE_PINHOLE, PINHOLE, RADIAL, OPENCV
    // Projection parameters
    const float fx = 439.12472698845284; // in pixel
    const float fy = 439.65762370518974;
    const float cx = 356.94016689742233; // principle point position in pixel
    const float cy = 231.07744714937556;
    // Distortion parameters, here, the image is already distorted!
    const float p1 = 0;
    const float p2 = 0;
    const float k1 = 0;
    const float k2 = 0;
}