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
    const float fx = 440.8503716053656; // in pixel
    const float fy = 440.67694216951423;
    const float cx = 354.173182391866; // principle point position in pixel
    const float cy = 233.00424421090963;
    // Distortion parameters
    // const float p1 = -0.00027894166759601926;
    // const float p2 = 0.0007775925641113786;
    // const float k1 = -0.3205125886805923;
    // const float k2 = 0.10861100855243255;
    const float p1 = 0;
    const float p2 = 0;
    const float k1 = 0;
    const float k2 = 0;
}