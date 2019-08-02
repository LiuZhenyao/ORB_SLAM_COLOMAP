#pragma once

#include <iostream>

namespace JohnDeere_Mono 
{
    // Define camera info for John Deere crossed view stereo camera pair
    // Calibration should be done beforehand
    const int CAMERA_ID = 1; // this ID will be written into cameras.txt file for COLMAP use, leave it always = 1 for monocular case
    const int IMG_WIDTH = 1280;
    const int IMG_HEIGHT = 800;
    const std::string CAMERA_MODEL = "OPENCV"; //SIMPLE_RADIAL, SIMPLE_PINHOLE, PINHOLE, RADIAL, OPENCV


    // This namespace only consider monocular case, choose the camera id you use
    //      sensor 5, slot 0
    const float fx = 9.1246540196702676e+02; // in pixel
    const float fy = 9.0908065486055887e+02;
    const float cx = 6.6656196310428618e+02; // principle point position in pixel
    const float cy = 3.7429248627169471e+02;
    // //      sensor 5, slot 1
    // const float fx = 9.1328804283838451e+02;
    // const float fy = 9.1016357654213766e+02;
    // const float cx = 6.5599695521162744e+02;
    // const float cy = 3.9819314264033534e+02;
    // //  sensor 6, slot 0
    // const float fx = 9.2323868309385364e+02;
    // const float fy = 9.1961577406117192e+02;
    // const float cx = 6.7038580219474363e+02;
    // const float cy = 3.8723317235022847e+02;
    // //      sensor 6, slot 1
    // const float fx = 9.2525206025617308e+02;
    // const float fy = 9.2135722741509221e+02;
    // const float cx = 6.6418193933398516e+02;
    // const float cy = 3.9153437691127681e+02;

    // Distortion parameters, here, the image is already distorted!
    const float p1 = 0;
    const float p2 = 0;
    const float k1 = 0;
    const float k2 = 0; 
}
