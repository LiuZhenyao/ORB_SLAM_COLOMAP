#include "read_binFile.h"


void vins_BinFile_reader::testFunc() {
    std::cout << "TEST FUNCTION RUNING -- sample namespace" << std::endl;
    
    cv::Mat image;
    image = cv::imread("/home/shu/Pictures/1.png", 1);
    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
    cv::imshow("Display Image", image);
    cv::waitKey(0);

}