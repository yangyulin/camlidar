//
// Created by linde on 8/3/17.
//

#include "imgRect.h"

using namespace cv;
using namespace std;



cv::Mat CL::imageRect(  std::string imgName,
                        cv::Mat K, cv::Mat D,
                        cv::Mat R_rect, cv::Mat P_rect,
                        cv::Size S_rect){

    // first read in the image
    cv::Mat img = imread(imgName,IMREAD_GRAYSCALE);
    cv::Mat mat1, mat2;

    // do the rectification
    cv::initUndistortRectifyMap(K, D, R_rect, P_rect, S_rect, CV_32F, mat1, mat2);
    cv::Mat matrect;
    cv::remap(img, matrect, mat1, mat2, INTER_LINEAR);

    return matrect;

}

void CL::showRect(   std::string imgNameL,
                        std::string imgNameR,
                        cv::Mat matLrect,
                        cv::Mat matRrect,
                        bool SHOW_RAW){

    // read in the original images
    if(SHOW_RAW){
        // first read in the raw images
        cv::Mat img0 = imread(imgNameL, IMREAD_GRAYSCALE);
        cv::Mat img1 = imread(imgNameR, IMREAD_GRAYSCALE);
        // Display nice image
        cv::Size s0 = img0.size();
        cv::Size s1 = img1.size();

        // Create the combined matrix
        cv::Mat im_raw(s0.height, s0.width+s1.width, CV_8UC1);

        cv::Mat left1(im_raw, cv::Rect(0, 0, s0.width, s0.height));
        img0.copyTo(left1);
        cv::Mat right1(im_raw, cv::Rect(s0.width, 0, s1.width, s1.height));
        img1.copyTo(right1);

        imshow("Vision Before Rectification", im_raw);
    }

    // Display nice image
    cv::Size sz0 = matLrect.size();
    cv::Size sz1 = matRrect.size();

    // Create the combined matrix
    cv::Mat im_rect(sz0.height, sz0.width+sz1.width+1, CV_8UC1);

    cv::Mat left1(im_rect, cv::Rect(0, 0, sz0.width, sz0.height));
    matLrect.copyTo(left1);
    cv::Mat right1(im_rect, cv::Rect(sz0.width, 0, sz1.width, sz1.height));
    matRrect.copyTo(right1);


    imshow("Vision After Rectification", im_rect);
    waitKey(0);

//return Mat();

}
