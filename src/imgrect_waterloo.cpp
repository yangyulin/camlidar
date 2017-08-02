//
// Created by root on 1/2/17.
//


#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>

using namespace cv;
using namespace std;

/**
 * When reading from OpenCV, must be careful that the format of yaml and xml must be accurate
 * This is very important
 */

int main( int argc, char** argv ){

    string dir = "/home/linde/project/imgrect/data/waterloo/";

    namedWindow("Vision Before Rectification");
    namedWindow("Vision After Rectification");

    //read in the config file for waterloo data set
    string setting_file_leftcam = dir + "left.yaml";
    string setting_file_rightcam = dir + "right.yaml";

    cv::FileStorage dataCalibLeft(setting_file_leftcam, cv::FileStorage::READ);
    cv::FileStorage dataCalibRight(setting_file_rightcam, cv::FileStorage::READ);

    if(!dataCalibLeft.isOpened() && !dataCalibRight.isOpened() ) {
        cerr << "Error, unable to open calibration file" << endl;
        exit(EXIT_FAILURE);
    }

    cv::Mat K_1, D_1, R_rect1, P_rect1;
    cv::Mat K_2, D_2, R_rect2, P_rect2;

    int cols_L = dataCalibLeft["image_width"];
    int rows_L = dataCalibLeft["image_height"];
    int cols_R = dataCalibRight["image_width"];
    int rows_R = dataCalibRight["image_height"];

    dataCalibLeft["camera_matrix"] >> K_1;
    dataCalibLeft["distortion_coefficients"] >> D_1;
    dataCalibLeft["rectification_matrix"] >> R_rect1;
    dataCalibLeft["projection_matrix"] >> P_rect1;
    dataCalibRight["camera_matrix"] >> K_2;
    dataCalibRight["distortion_coefficients"] >> D_2;
    dataCalibRight["rectification_matrix"] >> R_rect2;
    dataCalibRight["projection_matrix"] >> P_rect2;

    Size S_rect1(cols_L,rows_L);
    Size S_rect2(cols_R,rows_R);

    for(int i = 0; i < 74; i++){

        cout<<"the "<<i<<" iamge"<<endl;

        //input file names
        char base_name[256];
        sprintf(base_name, "%04d.png",i);
        string filename1 = dir + "left-" + base_name;
        string filename2 = dir + "right-" + base_name;

        Mat img1 = imread(filename1,IMREAD_GRAYSCALE);
        Mat img2 = imread(filename2,IMREAD_GRAYSCALE);

        Mat matL1, matL2;
        Mat matR1, matR2;

        cv::initUndistortRectifyMap(K_1, D_1, R_rect1, P_rect1, S_rect1, CV_32F, matL1, matL2);
        cv::initUndistortRectifyMap(K_2, D_2, R_rect2, P_rect2, S_rect2, CV_32F, matR1, matR2);

        Mat matLrect, matRrect;

        cv::remap(img1, matLrect, matL1, matL2, INTER_LINEAR);
        cv::remap(img2, matRrect, matR1, matR2, INTER_LINEAR);

        // Display nice image
        cv::Size sz1 = matLrect.size();
        cv::Size sz2 = matRrect.size();

        // Create the combined matrix
        cv::Mat im4(sz1.height, sz1.width+sz2.width+1, CV_8UC1);

        cv::Mat left1(im4, cv::Rect(0, 0, sz1.width, sz1.height));
        img1.copyTo(left1);
        cv::Mat right1(im4, cv::Rect(sz1.width, 0, sz2.width, sz2.height));
        img2.copyTo(right1);

        imshow("Vision Before Rectification", im4);

        // Create combined matrix
        cv::Mat im3(sz1.height, sz1.width+sz2.width+1, CV_8UC1);

        cv::Mat left(im3, cv::Rect(0, 0, sz1.width, sz1.height));
        matLrect.copyTo(left);
        cv::Mat right(im3, cv::Rect(sz1.width, 0, sz2.width, sz2.height));
        matRrect.copyTo(right);

        imshow("Vision After Rectification", im3);

        waitKey(0);
        //usleep(2000);

        im3.release();
        img1.release();
        img2.release();
        matLrect.release();
        matRrect.release();
        matL1.release();
        matL2.release();
        matR1.release();
        matR2.release();

    }


return 0;
}

//hard-coded configuration data
/*
Mat K_1 = (Mat_<double>(3,3) << 652.391460, 0.000000, 457.341596,
                                0.000000, 652.071465, 325.524184,
                                0.000000, 0.000000, 1.000000);

Mat D_1 = (Mat_<double>(1,5) << -0.213114, 0.069305, 0.000911, 0.000970, 0.000000);

Mat R_rect1 = (Mat_<double>(3,3) << 0.997430, 0.004947, -0.071473,
                                    -0.004523, 0.999971, 0.006096,
                                    0.071501, -0.005757, 0.997424);

Mat P_rect1 = (Mat_<double>(3,4) << 642.715797, 0.000000, 486.408535, 0.000000,
                                    0.000000, 642.715797, 316.962208, 0.000000,
                                    0.000000, 0.000000, 1.000000, 0.000000);

Mat K_2 = (Mat_<double>(3,3) << 655.781575, 0.000000, 452.371020,
                                0.000000, 655.569496, 306.659392,
                                0.000000, 0.000000, 1.000000);



Mat R_rect2 = (Mat_<double>(3,3) << 0.999914, 0.012610, 0.003556,
                                    -0.012589, 0.999903, -0.005953,
                                    -0.003631, 0.005907, 0.999976);



Mat P_rect2 = (Mat_<double>(3,4) << 642.715797, 0.000000, 486.408535, -647.671716,
                                    0.000000, 642.715797, 316.962208, 0.000000,
                                    0.000000, 0.000000, 1.000000, 0.000000);


Mat D_2 = (Mat_<double>(1,5) << -0.230124, 0.103053, -0.000446, -0.001376, 0.000000);


double cols = 900;
double rows = 600;
 */