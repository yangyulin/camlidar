//
// Created by linde on 8/3/17.
//

#include "dataprocess/initData.h"
#include "dataprocess/imgRect.h"

using namespace cv;
using namespace std;

/**
 * When reading from OpenCV, must be careful that the format of yaml and xml must be accurate
 * This is very important
 */

int main( int argc, char** argv ){

    string dir = "/home/linde/data/Project/camlidar/data/3plane2";
    CL::fileReader fReader(dir);

    string imgNameL = dir + "/" + "left-0000.JPG";
    string imgNameR = dir + "/" + "right-0000.JPG";

    cv::Mat matLrect, matRrect;
    matLrect = CL::imageRect(imgNameL,
                             fReader.get_K_0(),
                             fReader.get_D_0(),
                             fReader.get_R_rect0(),
                             fReader.get_P_rect0(),
                             fReader.get_S_rect0());
    matRrect = CL::imageRect(imgNameR,
                             fReader.get_K_1(),
                             fReader.get_D_1(),
                             fReader.get_R_rect0(),
                             fReader.get_P_rect1(),
                             fReader.get_S_rect1());

    CL::showRect(imgNameL, imgNameR, matLrect, matRrect, true);





    /*
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


    for(int i = 0; i < 47; i++){

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

    }*/


    return 0;
}