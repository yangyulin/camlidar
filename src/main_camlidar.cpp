//
// Created by linde on 8/3/17.
//

#include <opencv/cv.hpp>
#include <gtsam/geometry/StereoPoint2.h>
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
    matLrect = CL::imageRect(imgNameL,fReader.get_K_0(),fReader.get_D_0(),
                             fReader.get_R_rect0(),fReader.get_P_rect0(), fReader.get_S_rect0());
    matRrect = CL::imageRect(imgNameR, fReader.get_K_1(), fReader.get_D_1(),
                             fReader.get_R_rect1(), fReader.get_P_rect1(), fReader.get_S_rect1());

    CL::showRect(imgNameL, imgNameR, matLrect, matRrect, true);

    std::vector<cv::Point3f> cvchessCorner1, cvchessCorner2, cvchessCorner3;

    std::vector<gtsam::StereoPoint2> chessCorner1 = CL::detectChessboard1(matLrect, matRrect,
                                                              fReader.get_patternSize1(), cvchessCorner1);
    std::vector<gtsam::StereoPoint2> chessCorner2 = CL::detectChessboard2(matLrect, matRrect,
                                                              fReader.get_patternSize2(), cvchessCorner2);
    std::vector<gtsam::StereoPoint2> chessCorner3 = CL::detectChessboard3(matLrect, matRrect,
                                                              fReader.get_patternSize3(), cvchessCorner3);

    gtsam::Pose3 T_G1_C, T_G2_C, T_G3_C;
    T_G1_C = CL::getTargetPose(chessCorner1,
                                    fReader.get_fx(), fReader.get_fy(), fReader.get_px(),fReader.get_py(),
                                    fReader.get_baseline(),
                                    fReader.get_map1());

    T_G2_C = CL::getTargetPose(chessCorner2,
                                    fReader.get_fx(), fReader.get_fy(), fReader.get_px(),fReader.get_py(),
                                    fReader.get_baseline(),
                                    fReader.get_map2());
    T_G3_C = CL::getTargetPose(chessCorner3,
                                    fReader.get_fx(), fReader.get_fy(), fReader.get_px(),fReader.get_py(),
                                    fReader.get_baseline(),
                                    fReader.get_map3());




    std::cout<<"The chessboard 1 Pose "<<std::endl;
    std::cout<<T_G1_C<<std::endl;
    std::cout<<"The chessboard 2 Pose "<<std::endl;
    std::cout<<T_G2_C<<std::endl;
    std::cout<<"The chessboard 3 Pose "<<std::endl;
    std::cout<<T_G3_C<<std::endl;



    // check the detection
    /*
    std::cout<<"Chessboard 1: "<<std::endl;
    for (int i = 0; i < chessCorner1.size(); ++i) {
        std::cout<<"The detection is "<<i<<"  ";
        std::cout<<chessCorner1[i]<<std::endl;
        std::cout<<"The map is " <<endl;
        std::cout<<fReader.get_map1()[i]<<std::endl;
    }
    std::cout<<"Chessboard 2: "<<std::endl;
    for (int i = 0; i < chessCorner2.size(); ++i) {
        std::cout<<"The detection is "<<i<<"  ";
        std::cout<<chessCorner2[i]<<std::endl;
    }
    */



    return 0;
}