//
// Created by linde on 8/3/17.
//

#include <opencv/cv.hpp>
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

std::vector<gtsam::StereoPoint2> CL::detectChessboard( cv::Mat matLrect,
                                                        cv::Mat matRrect,
                                                cv::Size patternSize1,
                                                cv::Size patternSize2,
                                                cv::Size patternSize3){

    std::vector<gtsam::StereoPoint2> targetCorner;
    //
    std::vector<cv::Point2f> chessBoardL1;
    std::vector<cv::Point2f> chessBoardL2;
    std::vector<cv::Point2f> chessBoardL3;


    // we use a stupid method to detect the chessboard, here we know there three chessboard
    cv::Size sz = matRrect.size();
    cv::Mat left(matRrect, cv::Rect(0,0, sz.width/2, sz.height));
    bool TARGET_1 = cv::findChessboardCorners(left,
                                              patternSize1,
                                              chessBoardL1,
                                              CALIB_CB_ADAPTIVE_THRESH
                                              );

    if(TARGET_1)
        cornerSubPix(left,  chessBoardL1, Size(11, 11), Size(-1, -1),
                     TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    else
        std::cout<<"Fail to detect chessboard 1!!!"<<std::endl;


    return targetCorner;

}

std::vector<gtsam::StereoPoint2> CL::detectChessboard1(cv::Mat matLrect,
                                                        cv::Mat matRrect,
                                                       cv::Size patternSize1,
                                                       std::vector<cv::Point3f> cvdetectChessboard1){
    // prepare the output vector
    std::vector<gtsam::StereoPoint2> targetCorner;
    std::vector<cv::Point2f> chessBoardL1;
    std::vector<cv::Point2f> chessBoardR1;

    // detect the left image
    cv::Size szL = matLrect.size();
    cv::Mat imgL(matLrect, cv::Rect(0,0, szL.width/2, szL.height));
    bool TARGET_L1 = cv::findChessboardCorners(imgL,
                                              patternSize1,
                                              chessBoardL1,
                                              CALIB_CB_ADAPTIVE_THRESH
                                              + CALIB_CB_NORMALIZE_IMAGE
                                              + CALIB_CB_FAST_CHECK);

    if(TARGET_L1)
        cornerSubPix(matLrect,  chessBoardL1, Size(5, 5), Size(-1, -1),
                     TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    else
        std::cout<<"Fail to detect chessboard L1 !!!"<<std::endl;

    // detect the right image
    //TODO improve the chessboard detection part
    /// this detection is cheating, I need to improve this part
    cv::Size szR = matRrect.size();
    cv::Mat imgR(matRrect, cv::Rect(0,0, szR.width/7*3, szR.height));

    bool TARGET_R1 = cv::findChessboardCorners(imgR,
                                               patternSize1,
                                               chessBoardR1,
                                               CALIB_CB_ADAPTIVE_THRESH
                                               + CALIB_CB_NORMALIZE_IMAGE
                                               + CALIB_CB_FAST_CHECK);

    if(TARGET_R1)
        cornerSubPix(matRrect,  chessBoardR1, Size(5, 5), Size(-1, -1),
                     TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    else
        std::cout<<"Fail to detect chessboard R1 !!!"<<std::endl;

    // show the matches
    std::vector<cv::KeyPoint> chessKeyL1, chessKeyR1;
    cv::KeyPoint::convert(chessBoardL1,chessKeyL1);
    cv::KeyPoint::convert(chessBoardR1,chessKeyR1);
    // creat the matches
    std::vector<cv::DMatch> matches;
    for (int i = 0; i < chessBoardL1.size(); ++i) {
        matches.push_back(cv::DMatch(i,i, 0));
    }
    Mat img_matches;
    drawMatches( matLrect, chessKeyL1, matRrect, chessKeyR1,
                 matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                 vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    imshow("Matched ChessBoard", img_matches);
    waitKey(0);

    // store the stereo keypoints and output
    for (int j = 0; j < chessBoardL1.size() ; ++j) {
        float u1 = chessBoardL1[j].x;
        float u2 = chessBoardR1[j].x;
        float v1 = chessBoardL1[j].y;
        targetCorner.push_back(gtsam::StereoPoint2(u1,u2,v1));
        cvdetectChessboard1.push_back(cv::Point3f(u1,u2,v1));
    }

    return targetCorner;

}

std::vector<gtsam::StereoPoint2> CL::detectChessboard2(cv::Mat matLrect,
                                                        cv::Mat matRrect,
                                                       cv::Size patternSize2,
                                                       std::vector<cv::Point3f> cvdetectChessboard2){


    // prepare the output vector
    std::vector<gtsam::StereoPoint2> targetCorner;
    std::vector<cv::Point2f> chessBoardL2;
    std::vector<cv::Point2f> chessBoardR2;

    // detect the left image
    cv::Size szL = matLrect.size();
    cv::Mat imgL(matLrect, cv::Rect(szL.width/3,0, szL.width/2, szL.height));
    bool TARGET_L2 = cv::findChessboardCorners(imgL,
                                               patternSize2,
                                               chessBoardL2,
                                               CALIB_CB_ADAPTIVE_THRESH
                                               + CALIB_CB_NORMALIZE_IMAGE
                                               + CALIB_CB_FAST_CHECK);

    // recover the true pattern
    for(size_t iter = 0; iter < chessBoardL2.size(); ++iter) {
        chessBoardL2[iter].x = chessBoardL2[iter].x + szL.width/3;
    }

    if(TARGET_L2)
        cornerSubPix(matLrect,  chessBoardL2, Size(5, 5), Size(-1, -1),
                     TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    else
        std::cout<<"Fail to detect chessboard L2 !!!"<<std::endl;

    // detect the right image
    //TODO improve the chessboard detection part
    /// this detection is cheating, I need to improve this part
    cv::Size szR = matRrect.size();
    cv::Mat imgR(matRrect, cv::Rect(szL.width/3,0, szR.width/2, szR.height));

    bool TARGET_R2 = cv::findChessboardCorners(imgR,
                                               patternSize2,
                                               chessBoardR2,
                                               CALIB_CB_ADAPTIVE_THRESH
                                               + CALIB_CB_NORMALIZE_IMAGE
                                               + CALIB_CB_FAST_CHECK);
    // recover the true pattern
    for(size_t iter = 0; iter < chessBoardR2.size(); ++iter) {
        chessBoardR2[iter].x = chessBoardR2[iter].x + szL.width/3;
    }

    if(TARGET_R2)
        cornerSubPix(matRrect,  chessBoardR2, Size(5, 5), Size(-1, -1),
                     TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    else
        std::cout<<"Fail to detect chessboard R2 !!!"<<std::endl;

    // show the matches
    std::vector<cv::KeyPoint> chessKeyL2, chessKeyR2;
    cv::KeyPoint::convert(chessBoardL2,chessKeyL2);
    cv::KeyPoint::convert(chessBoardR2,chessKeyR2);
    // creat the matches
    std::vector<cv::DMatch> matches;
    for (int i = 0; i < chessBoardL2.size(); ++i) {
        matches.push_back(cv::DMatch(i,i, 0));
    }
    Mat img_matches;
    drawMatches( matLrect, chessKeyL2, matRrect, chessKeyR2,
                 matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                 vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    imshow("Matched ChessBoard", img_matches);
    waitKey(0);

    // store the stereo keypoints and output
    for (int j = 0; j < chessBoardL2.size() ; ++j) {
        float u1 = chessBoardL2[j].x;
        float u2 = chessBoardR2[j].x;
        float v1 = chessBoardL2[j].y;
        targetCorner.push_back(gtsam::StereoPoint2(u1,u2,v1));
        cvdetectChessboard2.push_back(cv::Point3f(u1,u2,v1));
    }

    return targetCorner;


}

std::vector<gtsam::StereoPoint2> CL::detectChessboard3(cv::Mat matLrect,
                                                        cv::Mat matRrect,
                                                        cv::Size patternSize3,
                                                       std::vector<cv::Point3f> cvdetectChessboard3){



    // prepare the output vector
    std::vector<gtsam::StereoPoint2> targetCorner;
    std::vector<cv::Point2f> chessBoardL3;
    std::vector<cv::Point2f> chessBoardR3;

    // detect the left image
    cv::Size szL = matLrect.size();
    cv::Mat imgL(matLrect, cv::Rect(szL.width/2,0, szL.width/2, szL.height));
    bool TARGET_L3 = cv::findChessboardCorners(imgL,
                                               patternSize3,
                                               chessBoardL3,
                                               CALIB_CB_ADAPTIVE_THRESH
                                               + CALIB_CB_NORMALIZE_IMAGE
                                               + CALIB_CB_FAST_CHECK);

    // recover the true pattern
    for(size_t iter = 0; iter < chessBoardL3.size(); ++iter) {
        chessBoardL3[iter].x = chessBoardL3[iter].x + szL.width/2;
    }

    if(TARGET_L3)
        cornerSubPix(matLrect,  chessBoardL3, Size(5, 5), Size(-1, -1),
                     TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    else
        std::cout<<"Fail to detect chessboard L3 !!!"<<std::endl;

    // detect the right image
    //TODO improve the chessboard detection part
    /// this detection is cheating, I need to improve this part
    cv::Size szR = matRrect.size();
    cv::Mat imgR(matRrect, cv::Rect(szL.width/2,0, szR.width/2, szR.height));

    bool TARGET_R3 = cv::findChessboardCorners(imgR,
                                               patternSize3,
                                               chessBoardR3,
                                               CALIB_CB_ADAPTIVE_THRESH
                                               + CALIB_CB_NORMALIZE_IMAGE
                                               + CALIB_CB_FAST_CHECK);
    // recover the true pattern
    for(size_t iter = 0; iter < chessBoardR3.size(); ++iter) {
        chessBoardR3[iter].x = chessBoardR3[iter].x + szL.width/2;
    }

    if(TARGET_R3)
        cornerSubPix(matRrect,  chessBoardR3, Size(5, 5), Size(-1, -1),
                     TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    else
        std::cout<<"Fail to detect chessboard R2 !!!"<<std::endl;

    // show the matches
    std::vector<cv::KeyPoint> chessKeyL3, chessKeyR3;
    cv::KeyPoint::convert(chessBoardL3,chessKeyL3);
    cv::KeyPoint::convert(chessBoardR3,chessKeyR3);
    // creat the matches
    std::vector<cv::DMatch> matches;
    for (int i = 0; i < chessBoardL3.size(); ++i) {
        matches.push_back(cv::DMatch(i,i, 0));
    }
    Mat img_matches;
    drawMatches( matLrect, chessKeyL3, matRrect, chessKeyR3,
                 matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                 vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    imshow("Matched ChessBoard", img_matches);
    waitKey(0);

    // store the stereo keypoints and output
    for (int j = 0; j < chessBoardL3.size() ; ++j) {
        float u1 = chessBoardL3[j].x;
        float u2 = chessBoardR3[j].x;
        float v1 = chessBoardL3[j].y;
        targetCorner.push_back(gtsam::StereoPoint2(u1,u2,v1));
        cvdetectChessboard3.push_back(cv::Point3f(u1,u2,v1));
    }

    return targetCorner;


}

std::vector<cv::Point3f> CL::backproject(std::vector<gtsam::StereoPoint2> targetCorner,
                                        double fx, double fy, double px, double py, double baseline){
    std::vector<cv::Point3f> P_C_f;
    size_t psz = targetCorner.size();
    for (int i = 0; i < psz; ++i) {
        double u1 = targetCorner[i].uL();
        double u2 = targetCorner[i].uR();
        double v = targetCorner[i].v();

        double u1n = (u1 - px) / fx;
        double u2n = (u2 - px) / fx;
        double vn = (v - py) / fy;

        double z = baseline * fx / (u1 - u2);
        double x = z * (u1 - px) / fx;
        double y = z * (v - py) / fy;

        P_C_f.push_back(cv::Point3f(x,y,z));
    }


    return P_C_f;

}

gtsam::Pose3 CL::estimateTargetPose(std::vector<cv::Point3f> P_C_f, std::vector<cv::Point3f> P_G_f) {

    cv::Mat T_G_C_cv, inlier;
    cv::estimateAffine3D(P_G_f, P_C_f, T_G_C_cv, inlier);


    std::cout<<"The output of the transformation "<<std::endl;
    std::cout<<T_G_C_cv<<std::endl;
    //std::cout<<inlier<<std::endl;

    Eigen::Matrix<double, 3, 4> T_G_C_eigen;
    cv::cv2eigen(T_G_C_cv, T_G_C_eigen);
    gtsam::Vector3 vec1 = T_G_C_eigen.block(0,0,3,1);
    gtsam::Vector3 vec2 = T_G_C_eigen.block(0,1,3,1);
    gtsam::Vector3 vec3 = vec1.cross(vec2);

    T_G_C_eigen.block(0,2,3,1) = vec3;

    std::cout<<"Test"<<endl;
    std::cout<<T_G_C_eigen.block(0,0,3,3) * T_G_C_eigen.block(0,0,3,3).transpose()<<endl;

    std::cout<<"Test 1"<<endl;
    Eigen::Vector3d vec11(P_C_f[1].x, P_C_f[1].y,P_C_f[1].z);
    Eigen::Vector3d vec12(P_C_f[7].x, P_C_f[7].y,P_C_f[7].z);
    Eigen::Vector3d vec13 = vec11.cross(vec12);
    Eigen::Matrix<double, 3, 3> R;
    //R.block()

    gtsam::Rot3 R_G_C(T_G_C_eigen.block(0,0,3,3));
    gtsam::Point3 P_G_C(T_G_C_eigen.block(0,3,3,1));

    gtsam::Pose3 T_G_C(R_G_C, P_G_C);



    return T_G_C;
}

gtsam::Pose3 CL::getTargetPose(std::vector<gtsam::StereoPoint2> targetCorner,
                           double fx, double fy, double px, double py,
                           double baseline, std::vector<cv::Point3f> P_G_f) {
    std::vector<cv::Point3f> P_C_f;
    P_C_f = CL::backproject(targetCorner, fx, fy, px, py, baseline);

    gtsam::Pose3 T_G_C = CL::estimateTargetPose(P_C_f, P_G_f);

    return T_G_C;
}
