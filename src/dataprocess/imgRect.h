//
// Created by linde on 8/3/17.
//

#ifndef CAMLIDAR_IMGRECT_H
#define CAMLIDAR_IMGRECT_H

#include <gtsam/geometry/StereoPoint2.h>
#include "initData.h"

using namespace std;
using namespace cv;

namespace CL{

        cv::Mat imageRect(std::string imgName,
                          cv::Mat K,
                          cv::Mat D,
                          cv::Mat R_rect,
                          cv::Mat P_rect,
                          cv::Size S_rect);

        void showRect(std::string imgNameL,
                         std::string imgNameR,
                         cv::Mat matLrect,
                         cv::Mat matRrect,
                         bool SHOW_RAW);

        std::vector<gtsam::StereoPoint2> detectChessboard( cv::Mat matLrect,
                                                            cv::Mat matRrect,
                                                            cv::Size patternSize1,
                                                            cv::Size patternSize2,
                                                            cv::Size PatternSize3);

        std::vector<gtsam::StereoPoint2> detectChessboard1(cv::Mat matLrect,
                                                            cv::Mat matRrect,
                                                            cv::Size patternSize1,
                                                            std::vector<cv::Point3f> cvdetectChessboard1);
        std::vector<gtsam::StereoPoint2> detectChessboard2(cv::Mat matLrect,
                                                            cv::Mat matRrect,
                                                            cv::Size patternSize2,
                                                           std::vector<cv::Point3f> cvdetectChessboard2);
        std::vector<gtsam::StereoPoint2> detectChessboard3(cv::Mat matLrect,
                                                            cv::Mat matRrect,
                                                            cv::Size patternSize3,
                                                           std::vector<cv::Point3f> cvdetectChessboard3);

        std::vector<cv::Point3f> backproject(std::vector<gtsam::StereoPoint2> targetCorner,
                                             double fx, double fy, double px, double py, double baseline);


        gtsam::Pose3 estimateTargetPose(std::vector<cv::Point3f> P_C_f, std::vector<cv::Point3f> P_G_f);

        gtsam::Pose3 getTargetPose(std::vector<gtsam::StereoPoint2> targetCorner,
                                        double fx, double fy, double px, double py, double baseline,
                                        std::vector<cv::Point3f> P_G_f);



};
#endif //CAMLIDAR_IMGRECT_H
