//
// Created by linde on 8/3/17.
//

#ifndef CAMLIDAR_IMGRECT_H
#define CAMLIDAR_IMGRECT_H

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

        std::vector<cv::Point2f> detectChessboard3(cv::Mat matrect,
                                                  cv::Size patternSize1,
                                                  cv::Size patternSize2,
                                                  cv::Size PatternSize3);

};



#endif //CAMLIDAR_IMGRECT_H
