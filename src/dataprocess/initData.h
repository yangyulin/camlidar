//
// Created by linde on 8/3/17.
//

#ifndef CAMLIDAR_INITDATA_H
#define CAMLIDAR_INITDATA_H

#include <string>
#include <iostream>
#include <fstream>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <boost/thread.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv/cxeigen.hpp>

#include <gtsam/geometry/Pose3.h>

using namespace cv;
using namespace std;

namespace CL{


    class fileReader{

    protected:

        std::string dataDir;

        /// stereo calibration data

        cv::Mat K_0, D_0, R_rect0, P_rect0;
        cv::Mat K_1, D_1, R_rect1, P_rect1;

        cv::Size S_rect0, S_rect1;

        cv::Size patternSize1, patternSize2, patternSize3;
        double edgeLength;
        std::vector<cv::Point3d> map1, map2, map3;
        std::vector<gtsam::Point3> map1_gt, map2_gt, map3_gt;

        /// the extrinsic calbration init in Mat formate

        cv::Mat R_I_L_init_Mat, P_I_L_init_Mat;
        cv::Mat R_I_C_vi_init_Mat, P_I_C_vi_init_Mat;
        cv::Mat R_I_C_zed_init_Mat, P_I_C_zed_init_Mat;
        cv::Mat R_C_zed_L_init_Mat, P_C_zed_L_init_Mat;
        cv::Mat R_C_vi_L_init_Mat, P_C_vi_L_init_Mat;


        Eigen::Matrix<double, 3, 3> R_I_L_init;
        Eigen::Matrix<double, 3, 1> P_I_L_init;
        Eigen::Matrix<double, 3, 3> R_I_C_vi_init;
        Eigen::Matrix<double, 3, 1> P_I_C_vi_init;
        Eigen::Matrix<double, 3, 3> R_I_C_zed_init;
        Eigen::Matrix<double, 3, 1> P_I_C_zed_init;
        Eigen::Matrix<double, 3, 3> R_C_zed_L_init;
        Eigen::Matrix<double, 3, 1> P_C_zed_L_init;
        Eigen::Matrix<double, 3, 3> R_C_vi_L_init;
        Eigen::Matrix<double, 3, 1> P_C_vi_L_init;

        gtsam::Pose3 T_I_L_init;
        gtsam::Pose3 T_I_C_vi_init;
        gtsam::Pose3 T_I_C_zed_init;
        gtsam::Pose3 T_C_zed_L_init;
        gtsam::Pose3 T_C_vi_L_init;



    public:
        fileReader() {};
        fileReader(std::string dataDir);

        cv::Mat get_K_0();
        cv::Mat get_D_0();
        cv::Mat get_R_rect0();
        cv::Mat get_P_rect0();

        cv::Mat get_K_1();
        cv::Mat get_D_1();
        cv::Mat get_R_rect1();
        cv::Mat get_P_rect1();

        cv::Size get_S_rect0();
        cv::Size get_S_rect1();

        cv::Size get_patternSize1();
        cv::Size get_patternSize2();
        cv::Size get_patternSize3();

        std::vector<cv::Point3d> get_map1();
        std::vector<cv::Point3d> get_map2();
        std::vector<cv::Point3d> get_map3();


        Eigen::Matrix<double, 3, 3> get_R_I_L_init();
        Eigen::Matrix<double, 3, 1> get_P_I_L_init();
        Eigen::Matrix<double, 3, 3> get_R_I_C_vi_init();
        Eigen::Matrix<double, 3, 1> get_P_I_C_vi_init();
        Eigen::Matrix<double, 3, 3> get_R_I_C_zed_init();
        Eigen::Matrix<double, 3, 1> get_P_I_C_zed_init();
        Eigen::Matrix<double, 3, 3> get_R_C_zed_L_init();
        Eigen::Matrix<double, 3, 1> get_P_C_zed_L_init();
        Eigen::Matrix<double, 3, 3> get_R_C_vi_L_init();
        Eigen::Matrix<double, 3, 1> get_P_C_vi_L_init();



        gtsam::Pose3 get_T_I_L_init();
        gtsam::Pose3 get_T_I_C_vi_init();
        gtsam::Pose3 get_T_I_C_zed_init();
        gtsam::Pose3 get_T_C_zed_L_init();
        gtsam::Pose3 get_T_C_vi_L_init();

    };


}





#endif //CAMLIDAR_INITDATA_H
