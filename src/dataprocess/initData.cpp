//
// Created by linde on 8/3/17.
//

#include "initData.h"

using namespace CL;

cv::Mat CL::fileReader::get_K_0() {
    return K_0;
}

cv::Mat fileReader::get_D_0() {
    return D_0;
}

cv::Mat fileReader::get_R_rect0() {
    return R_rect0;
}

cv::Mat fileReader::get_P_rect0() {
    return P_rect0;
}

cv::Mat fileReader::get_K_1() {
    return K_1;
}

cv::Mat fileReader::get_D_1() {
    return D_1;
}

cv::Mat fileReader::get_R_rect1() {
    return R_rect1;
}

cv::Mat fileReader::get_P_rect1() {
    return P_rect1;
}

cv::Size fileReader::get_S_rect0() {
    return S_rect0;
}

cv::Size fileReader::get_S_rect1() {
    return S_rect1;
}

cv::Size fileReader::get_patternSize1() {
    return patternSize1;
}

cv::Size fileReader::get_patternSize2() {
    return patternSize2;
}

cv::Size fileReader::get_patternSize3() {
    return patternSize3;
}


Eigen::Matrix<double, 3, 3> fileReader::get_R_I_L_init() {
    return R_I_L_init;
}

Eigen::Matrix<double, 3, 1> fileReader::get_P_I_L_init() {
    return P_I_L_init;
}

Eigen::Matrix<double, 3, 3> fileReader::get_R_I_C_vi_init() {
    return R_I_C_vi_init;
}

Eigen::Matrix<double, 3, 1> fileReader::get_P_I_C_vi_init() {
    return P_I_C_vi_init;
}

Eigen::Matrix<double, 3, 3> fileReader::get_R_I_C_zed_init() {
    return R_I_C_zed_init;
}

Eigen::Matrix<double, 3, 1> fileReader::get_P_I_C_zed_init() {
    return P_I_C_zed_init;
}

Eigen::Matrix<double, 3, 3> fileReader::get_R_C_zed_L_init() {
    return R_C_zed_L_init;
}

Eigen::Matrix<double, 3, 1> fileReader::get_P_C_zed_L_init() {
    return P_C_zed_L_init;
}

Eigen::Matrix<double, 3, 3> fileReader::get_R_C_vi_L_init() {
    return R_C_vi_L_init;
}

Eigen::Matrix<double, 3, 1> fileReader::get_P_C_vi_L_init() {
    return P_C_vi_L_init;
}

gtsam::Pose3 fileReader::get_T_I_L_init() {
    return T_I_L_init;
}

gtsam::Pose3 fileReader::get_T_I_C_vi_init() {
    return T_I_C_vi_init;
}

gtsam::Pose3 fileReader::get_T_I_C_zed_init() {
    return T_I_C_zed_init;
}

gtsam::Pose3 fileReader::get_T_C_zed_L_init() {
    return T_C_zed_L_init;
}

gtsam::Pose3 fileReader::get_T_C_vi_L_init() {
    return T_C_vi_L_init;
}

fileReader::fileReader(std::string dataDir) {

    // first generate the file path
    std::string file_init = dataDir + "/"  + "init.yaml";
    std::string file_cam_left = dataDir + "/" + "left.yaml";
    std::string file_cam_right = dataDir + "/" + "right.yaml";

    // begin to read file
    cv::FileStorage dataCalibLeft(file_cam_left, cv::FileStorage::READ);
    cv::FileStorage dataCalibRight(file_cam_right, cv::FileStorage::READ);
    cv::FileStorage dataInit(file_init, cv::FileStorage::READ);

    if(!dataCalibLeft.isOpened() && !dataCalibRight.isOpened() ) {
        std::cerr << "Error, unable to open calibration file" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    if(!dataInit.isOpened()) {
        cerr << "Error, unable to open init.yaml file" << endl;
        exit(EXIT_FAILURE);
    }
    /// begin to read the data
    // read the camera data into mat file
    int cols_L = dataCalibLeft["image_width"];
    int rows_L = dataCalibLeft["image_height"];
    int cols_R = dataCalibRight["image_width"];
    int rows_R = dataCalibRight["image_height"];

    S_rect0.height = rows_L;
    S_rect0.width = cols_L;
    S_rect1.height = rows_R;
    S_rect1.width = cols_R;

    dataCalibLeft["camera_matrix"] >> K_0;
    dataCalibLeft["distortion_coefficients"] >> D_0;
    dataCalibLeft["rectification_matrix"] >> R_rect0;
    dataCalibLeft["projection_matrix"] >> P_rect0;
    dataCalibRight["camera_matrix"] >> K_1;
    dataCalibRight["distortion_coefficients"] >> D_1;
    dataCalibRight["rectification_matrix"] >> R_rect1;
    dataCalibRight["projection_matrix"] >> P_rect1;


    // read the pattern size
    int width1 = dataInit["Target1_width"];
    int height1 = dataInit["Target1_height"];
    patternSize1.width = width1;
    patternSize1.height = height1;

    int width2 = dataInit["Target2_width"];
    int height2 = dataInit["Target2_height"];
    patternSize2.width = width2;
    patternSize2.height = height2;

    int width3 = dataInit["Target3_width"];
    int height3 = dataInit["Target3_height"];
    patternSize3.width = width3;
    patternSize3.height = height3;


    // read the init data file
    dataInit["R_I_L_init"] >> R_I_L_init_Mat;
    dataInit["P_I_L_init"] >> P_I_L_init_Mat;

    dataInit["R_I_C_vi_init"] >> R_I_C_vi_init_Mat;
    dataInit["P_I_C_vi_init"] >> P_I_C_vi_init_Mat;

    dataInit["R_I_C_zed_init"] >> R_I_C_zed_init_Mat;
    dataInit["P_I_C_zed_init"] >> P_I_C_zed_init_Mat;

    dataInit["R_C_vi_L_init"] >> R_C_vi_L_init_Mat;
    dataInit["P_C_vi_L_init"] >> P_C_vi_L_init_Mat;

    dataInit["R_C_zed_L_init"] >> R_C_zed_L_init_Mat;
    dataInit["P_C_zed_L_init"] >> P_C_zed_L_init_Mat;

    // convert these data into the format of Eigen
    cv::cv2eigen(R_I_L_init_Mat, R_I_L_init);
    cv::cv2eigen(P_I_L_init_Mat, P_I_L_init);

    cv::cv2eigen(R_I_C_vi_init_Mat, R_I_C_vi_init);
    cv::cv2eigen(P_I_C_vi_init_Mat, P_I_C_vi_init);

    cv::cv2eigen(R_I_C_zed_init_Mat, R_I_C_zed_init);
    cv::cv2eigen(P_I_C_zed_init_Mat, P_I_C_zed_init);

    cv::cv2eigen(R_C_vi_L_init_Mat, R_C_vi_L_init);
    cv::cv2eigen(P_C_vi_L_init_Mat, P_C_vi_L_init);

    cv::cv2eigen(R_C_zed_L_init_Mat, R_C_zed_L_init);
    cv::cv2eigen(P_C_zed_L_init_Mat, P_C_zed_L_init);

    gtsam::Rot3 R_I_L_init_gt(R_I_L_init);
    gtsam::Point3 P_I_L_init_gt(P_I_L_init);
    T_I_L_init.Create(R_I_L_init_gt, P_I_L_init_gt);

    gtsam::Rot3 R_I_C_vi_init_gt(R_I_C_vi_init);
    gtsam::Point3 P_I_C_vi_init_gt(P_I_C_vi_init);
    T_I_C_vi_init.Create(R_I_C_vi_init_gt, P_I_C_vi_init_gt);

    gtsam::Rot3 R_I_C_zed_init_gt(R_I_C_zed_init);
    gtsam::Point3 P_I_C_zed_init_gt(P_I_C_zed_init);
    T_I_C_zed_init.Create(R_I_C_zed_init_gt, P_I_C_zed_init_gt);

    gtsam::Rot3 R_C_vi_L_init_gt(R_C_vi_L_init);
    gtsam::Point3 P_C_vi_L_init_gt(P_C_vi_L_init);
    T_C_vi_L_init.Create(R_C_vi_L_init_gt, P_C_vi_L_init_gt);

    gtsam::Rot3 R_C_zed_L_init_gt(R_C_zed_L_init);
    gtsam::Point3 P_C_zed_L_init_gt(P_C_zed_L_init);
    T_C_zed_L_init.Create(R_C_zed_L_init_gt, P_C_zed_L_init_gt);

}
