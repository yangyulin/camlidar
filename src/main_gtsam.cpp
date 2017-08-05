//
// Created by linde on 8/3/17.
//
/* ----------------------------------------------------------------------------

/**
 * A 3D stereo visual odometry example
 *  - robot starts at origin
 *  -moves forward 1 meter
 *  -takes stereo readings on three landmarks
 */

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/StereoFactor.h>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <boost/thread.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv/cxeigen.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv/cv.hpp>

using namespace std;
using namespace gtsam;

int main(int argc, char** argv){


    cv::Mat C = (cv::Mat_<double>(2,2) << 0, -1, 0, -1);


    /// test the affine3D
    std::vector<cv::Point3f> src;
    cv::Point3f pt1(1,0, 0);
    cv::Point3f pt2(0,1, 0);
    cv::Point3f pt3(0,0, 1);
    cv::Point3f pt11(0,0, 0);
    src.push_back(pt1);
    src.push_back(pt2);
    src.push_back(pt3);
    src.push_back(pt11);

    std::vector<cv::Point3f> dst;
    cv::Point3f pt4(2,0, 0);
    cv::Point3f pt5(0,1, 0);
    cv::Point3f pt6(0,0, 1);
    cv::Point3f pt22(1,0, 0);
    dst.push_back(pt4);
    dst.push_back(pt5);
    dst.push_back(pt6);
    dst.push_back(pt22);

    cv::Mat affine;
    cv::Mat inlier;
    cv::estimateAffine3D(src, dst, affine, inlier);

    std::cout<<"the transformation is "<<std::endl;
    std::cout<<affine<<std::endl<<std::endl;


    /// test out the Point3
    Eigen::Matrix<double, 3, 1> P_ei(3,1,1);
    //gtsam::Point3 p(P_ei(0,0), P_ei(2,0), P_ei(1,0));
    gtsam::Point3 p = P_ei;
    std::cout<<p<<std::endl;

    /// Test out the Eigen and Mat conversion

    Eigen::Matrix<double, 3, 3> AE = I_3x3;

    cv::Mat AM = cv::Mat::eye(3, 3, CV_64F);

    //Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> A_Eigen(AM.ptr<double>(), AM.rows, AM.cols);

    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> A_Eigen(AM.ptr<double>(), AM.rows, AM.cols);

    Eigen::Matrix<double, 3, 3> ei_dst;
    cv::cv2eigen(AM, ei_dst);

    std::cout<<A_Eigen<<std::endl;

    std::cout<<ei_dst<<std::endl;

    //Eigen::Map<Eigen::Matrix<double, 3, 3>, Eigen::RowMajor> eigen_AM(AM.data);


    //create graph object, add first pose at origin with key '1'
    NonlinearFactorGraph graph;
    Pose3 first_pose;
    graph.emplace_shared<NonlinearEquality<Pose3> >(1, Pose3());

    //create factor noise model with 3 sigmas of value 1
    const noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Sigma(3,1);
    //create stereo camera calibration object with .2m between cameras
    const Cal3_S2Stereo::shared_ptr K(new Cal3_S2Stereo(1000, 1000, 0, 320, 240, 0.2));

    //create and add stereo factors between first pose (key value 1) and the three landmarks
    graph.emplace_shared<GenericStereoFactor<Pose3,Point3> >(StereoPoint2(520, 480, 440), model, 1, 3, K);
    graph.emplace_shared<GenericStereoFactor<Pose3,Point3> >(StereoPoint2(120, 80, 440), model, 1, 4, K);
    graph.emplace_shared<GenericStereoFactor<Pose3,Point3> >(StereoPoint2(320, 280, 140), model, 1, 5, K);

    //create and add stereo factors between second pose and the three landmarks
    graph.emplace_shared<GenericStereoFactor<Pose3,Point3> >(StereoPoint2(570, 520, 490), model, 2, 3, K);
    graph.emplace_shared<GenericStereoFactor<Pose3,Point3> >(StereoPoint2(70, 20, 490), model, 2, 4, K);
    graph.emplace_shared<GenericStereoFactor<Pose3,Point3> >(StereoPoint2(320, 270, 115), model, 2, 5, K);

    //create Values object to contain initial estimates of camera poses and landmark locations
    Values initial_estimate;

    //create and add iniital estimates
    initial_estimate.insert(1, first_pose);
    initial_estimate.insert(2, Pose3(Rot3(), Point3(0.1, -0.1, 1.1)));
    initial_estimate.insert(3, Point3(1, 1, 5));
    initial_estimate.insert(4, Point3(-1, 1, 5));
    initial_estimate.insert(5, Point3(0, -0.5, 5));

    //create Levenberg-Marquardt optimizer for resulting factor graph, optimize
    LevenbergMarquardtOptimizer optimizer(graph, initial_estimate);
    Values result = optimizer.optimize();

    //result.print("Final result:\n");

    return 0;
}