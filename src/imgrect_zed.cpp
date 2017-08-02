//
// Created by rpng on 7/26/16.
// Delete debug information 11/07/16
//

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <yaml-cpp/yaml.h>
#include <boost/lexical_cast.hpp>

#include <iostream>
#include <sstream>
#include <string>



using namespace cv;
using namespace std;

/**
 * learn how to read config from txt file with YAML-cpp lib
 * be careful to check the data format. a tiny error will cause the crash
 * @param argc
 * @param argv
 * @return
 */


int main( int argc, char** argv ){

    // read the images
    string filename1 = "/home/linde/project/imgrect/data/zed/001.png";
    string filename2 = "/home/linde/project/imgrect/data/zed/002.png";

    Mat img1,img2;
    img1 = imread(filename1,IMREAD_GRAYSCALE);
    img2 = imread(filename2,IMREAD_GRAYSCALE);

    imshow("before Left", img1);
    imshow("before Right", img2);

    //try to use YAML-cpp to read the config file

    string dir = "/home/linde/project/imgrect/data/zed/";

    string basename = "calib_cam_to_cam.txt";

    string config_file = dir + basename;
    YAML::Node calib_node;

    if (boost::filesystem::exists(config_file)){
        YAML::Node temp = YAML::LoadFile(config_file);
        for(YAML::const_iterator it = temp.begin(); it != temp.end(); ++it){
            std::string str;
            stringstream s(it->second.as<std::string>());
            while(s >> str){
                //cout<<str<<endl;
                double val = stod(str.c_str());
                calib_node[it->first.as<std::string>()].push_back(val);
            }
        }
    }

    Mat K_1 = (Mat_<double>(3,3) << calib_node["K_00"][0].as<double>(),
                                    calib_node["K_00"][1].as<double>(),
                                    calib_node["K_00"][2].as<double>(),
                                    calib_node["K_00"][3].as<double>(),
                                    calib_node["K_00"][4].as<double>(),
                                    calib_node["K_00"][5].as<double>(),
                                    calib_node["K_00"][6].as<double>(),
                                    calib_node["K_00"][7].as<double>(),
                                    calib_node["K_00"][8].as<double>());


    Mat K_2 = (Mat_<double>(3,3) << calib_node["K_01"][0].as<double>(),
                                    calib_node["K_01"][1].as<double>(),
                                    calib_node["K_01"][2].as<double>(),
                                    calib_node["K_01"][3].as<double>(),
                                    calib_node["K_01"][4].as<double>(),
                                    calib_node["K_01"][5].as<double>(),
                                    calib_node["K_01"][6].as<double>(),
                                    calib_node["K_01"][7].as<double>(),
                                    calib_node["K_01"][8].as<double>());

    Mat R_rect1 = (Mat_<double>(3,3) << calib_node["R_rect_00"][0].as<double>(),
                                        calib_node["R_rect_00"][1].as<double>(),
                                        calib_node["R_rect_00"][2].as<double>(),
                                        calib_node["R_rect_00"][3].as<double>(),
                                        calib_node["R_rect_00"][4].as<double>(),
                                        calib_node["R_rect_00"][5].as<double>(),
                                        calib_node["R_rect_00"][6].as<double>(),
                                        calib_node["R_rect_00"][7].as<double>(),
                                        calib_node["R_rect_00"][8].as<double>());

    Mat R_rect2 = (Mat_<double>(3,3) << calib_node["R_rect_01"][0].as<double>(),
                                        calib_node["R_rect_01"][1].as<double>(),
                                        calib_node["R_rect_01"][2].as<double>(),
                                        calib_node["R_rect_01"][3].as<double>(),
                                        calib_node["R_rect_01"][4].as<double>(),
                                        calib_node["R_rect_01"][5].as<double>(),
                                        calib_node["R_rect_01"][6].as<double>(),
                                        calib_node["R_rect_01"][7].as<double>(),
                                        calib_node["R_rect_01"][8].as<double>());

    Mat P_rect1 = (Mat_<double>(3,4) << calib_node["P_rect_00"][ 0].as<double>(),
                                        calib_node["P_rect_00"][ 1].as<double>(),
                                        calib_node["P_rect_00"][ 2].as<double>(),
                                        calib_node["P_rect_00"][ 3].as<double>(),
                                        calib_node["P_rect_00"][ 4].as<double>(),
                                        calib_node["P_rect_00"][ 5].as<double>(),
                                        calib_node["P_rect_00"][ 6].as<double>(),
                                        calib_node["P_rect_00"][ 7].as<double>(),
                                        calib_node["P_rect_00"][ 8].as<double>(),
                                        calib_node["P_rect_00"][ 9].as<double>(),
                                        calib_node["P_rect_00"][10].as<double>(),
                                        calib_node["P_rect_00"][11].as<double>());

    Mat P_rect2 = (Mat_<double>(3,4) << calib_node["P_rect_01"][ 0].as<double>(),
                                        calib_node["P_rect_01"][ 1].as<double>(),
                                        calib_node["P_rect_01"][ 2].as<double>(),
                                        calib_node["P_rect_01"][ 3].as<double>(),
                                        calib_node["P_rect_01"][ 4].as<double>(),
                                        calib_node["P_rect_01"][ 5].as<double>(),
                                        calib_node["P_rect_01"][ 6].as<double>(),
                                        calib_node["P_rect_01"][ 7].as<double>(),
                                        calib_node["P_rect_01"][ 8].as<double>(),
                                        calib_node["P_rect_01"][ 9].as<double>(),
                                        calib_node["P_rect_01"][10].as<double>(),
                                        calib_node["P_rect_01"][11].as<double>());

    Mat D_1 = (Mat_<double>(1,5) << calib_node["D_00"][0].as<double>(),
                                    calib_node["D_00"][1].as<double>(),
                                    calib_node["D_00"][2].as<double>(),
                                    calib_node["D_00"][3].as<double>(),
                                    calib_node["D_00"][4].as<double>());

    Mat D_2 = (Mat_<double>(1,5) << calib_node["D_01"][0].as<double>(),
                                    calib_node["D_01"][1].as<double>(),
                                    calib_node["D_01"][2].as<double>(),
                                    calib_node["D_01"][3].as<double>(),
                                    calib_node["D_01"][4].as<double>());

    int cols_L = calib_node["S_rect_00"][0].as<int>();
    int rows_L = calib_node["S_rect_00"][1].as<int>();
    int cols_R = calib_node["S_rect_01"][0].as<int>();
    int rows_R = calib_node["S_rect_01"][1].as<int>();



    Size S_rect1(cols_L,rows_L);
    Size S_rect2(cols_R,rows_R);

    Mat matL1, matL2;
    Mat matR1, matR2;

    cv::initUndistortRectifyMap(K_1, D_1, R_rect1, P_rect1, S_rect1, CV_32F, matL1, matL2);
    cv::initUndistortRectifyMap(K_2, D_2, R_rect2, P_rect2, S_rect2, CV_32F, matR1, matR2);

    Mat matLrect, matRrect;

    cv::remap(img1, matLrect, matL1, matL2, INTER_LINEAR);
    cv::remap(img2, matRrect, matR1, matR2, INTER_LINEAR);

    imshow("after Left", matLrect);
    imshow("after Right", matRrect);

    waitKey(0);



return 0;
}


/// hard-coded calib
/*
Mat R_rect1 = (Mat_<double>(3,3) << 9.826816289450e-01, 2.444704761493e-02, 1.836822201437e-01,
                                    -2.339291679524e-02, 9.996951034959e-01, -7.903890819248e-03,
                                    -1.838194428722e-01, 3.470145412675e-03, 9.829539005025e-01);

Mat R_rect2 = (Mat_<double>(3,3) << 9.836675825477e-01, 2.608950769921e-02, 1.780938646691e-01,
                                    -2.711049454173e-02, 9.996269917353e-01, 3.301284586158e-03,
                                    -1.779413052961e-01, -8.075579374197e-03, 9.840079658658e-01);

Mat P_rect1 = (Mat_<double>(3,4) << 4.328220393532e+02, 0.000000000000e+00, 3.413039855957e+02, 0.000000000000e+00,
                                    0.000000000000e+00, 4.328220393532e+02, 2.350234909058e+02, 0.000000000000e+00,
                                    0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00, 0.000000000000e+00);

Mat P_rect2 = (Mat_<double>(3,4) << 4.328220393532e+02, 0.000000000000e+00, 3.413039855957e+02, -4.963830275321e+01,
                                    0.000000000000e+00, 4.328220393532e+02, 2.350234909058e+02, 0.000000000000e+00,
                                    0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00, 0.000000000000e+00);
Mat D_1 = (Mat_<double>(1,5) << -1.966589467446e-01, 3.488312753209e-02, -1.109990416874e-03, 2.434258171672e-03, 0.000000000000e+00);

Mat D_2 = (Mat_<double>(1,5) << -1.928912817715e-01, 3.385670991895e-02, -1.063267710287e-03, 3.951352434793e-03, 1.842606618655e-04);


double cols = 9.140000000000e+02;
double rows = 4.480000000000e+02;
 */