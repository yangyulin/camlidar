# camlidar

Stereo Cam and LiDAR calibration

There are 3 mains parts for the code in the src folder:

# src/factor: including the factor with calibration parameters

BetweenCalibFactor: 3D relative pose factor with calibration parameters

PlaneCalibMapFactor: plane measurement factor with calibration parameters (with targets)

StereoCalibMapFactor: stereo measurement (point) factor with calibration parameters (with targets)

StereoMapFactor: stereo measurement factor (with targets)


# dataprocess

stereo image processing, point cloud processing and init data reading

imgRect: rectify the stereo images, extract the chessboard pattern

initData: read the configure file, the initial value for the parameters

plcProd: process the point cloud to get the plane pcl (not clean right now)


# main file: 

imgrect_XX: for the image rectification

main_camlidar: for the calibration between the camera and lidar (Not done yet)