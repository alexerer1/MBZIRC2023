//
//  camera_localizer.hpp
//  
//
//  Created by Ananda Nielsen on 29/04/2019
//  Copyright © 2019 Ananda Nielsen. All rights reserved.
//

#ifndef CAMERA_LOCALIZER_HPP_INCLUDED
#define CAMERA_LOCALIZER_HPP_INCLUDED

// Rasperry Pi libs
# include <raspicam/raspicam_cv.h> 
# include "opencv2/opencv.hpp"

struct H_decomped{
	cv::Mat T;
	cv::Mat R;
	cv::Mat N;
	cv::Mat Sigma;
} H_decomped;

// Prototypes 
int init_cam(raspicam::RaspiCam_Cv *cap);
extern "C" int start_camera_localizer(const char * map_file);

int flow_estimation(char calcFlow);
int ransac_homography_estimation(std::vector<uchar> * status);
int planar_homography_estimation(cv::Mat * H_dst, std::vector<uchar> * status);
int construct_chi( float chi[12][9], cv::Point2f point1, cv::Point2f point2, int position );

int decompose_homography(cv::Mat * H, struct H_decomped * motion);

//int update_aruco_pos( &d_motion );

#endif
