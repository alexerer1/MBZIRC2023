//
//  arudo_pose.h
//  
//
//  Created by Ananda Nielsen on 07/12/2018
//  Copyright © 2018 Ananda Nielsen. All rights reserved.
//

#ifndef ARUCO_POSE_H_INCLUDED
#define ARUCO_POSE_H_INCLUDED

// Rasperry Pi libs
# include <raspicam/raspicam_cv.h> 
# include "opencv2/opencv.hpp"

// Prototypes 
//int init_cam(raspicam::RaspiCam_Cv *cap);

//struct pose_t find_pos(raspicam::RaspiCam_Cv *cap, double roll, double pitch, bool drone_view);
struct pose_t find_pos(cv::Mat * src, double roll, double pitch);
struct pose_t estimate_pos(double roll, double pitch, double yaw, double x, double y, double z);

void transform_camera_to_body(struct pose_t * cam_pose, double roll, double pitch, double yaw);
void transform_to_global_map(pose_t * local_pose, int marker_id);

#endif
