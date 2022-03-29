//
//  arudo_pose.h
//  
//
//  Created by Ananda Nielsen on 07/12/2018
//  Copyright © 2018 Ananda Nielsen. All rights reserved.
//

#ifndef ARUCO_POSE_H_INCLUDED
#define ARUCO_POSE_H_INCLUDED

#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/core/core.hpp"

#include "CameraLocalizer.hpp"

#include "aruco_map.h"

struct pose_t{
    char valid;
    double x;
    double y;
    double z;
    double th;
};

//pose_t find_pos(cv::Mat * src, ar_map * global_map, double roll, double pitch);
pose_t find_pos(cv::Mat * image, CameraLocalizer * camera, double roll, double pitch);
pose_t estimate_pos(double roll, double pitch, double yaw, double x, double y, double z);

void transform_camera_to_body(pose_t * cam_pose, double roll, double pitch, double yaw);
void transform_to_global_map(pose_t * local_pose, CameraLocalizer * camera, int marker_id);

#endif
