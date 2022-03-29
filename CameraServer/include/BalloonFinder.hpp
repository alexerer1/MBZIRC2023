#ifndef BALLOONFINDER_HPP
#define BALLOONFINDER_HPP


#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/core/core.hpp"
#include "type_defines.hpp"


// #include "CameraLocalizer.hpp"

pose_t findBalloon(cv::Mat &image, bool draw);


#endif
