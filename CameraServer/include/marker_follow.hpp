#ifndef MARKERS_FOLLOW_HPP
#define MARKERS_FOLLOW_HPP

#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/core/core.hpp"

#include "type_defines.hpp"

pose_t find_marker(cv::Mat * image, bool draw_markers);

#endif
