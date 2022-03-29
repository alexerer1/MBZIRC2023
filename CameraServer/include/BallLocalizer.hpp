#ifndef BALL_LOCALIZER_HPP
#define BALL_LOCALIZER_HPP


#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/core/core.hpp"

#include "type_defines.hpp"

// #include "CameraLocalizer.hpp"

class BallLocalizer {

public:
    int x = 0;
    pose_t update(cv::Mat * image, float roll, float pitch);
    void init( float ball_r );
    pose_t update2(cv::Mat * image, float roll, float pitch);
    
private:
    int lost_limit = 5;
    int lost = lost_limit;
    
    float old_position[2] = {0,0};
    
    float avg_array[4][5] = {{0.0}};
    
    int array_counter = 0;
    int array_place = 0;
    
    cv::Mat cameraMatrix=(cv::Mat_<double>(3,3)<<633.06058204,0,330.28981083,0,631.01252673,226.42308878,0,0,1);
	cv::Mat distCoeffs=(cv::Mat_<double>(1,5)<< 0.0503468649,-0.0438421987, -0.000252895273 ,
                        0.00191361583,-0.490955908);
    
    std::vector<cv::Point3f> ball_points = std::vector<cv::Point3f>(5);
    std::vector<cv::Point2d> image_points = std::vector<cv::Point2d>(5);
    
    
    
};

#endif
