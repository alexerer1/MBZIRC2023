
#ifndef ROTATIONS_H_
#define ROTATIONS_H_

# include "opencv2/opencv.hpp"

using namespace cv;

Mat eulerAnglesToRotationMatrix(Vec3f &theta);
bool isRotationMatrix(Mat &R); 
Vec3f rotationMatrixToEulerAngles(Mat &R);

#endif