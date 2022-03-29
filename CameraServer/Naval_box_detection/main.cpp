#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include "navalBoxDetection.hpp"

using namespace cv;
using namespace std;
typedef Mat data_holder_type1;
void calibrate_colors(Mat);
void CallBackFunc(int, int, int, int, void *);
/**
 * @function main
 */
int main(int argc, char **argv)
{
    navalBoxDetection boxDetecter;
    // boxDetecter.load_image("C:/Users/cma/Documents/School/Master Thesis/Code/Image analysis/OpenCVTest/ocean.jpg");
    boxDetecter.load_image("Z:/MBZIRC2023/CameraServer/log/videoLog/lastImage.jpg");

    if (boxDetecter.COLOR_CALIBRATION)
        boxDetecter.calibrate_colors(boxDetecter.image_state.src);

    // Read the image
    boxDetecter.extract_color_calibrated();

    // Apply Canny edge detector and find contours
    boxDetecter.edge_detector();
    boxDetecter.contour_search();

    // Remove all duplicate contours and extract COM
    boxDetecter.remove_overlapping_patterns();

    // Find target
    boxDetecter.find_target();

    boxDetecter.draw_image(boxDetecter.image_state.contours, boxDetecter.identifiedCenterOfMass);

    boxDetecter.draw_image(boxDetecter.target_object.targetcontours, boxDetecter.target_object.targetCOM);

    return 0;
}
