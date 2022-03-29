#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

class navalBoxDetection
{
public:
    bool ENABLE_LOGGING = true, COLOR_CALIBRATION = true;
    /// Function Headers
    void load_image(string);
    void extract_color_calibrated();
    void edge_detector();
    void contour_search();
    void remove_overlapping_patterns();
    void find_target();
    void draw_image(vector<vector<Point>>, vector<Point2d>);
    void calibrate_colors(Mat src);

    struct imageState
    {
        Mat src, edges, src_color_extracted, drawing;
        vector<vector<Point>> contours;
    };

    imageState image_state;

    struct targetObject
    {
        vector<vector<Point>> targetcontours;
        vector<Point2d> targetCOM;
        Point2d target;
        targetObject() : targetCOM(1), targetcontours(1) {}
    };
    targetObject target_object;
    vector<Point2d> identifiedCenterOfMass;

private:
    /* General variables */
    struct colorExtraction
    {
        double CALIBRATION_VALUE_H = 179, CALIBRATION_VALUE_S = 226, CALIBRATION_VALUE_V = 237;
        double COLOR_VARIATION = 0.1;
    };
    colorExtraction color_extraction;

    int IMG_ROWS = 270, IMG_COLS = 480;

    vector<Vec4i> hierarchy;

    /// Global variables
    double DEG_TO_RAD = CV_PI / 180;
    double RAD_TO_DEG = 180 / CV_PI;

    bool is_target_locked = false;
    double distance_to_target_object = norm(Point2d(IMG_COLS, IMG_ROWS) - Point2d(IMG_COLS / 2, IMG_ROWS / 2));
};

void CallBackFunc(int event, int x, int y, int flags, void *userdata);