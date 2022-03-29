#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include "type_defines.hpp"

using namespace cv;
using namespace std;

class navalBoxDetection
{
public:
    bool ENABLE_LOGGING = false, COLOR_CALIBRATION = false, LOWPASSFILTER = true, MORPHINGFILTER = true, CONVEXHULL = true;
    /// Function Headers
    pose_t find_box(Mat *frame);
    Mat load_image(string);
    Mat extract_color_calibrated(Mat);
    Mat gaussian_smooth(Mat);
    Mat morphology_open(Mat);
    Mat morphology_close(Mat);
    Mat edge_detector(Mat);
    vector<vector<Point>> contour_search(Mat);
    vector<vector<Point>> convex_hull(vector<vector<Point>>);
    vector<tuple<vector<Point>, RotatedRect>> prune_identified_objects(vector<vector<Point>>);
    tuple<vector<Point>, RotatedRect> find_target(vector<tuple<vector<Point>, RotatedRect>>);
    pose_t image_to_position(tuple<vector<Point>, RotatedRect>);
    void add_contours_to_drawing(vector<vector<Point>>, Mat);
    void add_contour_to_drawing(vector<Point>, Mat);
    void add_target_to_drawing(tuple<vector<Point>, RotatedRect>, Mat);
    void calibrate_colors(Mat src);

    Mat src, filtered_image, color_extracted, morphed, edges, drawing;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    // A Vector containing a tuple containing the <contours, COM, Area> for each object
    vector<tuple<vector<Point>, RotatedRect>> identifiedObjects;
    tuple<vector<Point>, RotatedRect> targetObject;

private:
    /* General variables */
    struct colorExtraction
    {
        double CALIBRATION_VALUE_H = 168, CALIBRATION_VALUE_S = 158, CALIBRATION_VALUE_V = 221;
        double COLOR_VARIATION_H = 0.2, COLOR_VARIATION_S = 0.4, COLOR_VARIATION_V = 0.4;
    };
    colorExtraction color_extraction;

    int IMG_ROWS = 480, IMG_COLS = 640;
    float camera_fow_horisontal = 62.2, camera_fow_vertical = 48.8;

    /// Global variables
    double DEG_TO_RAD = CV_PI / 180;
    double RAD_TO_DEG = 180 / CV_PI;

    bool is_target_locked = false;
    double distance_to_target_object = norm(Point2d(IMG_COLS, IMG_ROWS) - Point2d(IMG_COLS / 2, IMG_ROWS / 2));
    double maximum_distance_to_target_object = norm(Point2d(IMG_COLS, IMG_ROWS) - Point2d(IMG_COLS / 2, IMG_ROWS / 2));
    Point2d imageCenter = Point2d(round(IMG_COLS / 2), round(IMG_ROWS / 2));
};

void CallBackFunc(int event, int x, int y, int flags, void *userdata);