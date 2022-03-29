#include "BalloonFinder.hpp"

// openCV libs
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"

#include <math.h>
#include <vector>
#include <stdio.h>

using namespace cv;
using namespace std;

/*********************************************
 *
 *   Function that draws and ellipse with the rotated
 *   rectangle that encloses it
 *
 **********************************************/
void drawEllipseWithBox(Mat &img, RotatedRect box, Scalar color, int lineThickness)
{
    int x = 0;
    int y = 0;
    float scale = 1;
    box.center = scale * cv::Point2f(box.center.x - x, box.center.y - y);
    box.size.width = (float)(scale * box.size.width);
    box.size.height = (float)(scale * box.size.height);

    ellipse(img, box, color, lineThickness, LINE_AA);

    Point2f vtx[4];
    box.points(vtx);
    for (int j = 0; j < 4; j++)
    {
        line(img, vtx[j], vtx[(j + 1) % 4], color, lineThickness, LINE_AA);
    }
}

/*********************************************
 *
 *   Function that finds the balloon based on a HSV
 *   color mask and then fits ellipses to find
 *   the best balloon
 *
 **********************************************/
pose_t findBalloon(cv::Mat &image, bool draw)
{

    Mat gray;

    // FOV of raspicam v2
    float horz_fov = 62.2;
    float vert_fov = 48.8;

    pose_t found_pose;
    found_pose.valid = 0;

    Mat hsv, low_mask, high_mask, mask;
    Mat rgb, masked_hsv;

    Mat ch1, ch2, ch3;
    // "channels" is a vector of 3 Mat arrays:
    vector<Mat> channels(3);

    // split img:
    // std::cout << image[Scalar(20,20,1)] << std::endl;

    // Some pre filtering and image conversions
    normalize(image, image, 0, 255, NORM_MINMAX);
    blur(image, image, Size(3, 3));
    cvtColor(image, hsv, COLOR_BGR2HSV);
    cvtColor(image, gray, COLOR_BGR2GRAY);

    // Threshhold the HSV image to find green
    // Scalar lowerRange = Scalar(50,50,40);

    // Scalar lowerRange = Scalar(30,100,100);
    // Scalar highRange = Scalar(85,255,255);
    Scalar lowerRange = Scalar(30, 50, 50);
    Scalar highRange = Scalar(90, 255, 255);

    inRange(hsv, lowerRange, highRange, masked_hsv);

    // adaptiveThreshold( gray, gray, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 5, 0);
    // Show in a window
    // namedWindow("Source", WINDOW_AUTOSIZE);
    // imshow("Source", image);
    // namedWindow("Mask", WINDOW_AUTOSIZE);
    // imshow("Mask", masked_hsv);

    cv::Mat im = image;
    cv::Mat not_im = masked_hsv;
    // bitwise_not(masked_hsv, not_im);
    cvtColor(not_im, not_im, COLOR_GRAY2BGR);

    bitwise_and(not_im, im, image);

    // image = masked_hsv;

    // Remove small particles
    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(3, 3), Size(-1, -1));
    erode(masked_hsv, masked_hsv, element);
    dilate(masked_hsv, masked_hsv, element);

    // Find contours lefter after filtering
    vector<vector<Point>> contours;
    vector<Vec4i> hierachy;
    findContours(masked_hsv, contours, hierachy, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point(0, 0));
    // findContours( masked_hsv, contours, hierachy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, Point(0,0));

    RotatedRect elipsoid, best_elipsoid; // = cv::fitEllipse(contours[0]);

    int best_ellipse = -1;
    float best_ratio = 0;
    float ratio = 0;
    float best_size = 0;
    float ellipse_area;
    int hull_area, contour_area;
    vector<vector<Point>> hull(contours.size());

    // Check all contours to find the best balloon
    for (int i = 0; i < contours.size(); i++)
    {

        convexHull(contours[i], hull[i]);
        hull_area = contourArea(hull[i]);
        contour_area = contourArea(contours[i]);

        // Check that the ballon is large enough
        if (hull_area > 250 && hull_area < (640 * 480))
        {

            // Fit ellipse and find its area (standard ellipse formula)
            elipsoid = fitEllipse(contours[i]);
            ellipse_area = 3.14159 * elipsoid.size.width / 2.0 * elipsoid.size.height / 2.0;
            ratio = ellipse_area / (float)contour_area;
            // ratio = ellipse_area / (float)hull_area;

            // Check the ratio of the size of the contours and the size of the ellipse area
            if (fabs(ratio - 1) < fabs(best_ratio - 1) && fabs(ratio - 1) < 0.05)
            {
                // Check if it is round enough to be a balloon
                float round_ratio = (fabs((float)(elipsoid.size.width - elipsoid.size.height)) / ((float)(elipsoid.size.width + elipsoid.size.height)));
                if (round_ratio < 0.2)
                {
                    // Find the largest
                    if (ellipse_area > best_size)
                    {
                        best_ratio = ratio;
                        best_ellipse = i;
                        best_elipsoid = elipsoid;
                        best_size = ellipse_area;
                        if (draw)
                        {
                            cout << "draw" << endl;
                            drawEllipseWithBox(image, elipsoid, Scalar(255, 0, 0), 2);
                            drawContours(image, contours, i, Scalar(0, 0, 255), 2, 8, hierachy, 0, Point());
                            drawContours(image, hull, i, Scalar(255, 0, 255), 2, 8, hierachy, 0, Point());
                        }
                    }
                }
                // cout << "Ratio = " << fabs(ratio - 1) << endl;
            }

            // cout << ellipse_area << "  |  " << hull_area << "  |  " << contour_area << endl;

            // if(abs(hull_area-ellipse_area) < ellipse_area*0.05){
            //     if( draw ) {
            //         drawEllipseWithBox(*image, elipsoid, Scalar(255,0,0), 2);
            //         drawContours( *image, contours, i, Scalar(0,0,255), 2, 8, hierachy, 0, Point() );
            //         drawContours( *image, hull, i, Scalar(255,0,255), 2, 8, hierachy, 0, Point() );
            //     }
            // }
        }
    }

    // If a suitable balloon is found
    if (best_ellipse != -1)
    {
        circle(image, best_elipsoid.center, 3, Scalar(0, 0, 255), -1, 8, 0);

        // Calculate the angular position of the balloon center i the camera frame
        found_pose.valid = 1;
        found_pose.x = (320.0 - best_elipsoid.center.x) / 640 * horz_fov * M_PI / 180;
        // found_pose.x = (320.0 - best_elipsoid.center.x) / 640 * horz_fov * M_PI/180;
        found_pose.y = (240.0 - best_elipsoid.center.y) / 480 * vert_fov * M_PI / 180;
        // Store diameter of the balloon
        found_pose.z = 0.5 * (float)(best_elipsoid.size.width + best_elipsoid.size.height);
        found_pose.th = found_pose.x;

        cout << "Balloon:  " << found_pose.x * 180 / M_PI << "  |  " << found_pose.y * 180 / M_PI << endl;
        cout << "Size:  " << best_elipsoid.size.width << "  |  " << best_elipsoid.size.height << endl;
    }

    return found_pose;
}
