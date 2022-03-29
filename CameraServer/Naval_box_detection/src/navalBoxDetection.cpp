#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include "navalBoxDetection.hpp"
using namespace cv;
using namespace std;

RNG rng(12345);
void navalBoxDetection::load_image(string filePath)
{
    Mat loadedImage;
    String imageName(filePath); // by default
    loadedImage = imread(samples::findFile(imageName), IMREAD_COLOR);
    if (loadedImage.empty())
    {

        if (ENABLE_LOGGING)
            cout << " image not loaded " << endl;
    }
    image_state.src = loadedImage;
}

void navalBoxDetection::extract_color_calibrated()
{
    Mat BGR_image = image_state.src.clone();
    Mat HSV_image, HSV_threshold_image, RGB_output;
    double high_H, high_S, high_V, low_H, low_S, low_V;
    high_H = color_extraction.CALIBRATION_VALUE_H * (1 + color_extraction.COLOR_VARIATION) > 180.0 ? 180 : round(color_extraction.CALIBRATION_VALUE_H * (1 + color_extraction.COLOR_VARIATION));
    high_S = color_extraction.CALIBRATION_VALUE_S * (1 + color_extraction.COLOR_VARIATION) > 255.0 ? 255 : round(color_extraction.CALIBRATION_VALUE_S * (1 + color_extraction.COLOR_VARIATION));
    high_V = color_extraction.CALIBRATION_VALUE_V * (1 + color_extraction.COLOR_VARIATION) > 255.0 ? 255 : round(color_extraction.CALIBRATION_VALUE_V * (1 + color_extraction.COLOR_VARIATION));
    low_H = round(color_extraction.CALIBRATION_VALUE_H * (1 - color_extraction.COLOR_VARIATION));
    low_S = round(color_extraction.CALIBRATION_VALUE_S * (1 - color_extraction.COLOR_VARIATION));
    low_V = round(color_extraction.CALIBRATION_VALUE_V * (1 - color_extraction.COLOR_VARIATION));

    cvtColor(BGR_image, HSV_image, COLOR_BGR2HSV);

    inRange(HSV_image, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), HSV_threshold_image);

    // dilate to smooth out any small inconsistencies in the shape
    Mat element = getStructuringElement(MORPH_RECT,
                                        Size(2 * 1 + 1, 2 * 1 + 1),
                                        Point(1, 1));
    dilate(HSV_threshold_image, HSV_threshold_image, element, Point(-1, -1), 2);

    image_state.src_color_extracted = HSV_threshold_image;
}

void navalBoxDetection::edge_detector()
{
    Canny(image_state.src_color_extracted, image_state.edges, 35, 140, 3);
    imshow("Contours", image_state.edges);
    waitKey(0);
}

void navalBoxDetection::contour_search()
{
    findContours(image_state.edges, image_state.contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
}

void navalBoxDetection::remove_overlapping_patterns()
{
    if (ENABLE_LOGGING)
        cout << "Remove_overlapping_patterns " << endl;
    vector<Point2d> identifiedCenterOfMass_local;
    int count = 0;
    auto duplicate_checker = std::remove_if(image_state.contours.begin(), image_state.contours.end(), [&identifiedCenterOfMass_local, this, &count](vector<Point> currentContour)
                                            {
        Moments mu = moments(currentContour);
        double area0 = contourArea(currentContour);
        if (ENABLE_LOGGING) cout << "Area (M_00) = " << std::fixed << std::setprecision(2) << mu.m00 << " Second opinion: " << area0;
            if (mu.m00 < 250)
            {
                if (ENABLE_LOGGING)
                {

                    cout << " - TO SMALL" << endl;

                    Mat drawing = Mat::zeros(image_state.edges.size(), CV_8UC3);
                    for (size_t i = 0; i < image_state.contours.size(); i++)
                    {
                        Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
                        drawContours(drawing, image_state.contours, (int)count, color, 1, LINE_8, hierarchy, 0);
                    }
                    imshow("Contours", drawing);

                    waitKey(0);
                }
                count++;
                return true;
            }
            else
            {
                if (ENABLE_LOGGING)
                    cout << " | ";
            }
            double x = round(mu.m10 / mu.m00);
            double y = round(mu.m01 / mu.m00);
            double limit_buffer = 0.1;
            if (ENABLE_LOGGING)
                cout << "COM = (" << std::fixed << std::setprecision(2) << x << "," << y << ")";
            for (size_t i = 0; i < identifiedCenterOfMass_local.size(); i++)
            {

                if (x > identifiedCenterOfMass_local[i].x * (1 - limit_buffer) && x < identifiedCenterOfMass_local[i].x * (1 + limit_buffer) &&
                    y > identifiedCenterOfMass_local[i].y * (1 - limit_buffer) && y < identifiedCenterOfMass_local[i].y * (1 + limit_buffer))
                {
                    if (ENABLE_LOGGING)
                        cout << " - ALREADY FOUND (" << std::fixed << std::setprecision(2) << identifiedCenterOfMass_local[i].x << "," << identifiedCenterOfMass_local[i].y << ")" << endl;
                    count++;
                    return true;
                }
            }
            if (ENABLE_LOGGING)
                cout << endl;
            identifiedCenterOfMass_local.push_back(Point2d(x, y));
            count++;
            return false; });
    identifiedCenterOfMass = identifiedCenterOfMass_local;
    image_state.contours.erase(duplicate_checker, image_state.contours.end());
}

void navalBoxDetection::find_target()
{
    int targetIndex;
    double currentDistance, smallestDistance = distance_to_target_object;
    Point2d bestTarget;
    Point2d imageCenter = Point2d(round(IMG_COLS / 2), round(IMG_ROWS / 2));
    if (ENABLE_LOGGING)
        cout << "imageCenter = (" << imageCenter.x << "," << imageCenter.y << ")" << endl;

    for (int i = 0; i < identifiedCenterOfMass.size(); i++)
    {

        if (is_target_locked)
        {
            currentDistance = norm(target_object.target - identifiedCenterOfMass[i]);
        }
        else
        {
            currentDistance = norm(imageCenter - identifiedCenterOfMass[i]);
        }
        if (currentDistance < smallestDistance)
        {
            smallestDistance = currentDistance;
            bestTarget = identifiedCenterOfMass[i];
            targetIndex = i;
        }
    }

    distance_to_target_object = smallestDistance;
    target_object.target = bestTarget;
    target_object.targetCOM[0] = bestTarget;
    is_target_locked = true;
    if (ENABLE_LOGGING)
        cout << "Locking the target (" << target_object.target.x << "," << target_object.target.y << ") | distance: " << distance_to_target_object << endl;

    target_object.targetcontours[0] = image_state.contours[targetIndex];
}

void navalBoxDetection::calibrate_colors(Mat src)
{
    Mat image;
    resize(src, image, src.size());
    imshow("Calibration", image);

    setMouseCallback("Calibration", &CallBackFunc, &image);

    waitKey(0);
}

void navalBoxDetection::draw_image(vector<vector<Point>> contours, vector<Point2d> identifiedCenterOfMass)
{
    image_state.drawing = Mat::zeros(image_state.edges.size(), CV_8UC3);

    // Testing

    for (size_t i = 0; i < identifiedCenterOfMass.size(); i++)
    {
        Point2d centerPoint = Point2d(identifiedCenterOfMass[i].x, identifiedCenterOfMass[i].y);

        circle(image_state.drawing, centerPoint, 3, Scalar(255, 0, 255), 2);
    }

    for (size_t i = 0; i < contours.size(); i++)
    {
        Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
        drawContours(image_state.drawing, contours, (int)i, color, 2, LINE_8, hierarchy, 0);
    }
    // Draw path to closest object
    line(image_state.drawing, Point2d(IMG_COLS / 2, IMG_ROWS / 2), target_object.target, Scalar(rng.uniform(0, 0), rng.uniform(0, 0), rng.uniform(0, 256)), 2, 2);
    imshow("Contours", image_state.drawing);

    waitKey(0);
}

void CallBackFunc(int event, int x, int y, int flags, void *userdata)
{

    if (event == EVENT_LBUTTONDOWN)
    {
        Mat *image_temp = static_cast<Mat *>(userdata);
        Mat image = image_temp->clone();

        cout << "CALIBRATION CLICK - size : " << image.size() << endl;
        Mat HSV_Pixel, HSV_Image;
        Mat RGB = image(Rect(x, y, 1, 1));
        cvtColor(RGB, HSV_Pixel, COLOR_BGR2HSV);
        Mat frame_threshold;
        cout << "Clicked coordinates: (" << x << ", " << y << ")";
        Vec3b hsv = HSV_Pixel.at<Vec3b>(0, 0);

        int H = hsv.val[0];
        int S = hsv.val[1];
        int V = hsv.val[2];
        double high_H, high_S, high_V, low_H, low_S, low_V;

        cout << " | Pixel HSV value : " << H << "," << S << "," << V;
        double COLOR_VARIATION = 0.1;
        high_H = H * (1 + COLOR_VARIATION) > 180 ? 180 : round(H * (1 + COLOR_VARIATION));
        high_S = S * (1 + COLOR_VARIATION) > 255 ? 255 : round(S * (1 + COLOR_VARIATION));
        high_V = V * (1 + COLOR_VARIATION) > 255 ? 255 : round(V * (1 + COLOR_VARIATION));
        low_H = round(H * (1 - COLOR_VARIATION));
        low_S = round(S * (1 - COLOR_VARIATION));
        low_V = round(V * (1 - COLOR_VARIATION));
        cout << " | Calibration range: [" << low_H << "," << low_S << "," << low_V << "] --> [" << high_H << "," << high_S << "," << high_V << "]" << endl;

        cvtColor(image, HSV_Image, COLOR_BGR2HSV);
        inRange(HSV_Image, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);

        imshow("FILTERED", frame_threshold);

        waitKey(0);
    }
}
