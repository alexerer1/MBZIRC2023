#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include "navalBoxDetection.hpp"
#include "type_defines.hpp"
using namespace cv;
using namespace std;

pose_t navalBoxDetection::find_box(Mat *frame)
{
    pose_t found_pose;
    found_pose.valid = 0;

    src = frame->clone();
    drawing = src;
    if (COLOR_CALIBRATION)
        calibrate_colors(src);

    filtered_image = src;
    if (LOWPASSFILTER)
        filtered_image = gaussian_smooth(src);

    color_extracted = extract_color_calibrated(filtered_image);

    if (MORPHINGFILTER)
    {
        morphed = morphology_open(color_extracted);
        morphed = morphology_close(morphed);
    }
    else
    {
        morphed = filtered_image;
    }

    // Apply Canny edge detector and find contours
    edges = edge_detector(morphed);

    edges = morphology_close(edges);

    contours = contour_search(edges);
    if (CONVEXHULL)
        contours = convex_hull(contours);

    add_contours_to_drawing(contours, drawing);

    if (contours.size() == 0)
    {
        if (ENABLE_LOGGING)
            cout << " No box found" << endl;
        return found_pose;
    }

    // Remove all duplicate contours and extract COM
    identifiedObjects = prune_identified_objects(contours);
    // Find target
    if (identifiedObjects.size() == 0)
    {
        if (ENABLE_LOGGING)
            cout << " No good objects found" << endl;
        return found_pose;
    }

    targetObject = find_target(identifiedObjects);

    found_pose = image_to_position(targetObject);

    return found_pose;
}

Mat navalBoxDetection::load_image(string filePath)
{
    Mat loadedImage;
    String imageName(filePath); // by default
    loadedImage = imread(samples::findFile(imageName), IMREAD_COLOR);
    if (loadedImage.empty())
    {

        if (ENABLE_LOGGING)
            cout << " image not loaded " << endl;
    }
    return loadedImage;
}

Mat navalBoxDetection::extract_color_calibrated(Mat frame)
{
    Mat HSV_image, HSV_threshold_image, RGB_output;
    double high_H, high_S, high_V, low_H, low_S, low_V;
    high_H = color_extraction.CALIBRATION_VALUE_H * (1 + color_extraction.COLOR_VARIATION_H) > 180.0 ? 180 : round(color_extraction.CALIBRATION_VALUE_H * (1 + color_extraction.COLOR_VARIATION_H));
    high_S = color_extraction.CALIBRATION_VALUE_S * (1 + color_extraction.COLOR_VARIATION_S) > 255.0 ? 255 : round(color_extraction.CALIBRATION_VALUE_S * (1 + color_extraction.COLOR_VARIATION_S));
    high_V = color_extraction.CALIBRATION_VALUE_V * (1 + color_extraction.COLOR_VARIATION_V) > 255.0 ? 255 : round(color_extraction.CALIBRATION_VALUE_V * (1 + color_extraction.COLOR_VARIATION_V));
    low_H = round(color_extraction.CALIBRATION_VALUE_H * (1 - color_extraction.COLOR_VARIATION_H));
    low_S = round(color_extraction.CALIBRATION_VALUE_S * (1 - color_extraction.COLOR_VARIATION_S));
    low_V = round(color_extraction.CALIBRATION_VALUE_V * (1 - color_extraction.COLOR_VARIATION_V));
    cvtColor(frame, HSV_image, COLOR_BGR2HSV);
    inRange(HSV_image, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), HSV_threshold_image);
    return HSV_threshold_image;
}

Mat navalBoxDetection::gaussian_smooth(Mat frame)
{
    Mat gaussian_filtered;
    GaussianBlur(frame, gaussian_filtered, Size(5, 5), 0);
    return gaussian_filtered;
}

Mat navalBoxDetection::morphology_open(Mat frame)
{
    Mat morph;
    Size ksize = Size(3, 3);
    int iterations = 2;
    Point anchor = Point(-1, -1);
    Mat structuring_element = getStructuringElement(MORPH_RECT, ksize);
    morphologyEx(frame, morph, MORPH_OPEN, structuring_element, anchor, iterations);
    return morph;
}

Mat navalBoxDetection::morphology_close(Mat frame)
{
    Mat morph;
    Size ksize = Size(3, 3);
    int iterations = 2;
    Point anchor = Point(-1, -1);
    Mat structuring_element = getStructuringElement(MORPH_RECT, ksize);
    morphologyEx(frame, morph, MORPH_CLOSE, structuring_element, anchor, iterations);
    return morph;
}
Mat navalBoxDetection::edge_detector(Mat frame)
{
    Mat canny_edges;
    Canny(frame, canny_edges, 35, 140, 3);
    return canny_edges;
}

vector<vector<Point>> navalBoxDetection::contour_search(Mat frame)
{
    vector<vector<Point>> all_contours;
    findContours(frame, all_contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    return all_contours;
    // create hull array for convex hull points
    /*
    vector<vector<Point>> hull(good_contours.size());

    for (int i = 0; i < good_contours.size(); i++)
    {


    }
    Mat drawing3 = Mat::zeros(edges.size(), CV_8UC3);
    for (size_t i = 0; i < hull.size(); i++)
    {
        drawContours(drawing3, hull, (int)i, Scalar(0, 0, 255), 2, LINE_8, hierarchy, 0);
    }
    imshow("Contours3", drawing3);

    waitKey(0);
    imshow("hull", drawing3);

    waitKey(0);

    identifiedObjects.resize(good_contours.size());
    for (int i = 0; i < contours.size(); i++)
    {
        convexHull(Mat(good_contours[i]), hull[i], false);
        identifiedObjects.push_back(make_tuple(good_contours[i], minAreaRect(good_contours[i])));
    }
    */
}

vector<vector<Point>> navalBoxDetection::convex_hull(vector<vector<Point>> contour_frame)
{
    vector<vector<Point>> hull(contour_frame.size());
    for (size_t i = 0; i < contour_frame.size(); i++)
    {
        convexHull(contour_frame[i], hull[i]);
    }
    return hull;
}

vector<tuple<vector<Point>, RotatedRect>> navalBoxDetection::prune_identified_objects(vector<vector<Point>> contour_frame)
{
    vector<tuple<vector<Point>, RotatedRect>> identifiedObjects_frame;

    int count = 0;
    vector<Point2d> identifiedCenterOfMass_temp;
    auto prune_object_function = std::remove_if(contour_frame.begin(), contour_frame.end(), [&identifiedCenterOfMass_temp, &identifiedObjects_frame, this, &count](vector<Point> currentObject)
                                                {
        RotatedRect rotated_rect=minAreaRect(currentObject);                                              
        double area = rotated_rect.size.area();
        if (ENABLE_LOGGING) cout << "Area (M_00) = " << std::fixed << std::setprecision(2) << area;
            if (area< 2000)
            {
                if (ENABLE_LOGGING)
                {
                    cout << " - TO SMALL" << endl;
                }
                count++;
                return true;
            }
            else
            {
                if (ENABLE_LOGGING)
                    cout << " | ";
            }
            Point2d CoM = rotated_rect.center;
            double limit_buffer = 0.1;
            if (ENABLE_LOGGING)
                cout << "COM = (" << std::fixed << std::setprecision(2) << CoM.x << "," << CoM.y << ")";
            for (size_t i = 0; i < identifiedCenterOfMass_temp.size(); i++)
            {

                if (CoM.x > identifiedCenterOfMass_temp[i].x * (1 - limit_buffer) && CoM.x < identifiedCenterOfMass_temp[i].x * (1 + limit_buffer) &&
                    CoM.y > identifiedCenterOfMass_temp[i].y * (1 - limit_buffer) && CoM.y < identifiedCenterOfMass_temp[i].y * (1 + limit_buffer))
                {
                    if (ENABLE_LOGGING)
                        cout << " - ALREADY FOUND (" << std::fixed << std::setprecision(2) << identifiedCenterOfMass_temp[i].x << "," << identifiedCenterOfMass_temp[i].y << ")" << endl;
                    count++;
                    return true;
                }
            }
            if (ENABLE_LOGGING)
                cout << endl;
            identifiedCenterOfMass_temp.push_back(CoM);
            count++;
            identifiedObjects_frame.push_back(make_tuple(currentObject, rotated_rect) );
            return false; });
    contour_frame.erase(prune_object_function, contour_frame.end());
    return identifiedObjects_frame;
}

tuple<vector<Point>, RotatedRect> navalBoxDetection::find_target(vector<tuple<vector<Point>, RotatedRect>> identifiedObjects_frame)
{
    int best_object_index = 0;
    double currentDistance, smallestDistance = maximum_distance_to_target_object;
    Point2d bestTarget, currentCoM;
    for (int i = 0; i < identifiedObjects_frame.size(); i++)
    {
        currentCoM = get<1>(identifiedObjects_frame[i]).center;
        currentDistance = norm(imageCenter - currentCoM);

        if (currentDistance < smallestDistance)
        {
            smallestDistance = currentDistance;
            best_object_index = i;
        }
    }
    return identifiedObjects_frame[best_object_index];
}

pose_t navalBoxDetection::image_to_position(tuple<vector<Point>, RotatedRect> identifiedObjects_frame)
{
    pose_t position;

    position.valid = 1;
    position.x = (IMG_COLS / 2 - get<1>(identifiedObjects_frame).center.x) / IMG_COLS * camera_fow_horisontal * M_PI / 180;
    position.y = (IMG_ROWS / 2 - get<1>(identifiedObjects_frame).center.y) / IMG_ROWS * camera_fow_vertical * M_PI / 180;
    position.th = get<1>(identifiedObjects_frame).angle;
    return position;
}

void navalBoxDetection::calibrate_colors(Mat src)
{
    Mat image;
    resize(src, image, src.size());
    // imshow("Calibration", image);

    setMouseCallback("Calibration", &CallBackFunc, &image);

    waitKey(0);
}

void navalBoxDetection::add_contours_to_drawing(vector<vector<Point>> add_contours, Mat drawing_frame)
{
    for (size_t i = 0; i < add_contours.size(); i++)
    {
        drawContours(drawing_frame, add_contours, (int)i, Scalar(0, 0, 255), 2, LINE_8, hierarchy, 0);
    }
}

void navalBoxDetection::add_contour_to_drawing(vector<Point> add_contour, Mat drawing_frame)
{
    drawContours(drawing_frame, vector<vector<Point>>(1, add_contour), -1, Scalar(0, 0, 255), 2, LINE_8, hierarchy, 0);
}

void navalBoxDetection::add_target_to_drawing(tuple<vector<Point>, RotatedRect> target_object, Mat drawing_frame)
{
    add_contour_to_drawing(get<0>(target_object), drawing_frame);
    circle(drawing_frame, get<1>(target_object).center, 3, Scalar(255, 0, 255), 2);
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
        double COLOR_VARIATION_H = 0.15;
        double COLOR_VARIATION_SV = 0.5;
        if (COLOR_VARIATION_H * H < 10)
        {
            COLOR_VARIATION_H = 10 / H;
        }
        high_H = H * (1 + COLOR_VARIATION_H) > 180 ? 180 : round(H * (1 + COLOR_VARIATION_H));
        high_S = S * (1 + COLOR_VARIATION_SV) > 255 ? 255 : round(S * (1 + COLOR_VARIATION_SV));
        high_V = V * (1 + COLOR_VARIATION_SV) > 255 ? 255 : round(V * (1 + COLOR_VARIATION_SV));
        low_H = round(H * (1 - COLOR_VARIATION_H));
        low_S = round(S * (1 - COLOR_VARIATION_SV));
        low_V = round(V * (1 - COLOR_VARIATION_SV));
        cout << " | Calibration range: [" << low_H << "," << low_S << "," << low_V << "] --> [" << high_H << "," << high_S << "," << high_V << "]" << endl;

        cvtColor(image, HSV_Image, COLOR_BGR2HSV);
        inRange(HSV_Image, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);

        // imshow("FILTERED", frame_threshold);
        // waitKey(0);
    }
}
