#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <stdio.h> /* printf */
#include <stdlib.h>
#include <string>
#include <time.h> /* clock_t, clock, CLOCKS_PER_SEC */

#include "boxDetection.hpp"

using namespace std;
using namespace cv;

Mat boxDetection::maskHSV(Mat image_HSV, char boxColor)
{
    Mat mask1, mask2;

    int* HSV_LOW;
    int* HSV_HIGH;

    switch (toupper(boxColor)) {
    case 'G':
        HSV_LOW = LOWGREEN;
        HSV_HIGH = HIGHGREEN;
        break;
    case 'R':
        HSV_LOW = LOWRED;
        HSV_HIGH = HIGHRED;
        break;
    case 'B':
        HSV_LOW = LOWBLUE;
        HSV_HIGH = HIGHBLUE;
        break;
    case 'O':
        HSV_LOW = LOWORANGE;
        HSV_HIGH = HIGHORANGE;
        break;
    case 'W':
        HSV_LOW = LOWWHITE;
        HSV_HIGH = HIGHWHITE;
        break;
    case 'Y':
        HSV_LOW = LOWYELLOW;
        HSV_HIGH = HIGHYELLOW;
        break;
    default:
        HSV_LOW = LOWGREEN;
        HSV_HIGH = HIGHGREEN;
        break;
    }

    if (HSV_LOW[0] >= 180) {
        inRange(image_HSV, Scalar(HSV_LOW[0] - 180, HSV_LOW[1], HSV_LOW[2]), Scalar(HSV_HIGH[0] - 180, HSV_HIGH[1], HSV_HIGH[2]), mask1);
        inRange(image_HSV, Scalar(HSV_LOW[0] - 180, HSV_LOW[1], HSV_LOW[2]), Scalar(HSV_HIGH[0] - 180, HSV_HIGH[1], HSV_HIGH[2]), mask2);
    }
    else if (HSV_HIGH[0] > 180) {
        inRange(image_HSV, Scalar(HSV_LOW[0], HSV_LOW[1], HSV_LOW[2]), Scalar(180, HSV_HIGH[1], HSV_HIGH[2]), mask1);
        inRange(image_HSV, Scalar(0, HSV_LOW[1], HSV_LOW[2]), Scalar(HSV_HIGH[0] - 180, HSV_HIGH[1], HSV_HIGH[2]), mask2);
    }
    else {
        inRange(image_HSV, Scalar(HSV_LOW[0], HSV_LOW[1], HSV_LOW[2]), Scalar(HSV_HIGH[0], HSV_HIGH[1], HSV_HIGH[2]), mask1);
        inRange(image_HSV, Scalar(HSV_LOW[0], HSV_LOW[1], HSV_LOW[2]), Scalar(HSV_HIGH[0], HSV_HIGH[1], HSV_HIGH[2]), mask2);
    }

    return mask1 + mask2;
}

vector<vector<vector<Point>>> boxDetection::extractBoxCountours(Size imageSize, vector<vector<Point>> contours, vector<Vec4i> hierarchy)
{
    vector<vector<vector<Point>>> boxCountours(2);
    double area = 0;
    // Area threshold is set to 1/400 of the pixels in the image
    double areaThreshold = imageSize.width * imageSize.height / 1000;
    for (int i = 0; i < contours.size(); i++) {
        area = contourArea(contours[i]);
        if (area > areaThreshold && hierarchy[i][3] != -1 && hierarchy[hierarchy[i][3]][3] == -1) {
            boxCountours[0].push_back(contours[hierarchy[i][3]]);
            boxCountours[1].push_back(contours[i]);
        }
    }
    return boxCountours;
}

vector<vector<vector<Point>>> boxDetection::extractBoxCountoursWhite(Size imageSize, vector<vector<Point>> contours, vector<Vec4i> hierarchy)
{
    vector<vector<vector<Point>>> boxCountours(1);
    double area = 0;
    // Area threshold is set to 1/400 of the pixels in the image
    double areaThreshold = imageSize.width * imageSize.height / 400;
    for (int i = 0; i < contours.size(); i++) {
        area = contourArea(contours[i]);
        if (area > areaThreshold) {
            boxCountours[0].push_back(contours[i]);
        }
    }
    return boxCountours;
}

Mat boxDetection::drawBoundingBoxes(Mat image, vector<vector<RotatedRect>> minRect)
{
    RNG rng(1234);
    Mat drawing = image.clone();
    for (int i = 0; i < minRect[0].size(); i++) {
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        // rotated rectangle
        Point2f rect_points[4];
        minRect[0][i].points(rect_points);
        for (int j = 0; j < 4; j++)
            line(drawing, rect_points[j], rect_points[(j + 1) % 4], color, 4, 8);
        drawMarker(drawing, minRect[0][i].center, color, MARKER_TILTED_CROSS, 20, 5);
    }
    for (int i = 0; i < minRect[1].size(); i++) {
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        // rotated rectangle
        Point2f rect_points[4];
        minRect[1][i].points(rect_points);
        for (int j = 0; j < 4; j++)
            line(drawing, rect_points[j], rect_points[(j + 1) % 4], color, 4, 8);
        drawMarker(drawing, minRect[1][i].center, color, MARKER_TILTED_CROSS, 20, 5);
    }
    return drawing;
}

Mat boxDetection::drawContour(Mat image, vector<vector<vector<Point>>> boxCountours)
{
    RNG rng(1234);
    Mat drawing = image.clone();
    for (int i = 0; i < boxCountours[0].size(); i++) {
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        // contour
        drawContours(drawing, boxCountours[0], i, color, 3, 8, vector<Vec4i>(), 0, Point());
    }
    for (int i = 0; i < boxCountours[1].size(); i++) {
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        // contour
        drawContours(drawing, boxCountours[1], i, color, 3, 8, vector<Vec4i>(), 0, Point());
    }
    return drawing;
}

vector<vector<int>> boxDetection::findMinMax(vector<vector<Point>> convex, bool returnMax)
{
    vector<vector<int>> yval(convex.size(), vector<int>(2));
    for (int i = 0; i < convex.size(); i++) {
        yval[i][0] = convex[i][0].y;
        yval[i][1] = 0;
        for (int j = 1; j < convex[i].size(); j++) {
            if (returnMax) {
                if (convex[i][j].y > yval[i][0]) {
                    yval[i][0] = convex[i][j].y;
                    yval[i][1] = j;
                }
            }
            else {
                if (convex[i][j].y < yval[i][0]) {
                    yval[i][0] = convex[i][j].y;
                    yval[i][1] = j;
                }
            }
        }
    }
    return yval;
}

vector<vector<boxDetection::vectorIndex>> boxDetection::getDistanceVectorIndex(vector<vector<Point>> convex)
{
    vector<vector<vectorIndex>> distanceVectorIndex(convex.size(), vector<vectorIndex>(2));
    double dist = 0;
    Point diff(0, 0);
    for (int i = 0; i < convex.size(); i++) {
        for (int j = 0; j < convex[i].size(); j++) {
            if (j == convex[i].size() - 1) {
                diff = convex[i][j] - convex[i][0];
            }
            else {
                diff = convex[i][j] - convex[i][j + 1];
            }
            dist = norm(diff);
            if (dist > norm(distanceVectorIndex[i][0].p)) {
                distanceVectorIndex[i][1] = distanceVectorIndex[i][0];

                distanceVectorIndex[i][0].p = diff;
                distanceVectorIndex[i][0].i = j;
            }
            else if (dist > norm(distanceVectorIndex[i][1].p)) {
                distanceVectorIndex[i][1].p = diff;
                distanceVectorIndex[i][1].i = j;
            }
        }
    }
    return distanceVectorIndex;
}

vector<vector<double>> boxDetection::calcAngle(vector<RotatedRect> innerMinRect)
{
    vector<vector<double>> angle(innerMinRect.size(), vector<double>(2));
    for (int i = 0; i < angle.size(); i++) {
        angle[i][1] = innerMinRect[i].angle;
        if (innerMinRect[i].size.width > innerMinRect[i].size.height) {
            angle[i][0] = - innerMinRect[i].angle * CV_PI / 180;
        } else {
            angle[i][0] = -CV_PI/2 -innerMinRect[i].angle * CV_PI / 180;
        }
        /*
        angle[i][2] = CV_PI / 2 - innerMinRect[i].angle * CV_PI / 180;
        if (angle[i][2] < -CV_PI / 2) {
            angle[i][2] += CV_PI;
        }
        else if (angle[i][2] > CV_PI / 2) {
            angle[i][2] -= CV_PI;
        }
        */

        // This is not nessesary with the new boxes being rectangular
        /*
        if (abs(angle[i][0] - angle[i][2]) > CV_PI / 3 && abs(angle[i][0] - angle[i][2]) < CV_PI * 2 / 3) {
            angle[i][2] += CV_PI / 2;
        }
        */
        
    }
    return angle;
}

int boxDetection::findBestBox(vector<RotatedRect> innerMinRect, Size imageSize)
{
    // Find the center of the image
    Point2f center = (Point2f)(imageSize) / 2;
    // First for the box at index 0
    int minIndex = 0;
    Point2f diff = center - innerMinRect[minIndex].center;
    float minDist = sqrt(diff.x * diff.x + diff.y * diff.y);
    // Then compare with the rest
    for (int i = 1; i < innerMinRect.size(); i++) {
        Point2f diff = center - innerMinRect[i].center;
        float dist = sqrt(diff.x * diff.x + diff.y * diff.y);
        if (dist < minDist) {
            minDist = dist;
            minIndex = i;
        }
    }
    return minIndex;
}

int boxDetection::findBoxInROI(vector<RotatedRect> innerMinRect, float regionRadiusScale)
{
    float radius = (box.w + box.h) / 2.0 * regionRadiusScale;
    // cout << radius << endl;
    int minIndex = -1;
    Point2f center = Point2f((float)box.x, (float)box.y);
    float minDist = radius;
    for (int i = 0; i < innerMinRect.size(); i++) {
        Point2f diff = center - innerMinRect[i].center;
        float dist = sqrt(diff.x * diff.x + diff.y * diff.y);
        if (dist < minDist) {
            minDist = dist;
            minIndex = i;
        }
    }

    return minIndex;
}

Mat boxDetection::drawROI(Mat image, vector<RotatedRect> innerMinRect, float regionRadiusScale)
{
    RNG rng(1234);
    Mat drawing = image.clone();
    Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    
    float radius = (box.w + box.h) / 2.0 * regionRadiusScale;
    
    circle(drawing, Point(box.x, box.y), radius, color, 3, 8, 0);
    drawMarker(drawing, Point(box.x, box.y), color, MARKER_TILTED_CROSS, 20, 5);

    // circle(drawing, Point(previousBox.center.x, previousBox.center.y), radius, color, 3, 8, 0);
    // drawMarker(drawing, previousBox.center, color, MARKER_TILTED_CROSS, 20, 5);

    return drawing;
}

bool boxDetection::boxNearBottomEdge(Mat image) {
    Size imageSize = Size(image.size());
    // extract the widest size of the inner box
    if (box.y + box.h/2 > imageSize.height*0.8) {
        // cout << "debug" << endl;
        // printf("Near bottom with size (%d,%d) and angle %f\n", width, height, box.angle * 180 / CV_PI);
        return true;
    } else {
        return false;
    }
}

vector<RotatedRect> boxDetection::findFakeBox(vector<vector<vector<Point>>> points, char boxColor) {
    float RATIO;
    switch (toupper(boxColor)) {
    case 'G':
        RATIO = GREENHEIGHTWIDTHRATIO;
        break;
    case 'R':
        RATIO = REDHEIGHTWIDTHRATIO;
        break;
    case 'B':
        RATIO = BLUEHEIGHTWIDTHRATIO;
        break;
    case 'O':
        RATIO = ORANGEHEIGHTWIDTHRATIO;
        break;
    default:
        RATIO = REDHEIGHTWIDTHRATIO;
        break;
    }
    // extract the widest size of the inner box
    Point upperLeft, upperRight, lowerLeft, lowerRight;
    for (int i = 0; i < points[0].size(); i++) {
        if (points[0][i].size() > 3) {
            for (int j = 0; j < points[0][i].size(); j++) {
                upperLeft = points[0][i][0];
                upperRight = points[0][i][1];
                lowerRight = points[0][i][2];
                lowerLeft = points[0][i][3];
                if (upperLeft.x < upperRight.x && upperLeft.x < lowerRight.x &&
                    upperLeft.y < lowerLeft.y && upperLeft.y < lowerRight.y &&
                    upperRight.x > upperLeft.x && upperRight.x > lowerLeft.x &&
                    upperRight.y < lowerLeft.y && upperRight.y < lowerRight.y &&
                    lowerLeft.x < upperRight.x && lowerLeft.x < lowerRight.x &&
                    lowerLeft.y > upperLeft.y && lowerLeft.y > upperRight.y &&
                    lowerRight.x > upperLeft.x && lowerRight.x > lowerLeft.x &&
                    lowerRight.y > upperLeft.y && lowerRight.y > upperRight.y) {
                    break;
                } else {
                    std::rotate(points[0][i].begin(), points[0][i].end()-1, points[0][i].end());
                }
            }   
        }
    }
    vector<RotatedRect> fakeBox(1);
    Point p;
    p = Point(upperRight-upperLeft);
    fakeBox[0].angle = atan2(-p.y, p.x);
    fakeBox[0].center = upperLeft + p*0.5 + Point(-p.y, p.x)*0.5*RATIO;
    fakeBox[0].size = Size((int)norm(p), (int)(norm(p)*RATIO));
    
    return fakeBox;
}

bool boxDetection::findBox(Mat& image, char boxColor, bool draw = false)
{
    Mat image_norm, image_blur, image_HSV;

    if (NORMALIZE) {
        normalize(image, image_norm, 0, 255, NORM_MINMAX);
    } else {
        image_norm = image;
    }
    if (PREFILTER) {
        GaussianBlur(image_norm, image_blur, Size(5, 5), 1);
        cvtColor(image_blur, image_HSV, COLOR_BGR2HSV);
    } else {
        image_blur = image_norm;
        cvtColor(image_norm, image_HSV, COLOR_BGR2HSV);
    }

    Mat mask;
    // Mask the HSV image
    mask = maskHSV(image_HSV, boxColor);

    if (PREFILTER) {
        int open_size = 2;
        int open_type = MORPH_ELLIPSE;
        Mat openElement = getStructuringElement(open_type, Size(2 * open_size + 1, 2 * open_size + 1), Point(open_size, open_size));
        morphologyEx(mask, mask, MORPH_OPEN, openElement);

        int close_size = 2;
        int close_type = MORPH_ELLIPSE;
        Mat closeElement = getStructuringElement(close_type, Size(2 * close_size + 1, 2 * close_size + 1), Point(close_size, close_size));
        morphologyEx(mask, mask, MORPH_CLOSE, closeElement);
    }

    // Find contours of boxes from the masked image
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

    vector<vector<vector<Point>>> boxCountours;
    Size imageSize = Size(mask.cols, mask.rows);
    boxCountours = extractBoxCountours(imageSize, contours, hierarchy);

    // Find minimum area rectangles of the contours
    vector<vector<RotatedRect>> minRect(2);
    minRect[0].resize(boxCountours[0].size());
    minRect[1].resize(boxCountours[1].size());
    for (int i = 0; i < boxCountours[0].size(); i++) {
        minRect[0][i] = minAreaRect(Mat(boxCountours[0][i]));
        minRect[1][i] = minAreaRect(Mat(boxCountours[1][i]));
    }

    /*
    vector<vector<vector<Point>>> approx = boxCountours;
    vector<vector<vector<int>>> convex(2);
    convex[0].resize(boxCountours[0].size());
    convex[1].resize(boxCountours[1].size());
    vector<vector<vector<Point>>> convexPoint = approx;
    vector<vector<Vec4i>> innerDefects(boxCountours[0].size());
    vector<vector<Vec4i>> outerDefects(boxCountours[1].size());

    for (int i = 0; i < boxCountours[0].size(); i++) {
        approxPolyDP(boxCountours[0][i], approx[0][i], 0.01 * arcLength(boxCountours[0][i], true), true);
        convexHull(approx[0][i], convex[0][i], false, true);
        if (convex[0][i].size() > 2) {
            convexityDefects(approx[0][i], convex[0][i], outerDefects[i]);
        }
        convexHull(approx[0][i], convexPoint[0][i], false, true);
    }
    for (int i = 0; i < boxCountours[1].size(); i++) {
        approxPolyDP(boxCountours[1][i], approx[1][i], 0.01 * arcLength(boxCountours[1][i], true), true);
        convexHull(approx[1][i], convex[1][i], false, true);
        if (convex[1][i].size() > 2) {
            convexityDefects(approx[0][i], convex[0][i], innerDefects[i]);
        }
        convexHull(approx[1][i], convexPoint[1][i], false, true);
    }
    */

    vector<vector<double>> angle = calcAngle(minRect[1]);
    // cout << minRect[1].size() << endl;

    int bestBoxIndex = -1;
    if (minRect[1].size() > 0) {
        bestBoxIndex = findBestBox(minRect[1], imageSize);

        box.angle = angle[bestBoxIndex][0];
        box.x = round(minRect[1][bestBoxIndex].center.x);
        box.y = round(minRect[1][bestBoxIndex].center.y);
        box.w = (box.angle < 0) ? round(minRect[1][bestBoxIndex].size.height) : round(minRect[1][bestBoxIndex].size.width);
        box.h = (box.angle < 0) ? round(minRect[1][bestBoxIndex].size.width) : round(minRect[1][bestBoxIndex].size.height);

        // cout << "BestBox width " << minRect[1][bestBoxIndex].size.width << endl;
        // cout << "BestBox height " << minRect[1][bestBoxIndex].size.height << endl;
        // cout << "BestBox angle " << minRect[1][bestBoxIndex].angle << endl;
        // cout << "BestBox modified angle " << angle[bestBoxIndex][0] * 180/CV_PI << endl;
    }

    if (draw) {
        RNG rng(1234);

        Mat contours = drawContour(image, boxCountours);
        Mat boxes = drawBoundingBoxes(image, minRect);

        // Show in a window
        namedWindow("Source", WINDOW_AUTOSIZE);
        imshow("Source", image);
        namedWindow("GaussianBlur", WINDOW_AUTOSIZE);
        imshow("GaussianBlur", image_blur);
        namedWindow("Mask", WINDOW_AUTOSIZE);
        imshow("Mask", mask);
        namedWindow("Contours", WINDOW_AUTOSIZE);
        imshow("Contours", contours);
        namedWindow("BoundingBoxes", WINDOW_AUTOSIZE);
        imshow("BoundingBoxes", boxes);

        
        char buffer[7];

        /*
        Mat corners = image.clone();
        for (size_t i = 0; i < convexPoint[0].size(); i++) {
            Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
            for (size_t j = 0; j < convexPoint[0][i].size(); j++) {
                circle(corners, convexPoint[0][i][j], 10, color, 3);
            }
            // cout << "Defect size: " << outerDefects[i].size() << endl;
            for (size_t j = 0; j < outerDefects[i].size(); j++) {
                drawMarker(corners, approx[0][i][outerDefects[i][j][0] + 1], color, MARKER_CROSS, 20, 5);
                // cout << approx[0][i][outerDefects[i][j][0]] << endl;
                // cout << outerDefects[i][j] << endl;
            }
            drawMarker(corners, minRect[0][i].center, color, MARKER_TILTED_CROSS, 20, 5);

            sprintf(buffer, "%3.2f", minRect[0][i].angle);
            putText(corners, buffer, minRect[0][i].center + Point2f(20, 0), FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 0), 2);
        }
        namedWindow("Corners", WINDOW_AUTOSIZE);
        imshow("Corners", corners);
        
        Mat centers;
        centers = image.clone();
        for (size_t i = 0; i < convexPoint[1].size(); i++) {
            Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
            drawMarker(centers, minRect[1][i].center, color, MARKER_TILTED_CROSS, 20, 5);
            sprintf(buffer, "%3.2f", angle[i][0] * 180 / CV_PI);
            putText(centers, buffer, minRect[1][i].center + Point2f(20, 0), FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 0), 2);
        }
        namedWindow("Centers", WINDOW_AUTOSIZE);
        imshow("Centers", centers);
        */

        if (bestBoxIndex != -1) {
            Mat bestBoxImage;
            bestBoxImage = image.clone();
            Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
            drawMarker(bestBoxImage, minRect[1][bestBoxIndex].center, color, MARKER_TILTED_CROSS, 20, 5);
            sprintf(buffer, "%3.2f", angle[bestBoxIndex][0] * 180 / CV_PI);
            putText(bestBoxImage, buffer, minRect[1][bestBoxIndex].center + Point2f(20, 0), FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 0), 2);
            namedWindow("BestBox", WINDOW_AUTOSIZE);
            imshow("BestBox", bestBoxImage);
        }
    }

    if (bestBoxIndex != -1) {
        return true;
    }
    else {
        return false;
    }
}

bool boxDetection::updateBox(Mat& image, char boxColor, bool draw = false)
{
    Mat image_norm, image_blur, image_HSV;

    if (NORMALIZE) {
        normalize(image, image_norm, 0, 255, NORM_MINMAX);
    } else {
        image_norm = image;
    }
    if (PREFILTER) {
        GaussianBlur(image_norm, image_blur, Size(5, 5), 1);
        cvtColor(image_blur, image_HSV, COLOR_BGR2HSV);
    } else {
        image_blur = image_norm;
        cvtColor(image_norm, image_HSV, COLOR_BGR2HSV);
    }

    Mat mask;
    // Mask the HSV image
    mask = maskHSV(image_HSV, boxColor);

    if (PREFILTER) {
        int open_size = 2;
        int open_type = MORPH_ELLIPSE;
        Mat openElement = getStructuringElement(open_type, Size(2 * open_size + 1, 2 * open_size + 1), Point(open_size, open_size));
        morphologyEx(mask, mask, MORPH_OPEN, openElement);

        int close_size = 2;
        int close_type = MORPH_ELLIPSE;
        Mat closeElement = getStructuringElement(close_type, Size(2 * close_size + 1, 2 * close_size + 1), Point(close_size, close_size));
        morphologyEx(mask, mask, MORPH_CLOSE, closeElement);
    }

    // Find contours of boxes from the masked image
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

    vector<vector<vector<Point>>> boxCountours;
    Size imageSize = Size(mask.cols, mask.rows);
    boxCountours = extractBoxCountours(imageSize, contours, hierarchy);

    // Find minimum area rectangles of the contours
    vector<vector<RotatedRect>> minRect(2);
    minRect[0].resize(boxCountours[0].size());
    minRect[1].resize(boxCountours[1].size());
    for (int i = 0; i < boxCountours[0].size(); i++) {
        minRect[0][i] = minAreaRect(Mat(boxCountours[0][i]));
        minRect[1][i] = minAreaRect(Mat(boxCountours[1][i]));
    }

    /*
    vector<vector<vector<Point>>> approx = boxCountours;
    vector<vector<vector<int>>> convex(2);
    convex[0].resize(boxCountours[0].size());
    convex[1].resize(boxCountours[1].size());
    vector<vector<vector<Point>>> convexPoint = approx;
    vector<vector<Vec4i>> innerDefects(boxCountours[0].size());
    vector<vector<Vec4i>> outerDefects(boxCountours[1].size());

    for (int i = 0; i < boxCountours[0].size(); i++) {
        approxPolyDP(boxCountours[0][i], approx[0][i], 0.01 * arcLength(boxCountours[0][i], true), true);
        convexHull(approx[0][i], convex[0][i], false, true);
        if (convex[0][i].size() > 2) {
            convexityDefects(approx[0][i], convex[0][i], outerDefects[i]);
        }
        convexHull(approx[0][i], convexPoint[0][i], false, true);
    }
    for (int i = 0; i < boxCountours[1].size(); i++) {
        approxPolyDP(boxCountours[1][i], approx[1][i], 0.01 * arcLength(boxCountours[1][i], true), true);
        convexHull(approx[1][i], convex[1][i], false, true);
        if (convex[1][i].size() > 2) {
            convexityDefects(approx[0][i], convex[0][i], innerDefects[i]);
        }
        convexHull(approx[1][i], convexPoint[1][i], false, true);
    }
    */

    vector<vector<double>> angle = calcAngle(minRect[1]);

    int boxInROI = findBoxInROI(minRect[1], ROISCALE);

    if (boxInROI != -1) {
        box.angle = angle[boxInROI][0];
        box.x = round(minRect[1][boxInROI].center.x);
        box.y = round(minRect[1][boxInROI].center.y);
        box.w = (box.angle < 0) ? round(minRect[1][boxInROI].size.height) : round(minRect[1][boxInROI].size.width);
        box.h = (box.angle < 0) ? round(minRect[1][boxInROI].size.width) : round(minRect[1][boxInROI].size.height);
    }

    if (draw) {
        RNG rng(1234);

        Mat contours = drawContour(image, boxCountours);
        Mat boxes = drawBoundingBoxes(image, minRect);
        Mat roi = drawROI(image, minRect[1], ROISCALE);

        // Show in a window
        namedWindow("Source", WINDOW_AUTOSIZE);
        imshow("Source", image);
        namedWindow("GaussianBlur", WINDOW_AUTOSIZE);
        imshow("GaussianBlur", image_blur);
        namedWindow("Mask", WINDOW_AUTOSIZE);
        imshow("Mask", mask);
        namedWindow("Contours", WINDOW_AUTOSIZE);
        imshow("Contours", contours);
        namedWindow("BoundingBoxes", WINDOW_AUTOSIZE);
        imshow("BoundingBoxes", boxes);
        namedWindow("ROI", WINDOW_AUTOSIZE);
        imshow("ROI", roi);

        char buffer[7];

        /*
        Mat corners = image.clone();
        for (size_t i = 0; i < convexPoint[0].size(); i++) {
            Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
            for (size_t j = 0; j < convexPoint[0][i].size(); j++) {
                circle(corners, convexPoint[0][i][j], 10, color, 3);
            }
            // cout << "Defect size: " << outerDefects[i].size() << endl;
            for (size_t j = 0; j < outerDefects[i].size(); j++) {
                drawMarker(corners, approx[0][i][outerDefects[i][j][0] + 1], color, MARKER_CROSS, 20, 5);
                // cout << approx[0][i][outerDefects[i][j][0]] << endl;
                // cout << outerDefects[i][j] << endl;
            }
            drawMarker(corners, minRect[0][i].center, color, MARKER_TILTED_CROSS, 20, 5);

            sprintf(buffer, "%3.2f", minRect[0][i].angle);
            putText(corners, buffer, minRect[0][i].center + Point2f(20, 0), FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 0), 2);
        }
        namedWindow("Corners", WINDOW_AUTOSIZE);
        imshow("Corners", corners);
        Mat centers;
        centers = image.clone();
        for (size_t i = 0; i < convexPoint[1].size(); i++) {
            Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
            drawMarker(centers, minRect[1][i].center, color, MARKER_TILTED_CROSS, 20, 5);
            sprintf(buffer, "%3.2f", angle[i][0] * 180 / CV_PI);
            putText(centers, buffer, minRect[1][i].center + Point2f(20, 0), FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 0), 2);
        }
        namedWindow("Centers", WINDOW_AUTOSIZE);
        imshow("Centers", centers);
        */

        if (boxInROI != -1) {
            Mat bestBoxImage;
            bestBoxImage = image.clone();
            Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
            drawMarker(bestBoxImage, minRect[1][boxInROI].center, color, MARKER_TILTED_CROSS, 20, 5);
            sprintf(buffer, "%3.2f", angle[boxInROI][0] * 180 / CV_PI);
            putText(bestBoxImage, buffer, minRect[1][boxInROI].center + Point2f(20, 0), FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 0), 2);
            namedWindow("BestBox", WINDOW_AUTOSIZE);
            imshow("BestBox", bestBoxImage);
        }
    }

    if (boxInROI != -1) {
        return true;
    }
    else {
        return false;
    }
}

bool boxDetection::updateBoxClose(Mat& image, char boxColor, bool draw = false)
{
    Mat image_norm, image_blur, image_HSV;

    if (NORMALIZE) {
        normalize(image, image_norm, 0, 255, NORM_MINMAX);
    } else {
        image_norm = image;
    }
    if (PREFILTER) {
        GaussianBlur(image_norm, image_blur, Size(5, 5), 1);
        cvtColor(image_blur, image_HSV, COLOR_BGR2HSV);
    } else {
        image_blur = image_norm;
        cvtColor(image_norm, image_HSV, COLOR_BGR2HSV);
    }

    Mat mask, mask_white, mask_white_flood;
    // Mask the HSV image
    mask = maskHSV(image_HSV, boxColor);
    mask_white_flood = mask.clone();
    mask_white = mask.clone();
    // mask_white = Mat::zeros(Size(mask.cols + 2, mask.rows + 2), CV_8U);
    // floodFill(mask, Point(box.x, box.y), 255, 0, Scalar(0), Scalar(0), FLOODFILL_MASK_ONLY);
    floodFill(mask_white_flood, Point(box.x, min(box.y, mask.rows-1)), 100);
    inRange(mask_white_flood, 90, 120, mask_white);
    if (PREFILTER) {
        int open_size = 2;
        int open_type = MORPH_ELLIPSE;
        Mat openElement = getStructuringElement(open_type, Size(2 * open_size + 1, 2 * open_size + 1), Point(open_size, open_size));
        morphologyEx(mask, mask, MORPH_OPEN, openElement);

        int close_size = 2;
        int close_type = MORPH_ELLIPSE;
        Mat closeElement = getStructuringElement(close_type, Size(2 * close_size + 1, 2 * close_size + 1), Point(close_size, close_size));
        morphologyEx(mask, mask, MORPH_CLOSE, closeElement);
    }

    // Find contours of boxes from the masked image
    vector<vector<Point>> contours, contours_white;
    vector<Vec4i> hierarchy, hierarchy_white;
    findContours(mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
    findContours(mask_white, contours_white, hierarchy_white, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

    vector<vector<vector<Point>>> boxCountours, boxCountoursWhite;
    Size imageSize = Size(mask.cols, mask.rows);
    boxCountours = extractBoxCountours(imageSize, contours, hierarchy);
    boxCountoursWhite = extractBoxCountoursWhite(imageSize, contours_white, hierarchy_white);

    // Find minimum area rectangles of the contours
    vector<vector<RotatedRect>> minRect(2);
    minRect[0].resize(boxCountours[0].size());
    minRect[1].resize(boxCountours[1].size());
    for (int i = 0; i < boxCountours[0].size(); i++) {
        minRect[0][i] = minAreaRect(Mat(boxCountours[0][i]));
        minRect[1][i] = minAreaRect(Mat(boxCountours[1][i]));
    }

    /*
    vector<vector<vector<Point>>> approx = boxCountours;
    vector<vector<vector<int>>> convex(2);
    convex[0].resize(boxCountours[0].size());
    convex[1].resize(boxCountours[1].size());
    vector<vector<vector<Point>>> convexPoint = approx;
    vector<vector<Vec4i>> innerDefects(boxCountours[0].size());
    vector<vector<Vec4i>> outerDefects(boxCountours[1].size());

    for (int i = 0; i < boxCountours[0].size(); i++) {
        approxPolyDP(boxCountours[0][i], approx[0][i], 0.01 * arcLength(boxCountours[0][i], true), true);
        convexHull(approx[0][i], convex[0][i], false, true);
        if (convex[0][i].size() > 2) {
            convexityDefects(approx[0][i], convex[0][i], outerDefects[i]);
        }
        convexHull(approx[0][i], convexPoint[0][i], false, true);
    }
    for (int i = 0; i < boxCountours[1].size(); i++) {
        approxPolyDP(boxCountours[1][i], approx[1][i], 0.01 * arcLength(boxCountours[1][i], true), true);
        convexHull(approx[1][i], convex[1][i], false, true);
        if (convex[1][i].size() > 2) {
            convexityDefects(approx[0][i], convex[0][i], innerDefects[i]);
        }
        convexHull(approx[1][i], convexPoint[1][i], false, true);
    }
    */

    vector<vector<vector<Point>>> approxWhite = boxCountoursWhite;
    vector<vector<vector<int>>> convexWhite(1);
    convexWhite[0].resize(boxCountoursWhite[0].size());
    vector<vector<vector<Point>>> convexPointWhite = approxWhite;
    vector<vector<Vec4i>> outerDefectsWhite(boxCountoursWhite[0].size());
    
    try 
    {
    for (int i = 0; i < boxCountoursWhite[0].size(); i++) {
        approxPolyDP(boxCountoursWhite[0][i], approxWhite[0][i], 0.03 * arcLength(boxCountoursWhite[0][i], true), true);
        convexHull(approxWhite[0][i], convexWhite[0][i], false, true);
        if (convexWhite[0][i].size() > 2) {
            convexityDefects(approxWhite[0][i], convexWhite[0][i], outerDefectsWhite[i]);
        }
        convexHull(approxWhite[0][i], convexPointWhite[0][i], false, true);
    }
    }
    catch(cv::Exception &e)
    {
      const char *err_msg = e.what();
      std::cout << "SH: exception caught: " << err_msg << std::endl;
    }

    vector<vector<double>> angle = calcAngle(minRect[1]);

    int boxInROI = findBoxInROI(minRect[1], ROISCALE);    

    vector<RotatedRect> fakeBox(1);
    int fakeBoxInROI = -1;

    if (boxInROI != -1) {
        box.angle = angle[boxInROI][0];
        box.x = round(minRect[1][boxInROI].center.x);
        box.y = round(minRect[1][boxInROI].center.y);
        box.w = (box.angle < 0) ? round(minRect[1][boxInROI].size.height) : round(minRect[1][boxInROI].size.width);
        box.h = (box.angle < 0) ? round(minRect[1][boxInROI].size.width) : round(minRect[1][boxInROI].size.height);
    } else if (norm(box.angle) < CV_PI/2) {
        fakeBox = findFakeBox(convexPointWhite, boxColor); 
        fakeBoxInROI = findBoxInROI(fakeBox, ROISCALE);
    }
    
    if (fakeBoxInROI != -1) {
        box.angle = fakeBox[fakeBoxInROI].angle;
        box.x = round(fakeBox[fakeBoxInROI].center.x);
        box.y = round(fakeBox[fakeBoxInROI].center.y);
        box.w = round(fakeBox[fakeBoxInROI].size.width);
        box.h = round(fakeBox[fakeBoxInROI].size.height);
    }
    if (draw) {
        RNG rng(1234);

        Mat contours = drawContour(image, boxCountours);
        Mat boxes = drawBoundingBoxes(image, minRect);
        Mat roi = drawROI(image, minRect[1], ROISCALE);

        // Show in a window
        namedWindow("Source", WINDOW_AUTOSIZE);
        imshow("Source", image);
        namedWindow("GaussianBlur", WINDOW_AUTOSIZE);
        imshow("GaussianBlur", image_blur);
        namedWindow("Mask", WINDOW_AUTOSIZE);
        imshow("Mask", mask);
        namedWindow("Contours", WINDOW_AUTOSIZE);
        imshow("Contours", contours);
        namedWindow("FloodMask", WINDOW_AUTOSIZE);
        copyMakeBorder(mask_white_flood, mask_white_flood, 0, image.rows/2, 0, 0, BORDER_CONSTANT, Scalar(127, 127, 127));
        imshow("FloodMask", mask_white_flood);
        namedWindow("WhiteMask", WINDOW_AUTOSIZE);
        copyMakeBorder(mask_white, mask_white, 0, image.rows/2, 0, 0, BORDER_CONSTANT, Scalar(127, 127, 127));
        imshow("WhiteMask", mask_white);
        namedWindow("BoundingBoxes", WINDOW_AUTOSIZE);
        imshow("BoundingBoxes", boxes);
        namedWindow("ROI", WINDOW_AUTOSIZE);
        imshow("ROI", roi);

        char buffer[7];

        /*
        Mat corners = image.clone();
        for (size_t i = 0; i < convexPoint[0].size(); i++) {
            Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
            for (size_t j = 0; j < convexPoint[0][i].size(); j++) {
                circle(corners, convexPoint[0][i][j], 10, color, 3);
            }
            // cout << "Defect size: " << outerDefects[i].size() << endl;
            for (size_t j = 0; j < outerDefects[i].size(); j++) {
                drawMarker(corners, approx[0][i][outerDefects[i][j][0] + 1], color, MARKER_CROSS, 20, 5);
                // cout << approx[0][i][outerDefects[i][j][0]] << endl;
                // cout << outerDefects[i][j] << endl;
            }
            drawMarker(corners, minRect[0][i].center, color, MARKER_TILTED_CROSS, 20, 5);

            sprintf(buffer, "%3.2f", minRect[0][i].angle);
            putText(corners, buffer, minRect[0][i].center + Point2f(20, 0), FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 0), 2);
        }
        namedWindow("Corners", WINDOW_AUTOSIZE);
        imshow("Corners", corners);
        Mat centers;
        centers = image.clone();
        for (size_t i = 0; i < convexPoint[1].size(); i++) {
            Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
            drawMarker(centers, minRect[1][i].center, color, MARKER_TILTED_CROSS, 20, 5);
            sprintf(buffer, "%3.2f", angle[i][0] * 180 / CV_PI);
            putText(centers, buffer, minRect[1][i].center + Point2f(20, 0), FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 0), 2);
        }
        namedWindow("Centers", WINDOW_AUTOSIZE);
        imshow("Centers", centers);
        */

        if (boxInROI != -1) {
            Mat bestBoxImage;
            bestBoxImage = image.clone();
            Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
            drawMarker(bestBoxImage, Point(box.x, box.y), color, MARKER_TILTED_CROSS, 20, 5);
            sprintf(buffer, "%3.2f", box.angle * 180 / CV_PI);
            putText(bestBoxImage, buffer, Point2f(box.x, box.y) + Point2f(20, 0), FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 0), 2);
            namedWindow("BestBox", WINDOW_AUTOSIZE);
            copyMakeBorder(bestBoxImage, bestBoxImage, 0, image.rows/2, 0, 0, BORDER_CONSTANT, Scalar(127, 127, 127));
            imshow("BestBox", bestBoxImage);
        }
        if (fakeBoxInROI != -1) {
            Mat bestBoxImage;
            bestBoxImage = image.clone();
            Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
            copyMakeBorder(bestBoxImage, bestBoxImage, 0, image.rows/2, 0, 0, BORDER_CONSTANT, Scalar(127, 127, 127));
            drawMarker(bestBoxImage, Point(box.x, box.y), color, MARKER_TILTED_CROSS, 20, 5);
            sprintf(buffer, "%3.2f", box.angle * 180 / CV_PI);
            putText(bestBoxImage, buffer, Point2f(box.x, box.y) + Point2f(20, 0), FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 0), 2);
            namedWindow("BestBox", WINDOW_AUTOSIZE);
            imshow("BestBox", bestBoxImage);
        }
    }

    if (boxInROI != -1 || fakeBoxInROI != -1)  {
        return true;
    }
    else {
        return false;
    }
}

bool boxDetection::updateBoxZooming(Mat& image, char boxColor, bool draw = false)
{
    Mat image_norm, image_blur, image_HSV;

    if (NORMALIZE) {
        normalize(image, image_norm, 0, 255, NORM_MINMAX);
    } else {
        image_norm = image;
    }
    if (PREFILTER) {
        GaussianBlur(image_norm, image_blur, Size(5, 5), 1);
        cvtColor(image_blur, image_HSV, COLOR_BGR2HSV);
    } else {
        image_blur = image_norm;
        cvtColor(image_norm, image_HSV, COLOR_BGR2HSV);
    }

    Mat mask, mask_white;
    // Mask the HSV image
    mask = maskHSV(image_HSV, boxColor);
    mask_white = mask.clone();
    // mask_white = Mat::zeros(Size(mask.cols + 2, mask.rows + 2), CV_8U);
    // floodFill(mask, Point(box.x, box.y), 255, 0, Scalar(0), Scalar(0), FLOODFILL_MASK_ONLY);
    floodFill(mask_white, Point(box.x, min(box.y, mask.rows-1)), 127);
    inRange(mask_white, 100, 200, mask_white);

    if (PREFILTER) {
        int open_size = 2;
        int open_type = MORPH_ELLIPSE;
        Mat openElement = getStructuringElement(open_type, Size(2 * open_size + 1, 2 * open_size + 1), Point(open_size, open_size));
        morphologyEx(mask_white, mask_white, MORPH_OPEN, openElement);

        int close_size = 2;
        int close_type = MORPH_ELLIPSE;
        Mat closeElement = getStructuringElement(close_type, Size(2 * close_size + 1, 2 * close_size + 1), Point(close_size, close_size));
        morphologyEx(mask_white, mask_white, MORPH_CLOSE, closeElement);
    }

    // Find contours of boxes from the masked image
    vector<vector<Point>> contours_white;
    vector<Vec4i> hierarchy_white;
    findContours(mask_white, contours_white, hierarchy_white, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

    vector<vector<vector<Point>>> boxCountoursWhite;
    Size imageSize = Size(mask_white.cols, mask_white.rows);
    boxCountoursWhite = extractBoxCountoursWhite(imageSize, contours_white, hierarchy_white);

    vector<vector<vector<Point>>> approxWhite = boxCountoursWhite;
    vector<vector<vector<int>>> convexWhite(1);
    convexWhite[0].resize(boxCountoursWhite[0].size());
    vector<vector<vector<Point>>> convexPointWhite = approxWhite;
    vector<vector<Vec4i>> outerDefectsWhite(boxCountoursWhite[0].size());

    for (int i = 0; i < boxCountoursWhite[0].size(); i++) {
        approxPolyDP(boxCountoursWhite[0][i], approxWhite[0][i], 0.03 * arcLength(boxCountoursWhite[0][i], true), true);
        convexHull(approxWhite[0][i], convexWhite[0][i], false, true);
        if (convexWhite[0][i].size() > 2) {
            convexityDefects(approxWhite[0][i], convexWhite[0][i], outerDefectsWhite[i]);
        }
        convexHull(approxWhite[0][i], convexPointWhite[0][i], false, true);
    }   

    vector<RotatedRect> fakeBox(1);
    int fakeBoxInROI = -1;

    if (norm(box.angle) < CV_PI/2) {
        fakeBox = findFakeBox(convexPointWhite, boxColor); 
        fakeBoxInROI = findBoxInROI(fakeBox, ROISCALE);
    }
    
    if (fakeBoxInROI != -1) {
        box.angle = fakeBox[fakeBoxInROI].angle;
        box.x = (int)fakeBox[fakeBoxInROI].center.x;
        box.y = (int)fakeBox[fakeBoxInROI].center.y;
        box.w = (int)fakeBox[fakeBoxInROI].size.width;
        box.h = (int)fakeBox[fakeBoxInROI].size.height;
    }
    if (draw) {
        RNG rng(1234);
        Mat roi = drawROI(image, fakeBox, ROISCALE);

        // Show in a window
        namedWindow("Source", WINDOW_AUTOSIZE);
        imshow("Source", image);
        namedWindow("GaussianBlur", WINDOW_AUTOSIZE);
        imshow("GaussianBlur", image_blur);
        namedWindow("Mask", WINDOW_AUTOSIZE);
        imshow("Mask", mask_white);
        namedWindow("WhiteMask", WINDOW_AUTOSIZE);
        copyMakeBorder(mask_white, mask_white, 0, image.rows/2, 0, 0, BORDER_CONSTANT, Scalar(127, 127, 127));
        imshow("WhiteMask", mask_white);
        namedWindow("ROI", WINDOW_AUTOSIZE);
        imshow("ROI", roi);

        char buffer[7];
        if (fakeBoxInROI != -1) {
            Mat bestBoxImage;
            bestBoxImage = image.clone();
            Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
            copyMakeBorder(bestBoxImage, bestBoxImage, 0, image.rows/2, 0, 0, BORDER_CONSTANT, Scalar(127, 127, 127));
            drawMarker(bestBoxImage, Point(box.x, box.y), color, MARKER_TILTED_CROSS, 20, 5);
            sprintf(buffer, "%3.2f", box.angle * 180 / CV_PI);
            putText(bestBoxImage, buffer, Point2f(box.x, box.y) + Point2f(20, 0), FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 0), 2);
            namedWindow("BestBox", WINDOW_AUTOSIZE);
            imshow("BestBox", bestBoxImage);
        }
    }

    if (fakeBoxInROI != -1)  {
        return true;
    }
    else {
        return false;
    }
}

bool boxDetection::getBox(Mat &image, char boxColor) {
    if (boxFound == false) {
        if (DEBUG) cout << "Looking for box" << endl;

        boxFound = findBox(image, boxColor, false);
    } else if (boxFound == true) {
        if(DEBUG) cout << "Updating box" << endl;
        if (boxClose) {
            // boxUpdated = updateBoxClose(image, boxColor, false);
            boxUpdated = updateBoxClose(image, boxColor, false);
        } else {
            boxUpdated = updateBox(image, boxColor, false);
        }
        if (!boxUpdated) {
            frameMissed++;
            if(DEBUG) cout << "Frames Missed: " << frameMissed << endl;
        } if (boxNearBottomEdge(image)) {
            boxClose = true;
            if(DEBUG) cout << "Box near bottom of image" << endl;
        } else {
            boxClose = false;
        }
        if(DEBUG) cout << "Press enter to continue;";
        // if(DEBUG) cin.get();
    }
    if (frameMissed > 2) {
        boxFound = false;
        boxClose = false;
        boxUpdated = false;
        frameMissed = 0;
    }

    if (boxFound || boxUpdated) {
        return true;
    } else {
        return false;
    }
}

int boxDetection::findBestWall(vector<RotatedRect> minRect, Size imageSize)
{
    // First for the box at index 0
    int minIndex = 0;
    float areaMax = minRect[0].size.width * minRect[0].size.height;
    // Then compare with the rest
    for (int i = 1; i < minRect.size(); i++) {
        float area = minRect[i].size.width * minRect[i].size.height;
        if ((int)area > imageSize.width * imageSize.height / 100 ) {
            float ratio = minRect[i].size.width / minRect[i].size.height;
            if (ratio < 5.0 && ratio > 0.2) {
                if (area > areaMax) {
                    areaMax = area;
                    minIndex = i;
                }
            }
        }
    }
    return minIndex;
}

Mat getAffineTransformForRotatedRect(RotatedRect rr) {
    float angle, width, height;
    angle = rr.angle * M_PI / 180.0;
    width = rr.size.width;
    height = rr.size.height;
    
    // angle += M_PI; // you may want rotate it upsidedown
    float sinA = sin(angle), cosA = cos(angle);
    float data[6] = {
         cosA, sinA, width/2.0f - cosA * rr.center.x - sinA * rr.center.y,
        -sinA, cosA, height/2.0f - cosA * rr.center.y + sinA * rr.center.x};
    Mat rot_mat(2, 3, CV_32FC1, data);
    return rot_mat.clone();
}

int isValidWall(Mat distancemask) {
    double min, max;
    minMaxLoc(distancemask, &min, &max);
    Size size = Size(distancemask.cols, distancemask.rows);
    char p0 = distancemask.at<char>(0, 0);
    char p1 = distancemask.at<char>(0, size.width/4 - 1);
    char p2 = distancemask.at<char>(0, size.width/2 - 1);
    char p3 = distancemask.at<char>(0, size.width/4*3 - 1);
    char p4 = distancemask.at<char>(0, size.width - 1);
    char p5 = distancemask.at<char>(size.height - 1, 0 - 1);
    char p6 = distancemask.at<char>(size.height - 1, size.width/4 - 1);
    char p7 = distancemask.at<char>(size.height - 1, size.width/2 - 1);
    char p8 = distancemask.at<char>(size.height - 1, size.width/4*3 - 1);
    char p9 = distancemask.at<char>(size.height - 1, size.width - 1);

    double lower = 0.2;
    double upper = 0.8;
    if (p2 > max * upper) {
        // wall is M
        if (p1 > max * lower || p3 > max * lower || p5 > max * lower || p7 > max * lower || p9 > max * lower) {
            return 0;
        }
        if (p0 < max * upper || p2 < max * upper || p4 < max * upper || p6 < max * upper || p8 < max * upper) {
            return 0;
        }
        return 1;
    } else if (p2 < max * lower) {
        // wall is W
        if (p0 > max * lower || p2 > max * lower || p4 > max * lower || p6 > max * lower || p8 > max * lower) {
            return 0;
        }
        if (p1 < max * upper || p3 < max * upper || p5 < max * upper || p7 < max * upper || p9 < max * upper) {
            return 0;
        }
        return 2;
    } else {
        return 0;
    }
}

bool boxDetection::findWall(Mat& image, bool draw) {
    Mat image_norm, image_blur, image_HSV;

    if (NORMALIZE) {
        normalize(image, image_norm, 0, 255, NORM_MINMAX);
    } else {
        image_norm = image;
    }
    if (PREFILTER) {
        GaussianBlur(image_norm, image_blur, Size(5, 5), 1);
        cvtColor(image_blur, image_HSV, COLOR_BGR2HSV);
    } else {
        image_blur = image_norm;
        cvtColor(image_norm, image_HSV, COLOR_BGR2HSV);
    }

    Mat mask;
    // Mask the HSV image
    mask = maskHSV(image_HSV, 'Y');

    dilate(mask, mask, getStructuringElement(MORPH_RECT, Size(5, 5)));

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));

    vector<RotatedRect> minRect;
    minRect.resize(contours.size());
    for (int i = 0; i < contours.size(); i++) {
        minRect[i] = minAreaRect(Mat(contours[i]));
    }

    Size imageSize = Size(mask.cols, mask.rows);
    int bestWall = -1;
    if(minRect.size() > 0) {
        bestWall = findBestWall(minRect, imageSize);
    }

    Mat M;
    Mat rectmask = Mat(image.size(), CV_8UC1, Scalar(255));
    Mat distancemask;
    int isValid = 0;
    if (bestWall != -1) {
        M = getAffineTransformForRotatedRect(minRect[bestWall]);
        
        warpAffine(mask, rectmask, M, minRect[bestWall].size, INTER_CUBIC);
        if (minRect[bestWall].size.height > minRect[bestWall].size.width) {
            transpose(rectmask, rectmask);  
            flip(rectmask, rectmask,0);
        }
        
        bitwise_not(rectmask, rectmask);
        distanceTransform(rectmask, distancemask, DIST_L1, 3, CV_8UC1);

        isValid = isValidWall(distancemask);
        if (isValid) {
            if (minRect[bestWall].size.width > minRect[bestWall].size.height) {
                if (isValid == 2) {
                    wall.angle = - minRect[bestWall].angle * CV_PI / 180 - CV_PI;
                } else {
                    wall.angle = - minRect[bestWall].angle * CV_PI / 180;
                }
                wall.w = round(minRect[bestWall].size.width);
                wall.h = round(minRect[bestWall].size.height);
            } else {
                if (isValid == 2) {
                    wall.angle = CV_PI/2 - minRect[bestWall].angle * CV_PI / 180;
                } else {
                    wall.angle = - CV_PI/2 - minRect[bestWall].angle * CV_PI / 180;
                }
                wall.w = round(minRect[bestWall].size.height);
                wall.h = round(minRect[bestWall].size.width);
            }
            wall.x = round(minRect[bestWall].center.x);
            wall.y = round(minRect[bestWall].center.y);
        }

    }
    
    if (draw) {
        RNG rng(1234);

        Mat drawing1 = image.clone();
        for (int i = 0; i < minRect.size(); i++) {
            Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
            // rotated rectangle
            Point2f rect_points[4];
            minRect[bestWall].points(rect_points);
            for (int j = 0; j < 4; j++)
                line(drawing1, rect_points[j], rect_points[(j + 1) % 4], color, 4, 8);
            drawMarker(drawing1, minRect[i].center, color, MARKER_TILTED_CROSS, 20, 5);
        }

        Mat drawing3 = image.clone();
        for (int i = 0; i < contours.size(); i++) {
            Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
            // contour
            drawContours(drawing3, contours, i, color, 3, 8, vector<Vec4i>(), 0, Point());
        }

        // Show in a window
        namedWindow("Source", WINDOW_AUTOSIZE);
        imshow("Source", image);
        namedWindow("GaussianBlur", WINDOW_AUTOSIZE);
        imshow("GaussianBlur", image_blur);
        namedWindow("Mask", WINDOW_AUTOSIZE);
        imshow("Mask", mask);
        namedWindow("Contours", WINDOW_AUTOSIZE);
        imshow("Contours", drawing3);
        namedWindow("BoundingBoxes", WINDOW_AUTOSIZE);
        imshow("BoundingBoxes", drawing1);
        if (bestWall != -1) {
            Mat drawing2 = image.clone();
            Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
            // rotated rectangle
            Point2f rect_points[4];
            minRect[bestWall].points(rect_points);
            for (int i = 0; i < 4; i++)
                line(drawing2, rect_points[i], rect_points[(i + 1) % 4], color, 4, 8);
            drawMarker(drawing2, minRect[bestWall].center, color, MARKER_TILTED_CROSS, 20, 5);

            namedWindow("BestBox", WINDOW_AUTOSIZE);
            imshow("BestBox", drawing2);
            namedWindow("Test", WINDOW_AUTOSIZE);
            imshow("Test", rectmask);
            namedWindow("DistanceMask", WINDOW_AUTOSIZE);
            imshow("DistanceMask", distancemask);
        }
        

    }
    if (isValid) {
        return true;
    } else {
        return false;
    }
}
