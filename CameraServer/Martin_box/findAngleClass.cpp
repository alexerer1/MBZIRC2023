#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <time.h>       /* clock_t, clock, CLOCKS_PER_SEC */
#include <stdio.h>      /* printf */

#include "findAngleClass.hpp"

using namespace std;
using namespace cv;

Mat findAngle::maskHSV(Mat image_HSV, char boxColor)
{
    Mat mask1, mask2;

    int* HSV_LOW;
    int* HSV_HIGH;

    switch (toupper(boxColor))
    {
    case 'G':
        HSV_LOW = LOWGREEN;
        HSV_HIGH = HIGHGREEN;
        break;
    case 'R':
        HSV_LOW = LOWRED;
        HSV_HIGH = HIGHRED;
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

vector<vector<vector<Point>>> findAngle::extractBoxCountours(Size imageSize, vector<vector<Point>> contours, vector<Vec4i> hierarchy)
{
    vector<vector<vector<Point>>> boxCountours(2);
    double area = 0;
    // Area threshold is set to 1/100 of the pixels in the image
    double areaThreshold = imageSize.width * imageSize.height / 300;
    for (int i = 0; i < contours.size(); i++) {
        area = contourArea(contours[i]);
        if (area > areaThreshold && hierarchy[i][3] != -1 && hierarchy[hierarchy[i][3]][3] == -1) {
            boxCountours[0].push_back(contours[hierarchy[i][3]]);
            boxCountours[1].push_back(contours[i]);
        }
    }
    return boxCountours;
}

Mat findAngle::drawBoundingBoxes(Mat image, vector<vector<RotatedRect>> minRect, vector<vector<vector<Point>>> boxCountours, Mat mask, bool showContours)
{
    RNG rng(1234);
    Mat drawing = image.clone();
    for (int i = 0; i < boxCountours[0].size(); i++) {
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        // contour
        drawContours(drawing, boxCountours[0], i, color, 1, 8, vector<Vec4i>(), 0, Point());
        // rotated rectangle
        Point2f rect_points[4];
        minRect[0][i].points(rect_points);
        for (int j = 0; j < 4; j++)
            line(drawing, rect_points[j], rect_points[(j + 1) % 4], color, 4, 8);
        drawMarker(drawing, minRect[0][i].center, color, MARKER_TILTED_CROSS, 20, 5);
    }
    for (int i = 0; i < boxCountours[1].size(); i++) {
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        // contour
        drawContours(drawing, boxCountours[1], i, color, 1, 8, vector<Vec4i>(), 0, Point());
        // rotated rectangle
        Point2f rect_points[4];
        minRect[1][i].points(rect_points);
        for (int j = 0; j < 4; j++)
            line(drawing, rect_points[j], rect_points[(j + 1) % 4], color, 4, 8);
        drawMarker(drawing, minRect[1][i].center, color, MARKER_TILTED_CROSS, 20, 5);
    }
    return drawing;
}

vector<vector<int>> findAngle::findMinMax(vector<vector<Point>> convex, bool returnMax) {
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
            } else {
                if (convex[i][j].y < yval[i][0]) {
                    yval[i][0] = convex[i][j].y;
                    yval[i][1] = j;
                }
            }
        }
    }
    return yval;
}

vector<vector<findAngle::vectorIndex>> findAngle::getDistanceVectorIndex(vector<vector<Point>> convex) {
    vector<vector<vectorIndex>> distanceVectorIndex(convex.size(), vector<vectorIndex>(2));
    double dist = 0;
    Point diff(0,0);
    for (int i = 0; i < convex.size(); i++) {
        for (int j = 0; j < convex[i].size(); j++) {
            if (j == convex[i].size()-1) {
                diff = convex[i][j] - convex[i][0];
            } else
            {
                diff = convex[i][j] - convex[i][j+1];
            }
            dist = norm(diff);
            if (dist > norm(distanceVectorIndex[i][0].p)) {
                distanceVectorIndex[i][1] = distanceVectorIndex[i][0];

                distanceVectorIndex[i][0].p = diff;
                distanceVectorIndex[i][0].i = j;
            } else if (dist > norm(distanceVectorIndex[i][1].p)){
                distanceVectorIndex[i][1].p = diff;
                distanceVectorIndex[i][1].i = j;
            }
        }
    }
    return distanceVectorIndex;
}

vector<vector<double>> findAngle::calcAngle(vector<vector<findAngle::vectorIndex>> distanceVectorIndex, vector<RotatedRect> innerMinRect) {
    vector<vector<double>> angle(distanceVectorIndex.size(), vector<double>(3));
    for (int i = 0; i < angle.size(); i++) {
        for (int j = 0; j < angle[i].size()-1; j++) {
            angle[i][j] = atan2(-distanceVectorIndex[i][j].p.x, -distanceVectorIndex[i][j].p.y);
            // limit to -90 to 90 degrees
            if (angle[i][j] < -CV_PI/2) {
                angle[i][j] += CV_PI;
            } else if (angle[i][j] > CV_PI/2) {
                angle[i][j] -= CV_PI;
            }
        }
        if (abs(angle[i][0] - angle[i][1]) > 0.8*CV_PI) {
            if (angle[i][1] < -CV_PI/4) {
                angle[i][1] += CV_PI;
            } else if (angle[i][1] > CV_PI/4) {
                angle[i][1] -= CV_PI;
            }
        }

        angle[i][2] = CV_PI/2-innerMinRect[i].angle*CV_PI/180;
        if (angle[i][2] < -CV_PI/2) {
                angle[i][2] += CV_PI;
            } else if (angle[i][2] > CV_PI/2) {
                angle[i][2] -= CV_PI;
            }

        if (abs(angle[i][0] - angle[i][2]) > CV_PI/3 && abs(angle[i][0] - angle[i][2]) < CV_PI*2/3) {
            angle[i][2] += CV_PI/2;
        }
        
    }
    return angle;
}

int findAngle::findBestBox(vector<RotatedRect> innerMinRect, Size imageSize) {
    // Find the center of the image
    Point2f center = (Point2f)(imageSize)/2;
    // First for the box at index 0
    int minIndex = 0;
    Point2f diff = center - innerMinRect[minIndex].center;
    float minDist = sqrt(diff.x*diff.x + diff.y*diff.y);
    // Then compare with the rest
    for (int i = 1; i < innerMinRect.size(); i++) {
        Point2f diff = center - innerMinRect[i].center;
        float dist =sqrt(diff.x*diff.x + diff.y*diff.y);
        if (dist < minDist) {
            minDist = dist;
            minIndex = i;
        }
    }
    return minIndex;
}

int findAngle::findBoxInROI(vector<RotatedRect> innerMinRect, float regionRadiusScale) {
    float radius = (previousBox.size.width + previousBox.size.height) / 2.0 * regionRadiusScale;
    // cout << radius << endl;
    int minIndex = -1;
    Point2f center = Point2f((float)box.x,(float)box.y);
    float minDist = radius;
    for (int i = 0; i < innerMinRect.size(); i++) {
        Point2f diff = center - innerMinRect[i].center;
        float dist =sqrt(diff.x*diff.x + diff.y*diff.y);
        if (dist < minDist) {
            minDist = dist;
            minIndex = i;
        }
    }
    
    return minIndex;
}

int findAngle::findBox(Mat &image, char boxColor, bool draw = false) {
    Mat image_norm, image_blur, image_HSV;

    normalize(image, image_norm, 0, 255, NORM_MINMAX);
    GaussianBlur(image_norm, image_blur, Size(1,1), 0);
    cvtColor(image_blur, image_HSV, COLOR_BGR2HSV);

    findAngle libAngle;

    Mat mask;
    // Mask the HSV image
    switch (toupper(boxColor)) {
    case 'G':
        mask = libAngle.maskHSV(image_HSV, boxColor);
        break;
    case 'R':
        mask = libAngle.maskHSV(image_HSV, boxColor);
        break;
    default:
        break;
    }
    // int erosion_size = 3;
    // int erosion_type = MORPH_ELLIPSE;
    // Mat element = getStructuringElement( erosion_type,
    //                 Size( 2*erosion_size + 1, 2*erosion_size+1 ),
    //                 Point( erosion_size, erosion_size ) );
    // erode(mask, mask, element, Point(-1,-1), 5);
    int open_size = 1;
    int open_type = MORPH_ELLIPSE;
    Mat openElement = getStructuringElement( open_type,
                    Size( 2*open_size + 1, 2*open_size+1 ),
                    Point( open_size, open_size ) );
    morphologyEx(mask, mask, MORPH_OPEN, openElement);

    int close_size = 1;
    int close_type = MORPH_ELLIPSE;
    Mat closeElement = getStructuringElement( close_type,
                    Size( 2*close_size + 1, 2*close_size+1 ),
                    Point( close_size, close_size ) );
    morphologyEx(mask, mask, MORPH_CLOSE, closeElement);

    int erosion_size = 1;
    int erosion_type = MORPH_ELLIPSE;
    Mat erodeElement = getStructuringElement( erosion_type,
                    Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                    Point( erosion_size, erosion_size ) );
    erode(mask, mask, erodeElement, Point(-1,-1), 1);

    // Find contours of boxes from the masked image
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE , Point(0, 0));

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

    vector<vector<vectorIndex>> distanceVectorIndexOuter = libAngle.getDistanceVectorIndex(convexPoint[0]);

    vector<vector<double>> angle = libAngle.calcAngle(distanceVectorIndexOuter, minRect[1]);
    // cout << minRect[1].size() << endl;

    int bestBoxIndex = -1;
    if (minRect[1].size() > 0) {
        int bestBoxIndex = findBestBox(minRect[1], imageSize);

        box.angle = angle[bestBoxIndex][2];
        box.x = (int)minRect[1][bestBoxIndex].center.x;
        box.y = (int)minRect[1][bestBoxIndex].center.y;

        previousBox = minRect[1][bestBoxIndex];
    }
    

    if (draw) {
        RNG rng(1234);

        Mat drawing = libAngle.drawBoundingBoxes(image, minRect, boxCountours, mask, true);

        image = drawing.clone();

        return 0;
        // Show in a window
        namedWindow("Source", WINDOW_AUTOSIZE);
        imshow("Source", image);
        namedWindow("Gaussian blur", WINDOW_AUTOSIZE);
        imshow("Gaussian blur", image_blur);
        namedWindow("Mask", WINDOW_AUTOSIZE);
        imshow("Mask", mask);
        namedWindow("Contours", WINDOW_AUTOSIZE);
        imshow("Contours", drawing);

        char buffer[7];
        Mat corners = image.clone();
        for (size_t i = 0; i < convexPoint[0].size(); i++) {
            Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
            for (size_t j = 0; j < convexPoint[0][i].size(); j++) {
                circle(corners, convexPoint[0][i][j], 10, color, 3);
            }
            // cout << "Defect size: " << outerDefects[i].size() << endl;
            for (size_t j = 0; j < outerDefects[i].size(); j++) {
                drawMarker(corners, approx[0][i][outerDefects[i][j][0]+1], color, MARKER_CROSS, 20, 5);
                // cout << approx[0][i][outerDefects[i][j][0]] << endl;
                // cout << outerDefects[i][j] << endl;
            }
            drawMarker(corners, minRect[0][i].center, color, MARKER_TILTED_CROSS, 20, 5);

            sprintf(buffer, "%3.2f", minRect[0][i].angle);
            putText(corners, buffer, minRect[0][i].center+Point2f(20,0), FONT_HERSHEY_PLAIN, 2, Scalar(0,0,0), 2);

            sprintf(buffer, "%3.2f", angle[i][0]*180/CV_PI);
            putText(corners, buffer, convexPoint[0][i][distanceVectorIndexOuter[i][0].i]+Point(20,0), FONT_HERSHEY_PLAIN, 2, Scalar(0,0,0), 2);
            sprintf(buffer, "%3.2f", angle[i][1]*180/CV_PI);
            putText(corners, buffer, convexPoint[0][i][distanceVectorIndexOuter[i][1].i]+Point(20,0), FONT_HERSHEY_PLAIN, 2, Scalar(0,0,0), 2);
        }
        namedWindow("Corners", WINDOW_AUTOSIZE);
        imshow("Corners", corners);
        Mat centers;
        centers = image.clone();
        for (size_t i = 0; i < convexPoint[1].size(); i++) {
            Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
            drawMarker(centers, minRect[1][i].center, color, MARKER_TILTED_CROSS, 20, 5);
            sprintf(buffer, "%3.2f", angle[i][2]*180/CV_PI);
            putText(centers, buffer, minRect[1][i].center+Point2f(20,0), FONT_HERSHEY_PLAIN, 2, Scalar(0,0,0), 2);
        }
        namedWindow("Centers", WINDOW_AUTOSIZE);
        imshow("Centers", centers);

        if (bestBoxIndex != -1) {
            Mat bestBoxImage;
            bestBoxImage = image.clone();
            Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
            drawMarker(bestBoxImage, minRect[1][bestBoxIndex].center, color, MARKER_TILTED_CROSS, 20, 5);
            sprintf(buffer, "%3.2f", angle[bestBoxIndex][2]*180/CV_PI);
            putText(bestBoxImage, buffer, minRect[1][bestBoxIndex].center+Point2f(20,0), FONT_HERSHEY_PLAIN, 2, Scalar(0,0,0), 2);
            namedWindow("Best Box", WINDOW_AUTOSIZE);
            imshow("Best Box", bestBoxImage);
        }
    }
    return 0;
}

int findAngle::updateBox(Mat &image, char boxColor, bool draw = false) {
    Mat image_norm, image_blur, image_HSV;

    normalize(image, image_norm, 0, 255, NORM_MINMAX);
    GaussianBlur(image_norm, image_blur, Size(1,1), 0);
    cvtColor(image_blur, image_HSV, COLOR_BGR2HSV);

    findAngle libAngle;

    Mat mask;
    // Mask the HSV image
    switch (toupper(boxColor)) {
    case 'G':
        mask = libAngle.maskHSV(image_HSV, boxColor);
        break;
    case 'R':
        mask = libAngle.maskHSV(image_HSV, boxColor);
        break;
    default:
        break;
    }
    // int erosion_size = 3;
    // int erosion_type = MORPH_ELLIPSE;
    // Mat element = getStructuringElement( erosion_type,
    //                 Size( 2*erosion_size + 1, 2*erosion_size+1 ),
    //                 Point( erosion_size, erosion_size ) );
    // erode(mask, mask, element, Point(-1,-1), 5);
    int open_size = 1;
    int open_type = MORPH_ELLIPSE;
    Mat openElement = getStructuringElement( open_type,
                    Size( 2*open_size + 1, 2*open_size+1 ),
                    Point( open_size, open_size ) );
    morphologyEx(mask, mask, MORPH_OPEN, openElement);

    int close_size = 1;
    int close_type = MORPH_ELLIPSE;
    Mat closeElement = getStructuringElement( close_type,
                    Size( 2*close_size + 1, 2*close_size+1 ),
                    Point( close_size, close_size ) );
    morphologyEx(mask, mask, MORPH_CLOSE, closeElement);

    int erosion_size = 1;
    int erosion_type = MORPH_ELLIPSE;
    Mat erodeElement = getStructuringElement( erosion_type,
                    Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                    Point( erosion_size, erosion_size ) );
    erode(mask, mask, erodeElement, Point(-1,-1), 1);

    // Find contours of boxes from the masked image
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE , Point(0, 0));

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

    vector<vector<vectorIndex>> distanceVectorIndexOuter = libAngle.getDistanceVectorIndex(convexPoint[0]);

    vector<vector<double>> angle = libAngle.calcAngle(distanceVectorIndexOuter, minRect[1]);

    int boxInROI = findBoxInROI(minRect[1], ROISCALE);

    if (boxInROI != -1) {
        box.angle = angle[boxInROI][2];
        box.x = (int)minRect[1][boxInROI].center.x;
        box.y = (int)minRect[1][boxInROI].center.y;
    }
    

    if (draw) {
        RNG rng(1234);

        Mat drawing = libAngle.drawBoundingBoxes(image, minRect, boxCountours, mask, true);

        image = drawing.clone();
        return 0;

        // Show in a window
        namedWindow("Source", WINDOW_AUTOSIZE);
        imshow("Source", image);
        namedWindow("Gaussian blur", WINDOW_AUTOSIZE);
        imshow("Gaussian blur", image_blur);
        namedWindow("Mask", WINDOW_AUTOSIZE);
        imshow("Mask", mask);
        namedWindow("Contours", WINDOW_AUTOSIZE);
        imshow("Contours", drawing);

        char buffer[7];
        Mat corners = image.clone();
        for (size_t i = 0; i < convexPoint[0].size(); i++) {
            Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
            for (size_t j = 0; j < convexPoint[0][i].size(); j++) {
                circle(corners, convexPoint[0][i][j], 10, color, 3);
            }
            // cout << "Defect size: " << outerDefects[i].size() << endl;
            for (size_t j = 0; j < outerDefects[i].size(); j++) {
                drawMarker(corners, approx[0][i][outerDefects[i][j][0]+1], color, MARKER_CROSS, 20, 5);
                // cout << approx[0][i][outerDefects[i][j][0]] << endl;
                // cout << outerDefects[i][j] << endl;
            }
            drawMarker(corners, minRect[0][i].center, color, MARKER_TILTED_CROSS, 20, 5);

            sprintf(buffer, "%3.2f", minRect[0][i].angle);
            putText(corners, buffer, minRect[0][i].center+Point2f(20,0), FONT_HERSHEY_PLAIN, 2, Scalar(0,0,0), 2);

            sprintf(buffer, "%3.2f", angle[i][0]*180/CV_PI);
            putText(corners, buffer, convexPoint[0][i][distanceVectorIndexOuter[i][0].i]+Point(20,0), FONT_HERSHEY_PLAIN, 2, Scalar(0,0,0), 2);
            sprintf(buffer, "%3.2f", angle[i][1]*180/CV_PI);
            putText(corners, buffer, convexPoint[0][i][distanceVectorIndexOuter[i][1].i]+Point(20,0), FONT_HERSHEY_PLAIN, 2, Scalar(0,0,0), 2);
        }
        namedWindow("Corners", WINDOW_AUTOSIZE);
        imshow("Corners", corners);
        Mat centers;
        centers = image.clone();
        for (size_t i = 0; i < convexPoint[1].size(); i++) {
            Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
            drawMarker(centers, minRect[1][i].center, color, MARKER_TILTED_CROSS, 20, 5);
            sprintf(buffer, "%3.2f", angle[i][2]*180/CV_PI);
            putText(centers, buffer, minRect[1][i].center+Point2f(20,0), FONT_HERSHEY_PLAIN, 2, Scalar(0,0,0), 2);
        }
        namedWindow("Centers", WINDOW_AUTOSIZE);
        imshow("Centers", centers);

        if (boxInROI != -1) {
            Mat bestBoxImage;
            bestBoxImage = image.clone();
            Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
            drawMarker(bestBoxImage, minRect[1][boxInROI].center, color, MARKER_TILTED_CROSS, 20, 5);
            sprintf(buffer, "%3.2f", angle[boxInROI][2]*180/CV_PI);
            putText(bestBoxImage, buffer, minRect[1][boxInROI].center+Point2f(20,0), FONT_HERSHEY_PLAIN, 2, Scalar(0,0,0), 2);
            namedWindow("Best Box", WINDOW_AUTOSIZE);
            imshow("Best Box", bestBoxImage);
        }
        
    }
    return 0;
}