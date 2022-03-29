#pragma once

#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

class findAngle
{
    public:
        struct boxPlacement
        {
            int x=-1, y=-1;
            double angle = 0;
        };
        boxPlacement box;
        class vectorIndex
        {
            public:
                int i;
                Point p;
        };
        int findBox(Mat &image, char boxColor, bool draw);
        int updateBox(Mat &image, char boxColor, bool draw);

    private:
        int max_value = 255;

        int LOWGREEN[3] = {40, 60, 55};
        int HIGHGREEN[3] = {100, max_value, max_value};

        int LOWRED[3] = {165, 60, 100};
        int HIGHRED[3] = {195, max_value, max_value};

        float ROISCALE = 2.0;

        RotatedRect previousBox;

        Mat maskHSV(Mat image_HSV, char boxColor);

        vector<vector<vector<Point>>> extractBoxCountours(Size imageSize, vector<vector<Point>> contours, vector<Vec4i>);

        Mat drawBoundingBoxes(Mat image, vector<vector<RotatedRect>> minRect, vector<vector<vector<Point>>> boxCountours, Mat mask, bool showContours);

        vector<vector<int>> findMinMax(vector<vector<Point>> convex, bool returnMax);

        vector<vector<vectorIndex>> getDistanceVectorIndex(vector<vector<Point>> convex);

        vector<vector<double>> calcAngle(vector<vector<vectorIndex>> distanceVectorIndex, vector<RotatedRect> innerMinRect);

        int findBestBox(vector<RotatedRect> innerMinRect, Size imageSize);

        int findBoxInROI(vector<RotatedRect> innerMinRect, float regionRadiusScale);
};

