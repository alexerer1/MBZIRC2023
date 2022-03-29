#pragma once

#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

class boxDetection
{
    public:
        struct boxPlacement
        {
            int x=-1, y=-1;
            int w=-1, h=-1;
            double angle = 0;
        };
        boxPlacement box;
        struct wallPlacement
        {
            int x=-1, y=-1;
            int w=-1, h=-1;
            double angle = 0;
        };
        boxPlacement wall;
        class vectorIndex
        {
            public:
                int i;
                Point p;
        };
        bool findBox(Mat &image, char boxColor, bool draw);
        bool updateBox(Mat &image, char boxColor, bool draw);
        bool updateBoxClose(Mat &image, char boxColor, bool draw);
        bool updateBoxZooming(Mat &image, char boxColor, bool draw);
        bool boxNearBottomEdge(Mat image);

        int frameMissed = 0;
        bool boxFound = false;
        bool boxUpdated = false;
        bool boxClose = false;
        bool getBox(Mat &image, char boxColor);

        bool findWall(Mat &image, bool draw);

        bool wallFound = false;
        bool wallUpdated = false;
        bool wallDown = false;
        bool getWall(Mat &image);

        const bool DEBUG = false;
        boxDetection(bool debug) : DEBUG(debug) {
            
        }
        boxDetection() {
            
        }

        
    private:
        int max_value = 255;

        int LOWGREEN[3] = {40, 60, 55};
        int HIGHGREEN[3] = {100, max_value, max_value};

        int LOWRED[3] = {165, 80, 100}; // {165, 60, 70}
        int HIGHRED[3] = {185, max_value, max_value}; // {195, max_value, max_value}

        int LOWBLUE[3] = {275, 90, 90};
        int HIGHBLUE[3] = {300, max_value, max_value};

        int LOWORANGE[3] = {185, 130, 130};
        int HIGHORANGE[3] = {215, max_value, max_value};

        int LOWWHITE[3] = {0, 0, 105}; // {0, 0, 150}
        int HIGHWHITE[3] = {360, 105, max_value}; // {360, 100, max_value}

        int LOWYELLOW[3] = {0, 120, 85};
        int HIGHYELLOW[3] = {50, max_value, max_value}; // {360, 100, max_value}

        bool PREFILTER = false;
        bool NORMALIZE = true;

        float ROISCALE = 1.5;

        float REDHEIGHTWIDTHRATIO = 150.0/200.0;
        float GREENHEIGHTWIDTHRATIO = 150.0/300.0;
        float BLUEHEIGHTWIDTHRATIO = 150.0/300.0;
        float ORANGEHEIGHTWIDTHRATIO = 150.0/190.0;

        Mat maskHSV(Mat image_HSV, char boxColor);

        vector<vector<vector<Point>>> extractBoxCountours(Size imageSize, vector<vector<Point>> contours, vector<Vec4i> hierarchy);
        vector<vector<vector<Point>>> extractBoxCountoursWhite(Size imageSize, vector<vector<Point>> contours, vector<Vec4i> hierarchy);

        Mat drawBoundingBoxes(Mat image, vector<vector<RotatedRect>> minRect);
        Mat drawContour(Mat image, vector<vector<vector<Point>>> boxCountours);
        Mat drawROI(Mat image, vector<RotatedRect> innerMinRect, float regionRadiusScale);

        vector<vector<int>> findMinMax(vector<vector<Point>> convex, bool returnMax);

        vector<vector<vectorIndex>> getDistanceVectorIndex(vector<vector<Point>> convex);

        vector<vector<double>> calcAngle(vector<RotatedRect> innerMinRect);

        int findBestBox(vector<RotatedRect> innerMinRect, Size imageSize);

        int findBoxInROI(vector<RotatedRect> innerMinRect, float regionRadiusScale);

        vector<RotatedRect> findFakeBox(vector<vector<vector<Point>>>, char boxColor);

        int findBestWall(vector<RotatedRect> minRect, Size imageSize);
};

