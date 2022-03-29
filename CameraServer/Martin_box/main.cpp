#include <iostream>
#include <time.h>       /* clock_t, clock, CLOCKS_PER_SEC */
#include <stdio.h>
#include <unistd.h>
#include "boxDetection.hpp"

RNG rng(1234);

int main(int argc, char* argv[])
{
    boxDetection libBoxDetection;
    String videoName = "~/data/videos/outdoor_box_test_1.avi";
    char boxColor = 'r';
    if (argc > 1){
        String tmp = argv[1];
        boxColor = tmp[0];
    }
    if (argc > 2) {
        videoName = argv[2];
    }

    cv::VideoCapture cap(videoName);

    if(!cap.isOpened()){
        cout << "Error opening video stream or file" << endl;
        return -1;
    }
    Mat frame;
    clock_t t;

    bool boxFound = false;
    bool boxUpdated = false;
    int framesSinceBox = 0;
    while (1) {

        t = clock();

        cap >> frame;
        if (frame.empty()) {
            break;
        }
        cv::imshow("Frame", frame);

        if (boxFound == false) {
            cout << "Looking for box" << endl;
            boxFound = libBoxDetection.findBox(frame, boxColor, true);
        } else if (boxFound == true) {
            cout << "Updating box" << endl;
            boxUpdated = libBoxDetection.updateBox(frame, boxColor, true);
            if (!boxUpdated) {
                framesSinceBox++;
                cout << "Frames since Box: " << framesSinceBox << endl;
            }
            while (waitKey(0) != 'n');
        }
        if (framesSinceBox > 10) {
            boxFound = false;
            framesSinceBox = 0;
        }
        
        t = clock() - t;
        printf ("It took me %ld clicks (%f seconds) to find boxes.\n",t,((float)t)/CLOCKS_PER_SEC);
        printf ("Box found at (%d,%d) with size (%d,%d) and with angle %f.\n",
                libBoxDetection.box.x,libBoxDetection.box.y,libBoxDetection.box.w,
                libBoxDetection.box.h,libBoxDetection.box.angle * 180 / CV_PI);

        // while (waitKey(0) != 'n');
    }

    while (waitKey(0) != 'q');
    return 0;
}