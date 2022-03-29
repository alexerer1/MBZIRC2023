#include <iostream>
#include <time.h>       /* clock_t, clock, CLOCKS_PER_SEC */
#include <stdio.h>
#include <unistd.h>
#include "boxDetection.hpp"

RNG rng(1234);

int main(int argc, char* argv[])
{
    // boxDetection libBoxDetection;
    boxDetection libBoxDetection(true);
    String videoName = "~/data/videos/outdoor_box_test_1.avi";
    if (argc > 1){
        videoName = argv[1];
    }
    long frameStart = 0;
    if (argc > 2){
        char* tmp = argv[2];
        frameStart = strtol(tmp, NULL, 10);
    }

    cv::VideoCapture cap(videoName);

    if(!cap.isOpened()){
        cout << "Error opening video stream or file" << endl;
        return -1;
    }
    Mat frame;
    clock_t t;

    long frameNumber = 0;
    while (1) {
        cap >> frame;
        frameNumber = cap.get(CAP_PROP_POS_FRAMES);
        if (frameNumber > frameStart) {
            break;
        }
    }

    bool valid = false;
    while (1) {

        t = clock();

        frameNumber = cap.get(CAP_PROP_POS_FRAMES);
        cout << "Frame number: " << frameNumber << endl;
        
        cap >> frame;
        if (frame.empty()) {
            break;
        }
        // flip(frame, frame, 0);
        valid = libBoxDetection.findWall(frame, false);
        
        t = clock() - t;
        if (valid) {
            printf ("It took me %ld clicks (%f seconds) to find the wall.\n",t,((float)t)/CLOCKS_PER_SEC);
            printf ("Wall found at (%d,%d) with size (%d,%d) and with angle %f.\n",
                libBoxDetection.wall.x,libBoxDetection.wall.y,libBoxDetection.wall.w,
                libBoxDetection.wall.h,libBoxDetection.wall.angle * 180 / CV_PI);
            while (waitKey(0) != 'n');
        }
    }

    cout << "Video has ended, press enter to exit" << endl;
    cin.get();
    return 0;
}