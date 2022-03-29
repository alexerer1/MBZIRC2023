#ifndef _CAMERA_LOCALIZER_PI_HPP
#define _CAMERA_LOCALIZER_PI_HPP

#include <raspicam/raspicam_cv.h> 
#include "aruco_map.h"
//#include "aruco_pose.h"

#include "loop-heartbeat.hpp"
#include <pthread.h>


class CameraLocalizer : public HeartbeatTimer{
public:
    //CameraLocalizer() {}
    CameraLocalizer(int w = 640, int h = 480) {res_width = w; res_height = h;}
    ~CameraLocalizer() {}
	bool init_camera();
    bool stop_camera();
    
    void InternalThreadEntry();
    
    void cameraMainLoop();
    
    ar_map global_map;

private:
	
    HeartbeatTimer loopTimer;
    
    //void * cameraMainLoop(void * dnu);
    
	int res_width = 320;
	int res_height = 240;
    int loopFreq = 2;
    
    bool camera_running = false;
    
    pthread_t camera_thread;
    
	raspicam::RaspiCam_Cv piCamera;

};

#endif
