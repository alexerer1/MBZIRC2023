
#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/aruco/aruco.hpp"
// Raspicam
#include <raspicam/raspicam_cv.h> 

#include "CameraLocalizer.hpp"

#include "loop-heartbeat.hpp"

#include "InternalThread.hpp"

#include <pthread.h>

#include <unistd.h>
#include "time.h"

#include "aruco_map.h"
#include "aruco_pose.h"

using namespace std;
using namespace cv;

bool CameraLocalizer::stop_camera(){
    camera_running = false;
    pthread_join(camera_thread, NULL);
    loopTimer.stop_hb();
}

bool CameraLocalizer::init_camera(){

	piCamera.set(CV_CAP_PROP_FRAME_WIDTH, res_width); // 320
	piCamera.set(CV_CAP_PROP_FRAME_HEIGHT, res_height); // 240
	// printf("Opening camera ...");
	if(!piCamera.open()) {
		// printf("cam not opened!\n");
		return 0;
	}
	
	global_map.decode_xml("maps/map1.xml");
	
	loopTimer.start_hb(loopFreq);
	camera_running = true;
    
    if( StartInternalThread() ){
        std::cout << "Starting thread\n";
        return 1;
    }
    
    //     if( pthread_create(&camera_thread, NULL, cameraMainLoop, this) ){
    //         std::cout << "Camera thread not started\n";
    //     }
    
	// printf("cam is opened!\n");
	return 1;
}

void CameraLocalizer::InternalThreadEntry()
{
    std::cout << "InternalThreadEntry\n";
    cameraMainLoop();
}

void CameraLocalizer::cameraMainLoop() {
    
    cv::Mat frame;
    //HeartbeatTimer hb_Timer = new HeartbeatTimer();
    struct timespec gettime_now; 
    
    long int last_heartbeat = 0;
    long int new_heartbeat = 0;
    long int time_diff = 0;
    
    pose_t cameraPose;
    
    clock_gettime(CLOCK_REALTIME, &gettime_now);
    last_heartbeat = gettime_now.tv_nsec;
    
    while(camera_running) {

        while( loopTimer.get_hb_flag() == 0 ) usleep(10);
        
//         clock_gettime(CLOCK_REALTIME, &gettime_now);
//         new_heartbeat = gettime_now.tv_nsec;
        piCamera.grab();
        piCamera.retrieve(frame);
        
//         clock_gettime(CLOCK_REALTIME, &gettime_now);
//         std::cout << "Grab / ret time = " << (gettime_now.tv_nsec - new_heartbeat)/1000000.0 << std::endl;
        
        //cameraPose = find_pos(&frame, &global_map, 0,0);
        cameraPose = find_pos(&frame, this, 0,0);
        
//         std::cout << "X = " << cameraPose.x << std::endl;
//         std::cout << "Y = " << cameraPose.y << std::endl;
//         std::cout << "Z = " << cameraPose.z << std::endl;
//         std::cout << "Th = " << cameraPose.th << std::endl;
//         
        cv::circle(frame,Point(160,120),1,Scalar(0,0,255),5,8,0); 
        cv::imshow("Frame", frame);
         
        if ( cv::waitKey(1) == 'q' ) camera_running = false;
        
        int sum = 0;
        
        for( int i = 0; i < 10000; i++) sum+=i;
        
        clock_gettime(CLOCK_REALTIME, &gettime_now);
        new_heartbeat = gettime_now.tv_nsec;
        
        time_diff = abs(new_heartbeat - last_heartbeat);
        
        std::cout << "Freq = " << 1.0/(time_diff / 1000000000.0) << std::endl;
        
        std::cout << std::endl;
        
        clock_gettime(CLOCK_REALTIME, &gettime_now);
        last_heartbeat = gettime_now.tv_nsec;
        
    }
    
    cv::destroyAllWindows();
    
}
