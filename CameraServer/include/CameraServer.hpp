#ifndef _CAMERA_SERVER_HPP
#define _CAMERA_SERVER_HPP

#include <pthread.h>
// #include <raspicam/raspicam_cv.h>
// #include <raspicam/raspicam.h>

// openCV libs
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/aruco.hpp>

#include "InternalThread.hpp"
#include "MJPEGWriter.h"

#include <sys/types.h>
#include <sys/socket.h>

#include "type_defines.hpp"

#include "aruco_map.hpp"

#include "BallLocalizer.hpp"
#include "navalBoxDetection.hpp"

#include "Gimbal.hpp"

//#include "findAngleClass.hpp"

#define STREAMPORT 4950
#define DATASOCKET 7740

enum CamLocType
{
    NONE,
    ARUCO,
    RED_BOX,
    BALL,
    FIND_BOX,
    UPDATE_BOX,
    FIND_WALL,
    FOLLOW_MARKER,
    J_FIND_BOX,
    FIND_BALLOON,
    DRAW_CROSS,
    FIND_NAVAL_BOX,
    LAST_LOC_TYPE

};

class CameraServer : public InternalThread
{

private:
    bool image_target_color_calibration = false;
    int res_width = 640;
    int res_height = 480;

    int frame_skips = 1;

    int used_localizer = NONE;

    BallLocalizer ball_track;
    navalBoxDetection box_detector;
    // raspicam::RaspiCam_Cv piCamera;
    cv::VideoCapture piCamera;

    cv::Mat frame;
    cv::Mat save_frame;

    MJPEGWriter http_stream;

    Gimbal gimbal;

    // BoxDetection j_boxDetector = BoxDetection(cv::Size(640,480), BoxDetection::GREEN_BOX);

    long int frame_count = 0;
    long int out_frame_count = 0;

    bool save_flag = false;

    void InternalThreadEntry();
    void mainLoop();

    pose_t estimated_position;
    float roll = 0.0;
    float pitch = 0.0;
    float yaw = 0.0;
    float long_rf = 0.0;
    float short_rf = 0.0;

    float gimbal_angle = 0;

    float camera_angle = 0.0;

    pthread_mutex_t pose_mutex = PTHREAD_MUTEX_INITIALIZER;
    pthread_mutex_t atti_mutex = PTHREAD_MUTEX_INITIALIZER;
    pthread_mutex_t loc_type_mutex = PTHREAD_MUTEX_INITIALIZER;

    pthread_mutex_t save_frame_mutex = PTHREAD_MUTEX_INITIALIZER;

private:
    // The socket server thread on localhost for communication with the server
    pthread_t socket_thread;
    void *socket_handle();
    static void *socket_starter(void *context)
    {
        return ((CameraServer *)context)->socket_handle();
    }

    // Thread for logging
    pthread_t logging_thread;
    void *logging_handle();
    bool logging_flag = false;
    static void *logging_starter(void *context)
    {
        return ((CameraServer *)context)->logging_handle();
    }

    // Thread for saving the video
    pthread_t save_thread;
    void *save_handle();
    void update_save_frame(cv::Mat frame);
    static void *save_starter(void *context)
    {
        return ((CameraServer *)context)->save_handle();
    }

    // Functions
public:
    CameraServer(int w, int h)
    {
        res_width = w;
        res_height = h;
    }
    ~CameraServer() {}

    bool init_camera();
    void stop_server();
    bool set_loc_type(int type);
    bool set_calibration(bool calibrate);
    bool check_loc_type(int type);

    // Variables
public:
    float get_last_height();
    float get_last_heading();
    uint get_loc_type_used();

    bool camera_running = false;
    bool show_image = false;
    bool stream_image = true;
    bool save_video = false;

    bool draw_cross = false;

    float angle_offset = 0;

    ar_map global_map;

    float fps = 0;
    float server_run_time = 0;

    bool draw_box = false;
    char boxColor = 'r';
};

#endif
