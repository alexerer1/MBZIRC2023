
#include "CameraServer.hpp"
#include <stdio.h>
#include <iostream>
#include <loop-heartbeat.hpp>
#include <unistd.h>
#include <string.h>
#include <string>
#include <fstream>
#include <time.h>

#include "aruco_pose.hpp"
#include "marker_follow.hpp"
#include "BalloonFinder.hpp"

/*********************************************
 *
 *   Initialize and start the camera thread to default parameters
 *
 *
 *
 *
 **********************************************/
bool CameraServer::init_camera()
{

    // Open RaspiCam and set standard resolution
    std::cout << "Opening camera with resolution : " << res_width << " x " << res_height << std::endl;

    piCamera.set(cv::CAP_PROP_FRAME_WIDTH, (float)res_width);   // 320
    piCamera.set(cv::CAP_PROP_FRAME_HEIGHT, (float)res_height); // 240
    piCamera.set(cv::CAP_PROP_FPS, 30);

    // piCamera.setVerticalFlip(true);
    // piCamera.setRotation(2);

    // piCamera.set ( cv::CAP_PROP_FRAME_WIDTH,  1280 ) ;
    // piCamera.set ( cv::CAP_PROP_FRAME_HEIGHT, 960 ) ;
    // piCamera.set ( cv::CAP_PROP_BRIGHTNESS, 50 );
    // piCamera.set ( cv::CAP_PROP_CONTRAST , 50 ) ;
    // piCamera.set ( cv::CAP_PROP_SATURATION, 50 ) ;
    // piCamera.set ( cv::CAP_PROP_GAIN, 50 ) ;
    // piCamera.set ( cv::CAP_PROP_FPS,  0 ) ;

    piCamera.set(cv::CAP_PROP_FORMAT, CV_8UC3);

    if (!piCamera.open(0))
    {
        return 0;
    }

    if (piCamera.isOpened())
        std::cout << "RaspiCam_CV open\n";

    // std::cout << "Cam opened\n";
    // std::cout << "Connected to camera =" << piCamera.getId() << std::endl;
    // std::cout << piCamera.get(cv::CAP_PROP_FRAME_WIDTH) << " x " << piCamera.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;

    if (!gimbal.init())
        return 0;
    gimbal.set_angle(0);

    piCamera.grab();

    piCamera.retrieve(frame);

    std::cout << "retrieve ok\n";

    camera_running = true;

    http_stream = MJPEGWriter(STREAMPORT);
    http_stream.write(frame);
    http_stream.start();

    char name[20];

    // Start threads

    if (!(pthread_create(&socket_thread, NULL, socket_starter, this) == 0))
        return 0;

    if (!(pthread_create(&logging_thread, NULL, logging_starter, this) == 0))
        return 0;

    if (!(pthread_create(&save_thread, NULL, save_starter, this) == 0))
        return 0;

    if (!StartInternalThread())
    {
        return 0;
    }

    // Decode the standard map (marker 0 is at 0,0,0)
    global_map.decode_xml("/home/local/MBZIRC2023/CameraServer/maps/startmap.xml");

    // ball_track.init(0.075);

    pthread_mutex_lock(&pose_mutex);
    estimated_position.valid = 0;
    estimated_position.num_pose = 0;
    estimated_position.x = 0;
    estimated_position.y = 0;
    estimated_position.z = 0;
    estimated_position.th = 0;
    pthread_mutex_unlock(&pose_mutex);

    return 1;
}

/*********************************************
 *
 *
 *
 *
 **********************************************/
void CameraServer::InternalThreadEntry()
{
    // std::cout << "InternalThreadEntry\n";
    mainLoop();
}

/*********************************************
 *
 *   Stops the server and closes all threads
 *   Needed to get the video saved correctly
 *
 **********************************************/
void CameraServer::stop_server()
{
    camera_running = false;
    pthread_join(socket_thread, NULL);
    WaitForInternalThreadToExit();
}

/*********************************************
 *
 *   Check if a localizations type is valid (known)
 *   by the server
 *
 **********************************************/
bool CameraServer::check_loc_type(int type)
{
    for (int i = 0; i < LAST_LOC_TYPE; i++)
    {
        if (type == i)
            return true;
    }
    return false;
}

/*********************************************
 *
 *   Sets localization type if it is known
 *   to the camera server
 *
 **********************************************/
bool CameraServer::set_loc_type(int type)
{

    if (check_loc_type(type))
    {
        used_localizer = type;
        return true;
    }
    std::cout << "Invalid loctype requested\n";
    return false;
}

/*********************************************
 *
 *   Initiates the color calibration for of the
 *   target
 *
 **********************************************/
bool CameraServer::set_calibration(bool calibrate)
{
    return true;
}

/*********************************************
 *
 *   Returns different values
 *
 **********************************************/
float CameraServer::get_last_height() { return long_rf; }
float CameraServer::get_last_heading() { return yaw; }
uint CameraServer::get_loc_type_used() { return used_localizer; }

/*********************************************
 *
 *   The thread for saving the video. Started
 *   in the initialization. Needs to be stopped for the
 *   video to be save / released properly
 *
 **********************************************/
void *CameraServer::save_handle(void)
{
    int old_frames = 0;
cv:
    Mat frame_tmp;
    clock_t t;

    cv::VideoWriter out_video("/home/local/MBZIRC2023/CameraServer/log/videoLog/output_stream.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), (int)30 / frame_skips, cv::Size(res_width, res_height), true);

    // Sets the name of the thread to see it in htop
    if (pthread_setname_np(pthread_self(), "Save handle") != 0)
        std::cout << "namefail\n";

    while (camera_running)
    {
        // Save new frames if save_flag is valid
        if (old_frames < out_frame_count && save_flag)
        {
            // frame_tmp.release();
            // std::cout << "Waiting for saveframe pre save\n";
            pthread_mutex_lock(&save_frame_mutex);
            frame_tmp = save_frame.clone();
            pthread_mutex_unlock(&save_frame_mutex);
            // std::cout << "Done and saving\n";
            // t = clock();
            out_video.write(frame_tmp);
            imwrite("/home/local/MBZIRC2023/CameraServer/log/videoLog/lastImage.jpg", frame_tmp);
            // std::cout << "Time = " << (float)(clock() - t)/CLOCKS_PER_SEC << std::endl;
            old_frames = out_frame_count;
            save_flag = false;
        }
        usleep(500);
    }
    std::cout << "\n# # # # # # # # # # \n";
    std::cout << "Releasing video\n";
    out_video.release();
}

/*********************************************
 *
 *   Function for updating the frame that should be
 *   saved. Done so that a frame is not blocked
 *   when the frame is saved to memory
 *
 **********************************************/
void CameraServer::update_save_frame(cv::Mat frame)
{

    if (!frame.empty())
    {
        // save_frame.release();
        // std::cout << "Waiting for saveframe update\n";
        pthread_mutex_lock(&save_frame_mutex);
        save_frame = frame.clone();
        pthread_mutex_unlock(&save_frame_mutex);
        save_flag = true;
        // std::cout << "Save frame update complete\n";
    }
}

/*********************************************
 *
 *   Thread that logs the data.
 *
 *
 **********************************************/
void *CameraServer::logging_handle()
{
    // truncate("/tmp/CameraServer/camera_log.txt", 1024*1024*2);
    std::ofstream logfile("/tmp/CameraServer/camera_log.txt");
    // std::ofstream logfile("/home/local/CameraServer_Ananda/log/camera_log.txt");
    // set the number of decimal points
    logfile.precision(8);

    // Init the local vairables used for the actual save call
    float roll_log, pitch_log, yaw_log, long_rf_log, short_rf_log;
    long int frame_count_log, out_frame_count_log;
    pose_t pose_log;
    float runtime_log, fps_log, camera_angle_log;

    // Sets the name of the thread to see it in htop
    if (pthread_setname_np(pthread_self(), "Logging handle") != 0)
        std::cout << "namefail\n";

    while (camera_running)
    {
        if (logging_flag)
        {

            // Update local variables needed when saving the data
            fps_log = fps;
            runtime_log = server_run_time;
            frame_count_log = frame_count;
            out_frame_count_log = out_frame_count;

            pthread_mutex_lock(&atti_mutex);
            roll_log = roll;
            pitch_log = pitch;
            yaw_log = yaw;
            long_rf_log = long_rf;
            short_rf_log = short_rf;
            pthread_mutex_unlock(&atti_mutex);

            pthread_mutex_lock(&pose_mutex);
            pose_log = estimated_position;
            pthread_mutex_unlock(&pose_mutex);

            camera_angle_log = camera_angle;

            // Save to file
            logfile << runtime_log << " " << fps_log << " " << frame_count_log << " " << out_frame_count_log << " " << camera_angle_log << " " << roll_log << " " << pitch_log << " " << yaw_log << " " << long_rf_log << " " << short_rf_log << " " << pose_log.x << " " << pose_log.y << " " << pose_log.z << " " << pose_log.th << " " << pose_log.valid << " " << pose_log.num_pose
                    << std::endl;

            logging_flag = false;
        }

        usleep(1000);
    }

    logfile.close();
}

/*********************************************
 *
 *   Finds the time between in_time and the current
 *   time. Returned in seconds
 *
 **********************************************/
float get_delta_time(struct timespec in_time)
{

    struct timespec gettime_now;
    float delta_t = 0;

    clock_gettime(CLOCK_REALTIME, &gettime_now);

    delta_t = (float)(gettime_now.tv_sec - in_time.tv_sec) + (float)(gettime_now.tv_nsec - in_time.tv_nsec) / 1000000000.0;

    return delta_t;
}

/*********************************************
 *
 *   The image capturing and analysing loop.
 *
 *
 **********************************************/
void CameraServer::mainLoop()
{

    struct timespec start_time;
    clock_gettime(CLOCK_REALTIME, &start_time);
    RNG rng(12345);
    float old_time = 0, new_time = 0;

    // clock_t start_clock = clock();
    // clock_t old_time = start_clock;
    // clock_t new_time = start_clock;

    float loop_frequency = 0;

    pose_t cameraPose;

    bool got_box = false;
    vector<Vec4i> hierarchy;
    // Sets the name of the thread to see it in htop
    if (pthread_setname_np(pthread_self(), "Cam main loop") != 0)
        std::cout << "namefail cam main loop\n";

    while (camera_running)
    {

        // Take a picture
        if (piCamera.grab())
        {
            piCamera.retrieve(frame);
        }
        else
        {
            usleep(10000);
            continue;
        }

        frame_count++;

        // Depending on the chosen localization option do that.
        if (used_localizer == DRAW_CROSS)
        {
            cv::line(frame, Point(res_width / 2 - 100, res_height / 2), Point(res_width / 2 + 100, res_height / 2), Scalar(0, 0, 255), 3, 8, 0);
            cv::line(frame, Point(res_width / 2, res_height / 2 - 100), Point(res_width / 2, res_height / 2 + 100), Scalar(0, 0, 255), 3, 8, 0);
        }
        if (used_localizer == ARUCO || used_localizer == RED_BOX)
            cameraPose = find_pos(&frame, this, roll * 0, pitch * 0);
        else if (used_localizer == BALL)
            cameraPose = ball_track.update(&frame, roll, pitch);
        else if (used_localizer == FOLLOW_MARKER)
            cameraPose = find_marker(&frame, 1);
        else if (used_localizer == FIND_BALLOON)
        {
            cameraPose = findBalloon(frame, 1);
            // std::cout << "Finding balloon\n";
        }
        else if (used_localizer == FIND_NAVAL_BOX)
        {
            cameraPose = box_detector.find_box(&frame);
            drawContours(frame, vector<vector<Point>>(1, get<0>(box_detector.targetObject)), -1, Scalar(0, 255, 255), 2, LINE_8, hierarchy, 0);
            circle(frame, get<1>(box_detector.targetObject).center, 3, Scalar(0, 255, 255), 2);
        }

        // Updates the estimated position after the analysis
        pthread_mutex_lock(&pose_mutex);
        estimated_position.x = cameraPose.x;
        estimated_position.y = cameraPose.y;
        estimated_position.z = cameraPose.z;
        // Angle offset compensation. Should be investigated before used
        estimated_position.th = cameraPose.th; // - angle_offset;
        estimated_position.valid = cameraPose.valid;
        estimated_position.num_pose = frame_count;
        pthread_mutex_unlock(&pose_mutex);

        // Get some time and frequency data
        new_time = get_delta_time(start_time);
        fps = 1.0 / (new_time - old_time);
        server_run_time = new_time;
        old_time = new_time;

        // Draws a cross in the middle of the image
        if (draw_cross)
        {
            cv::line(frame, Point(res_width / 2 - 100, res_height / 2), Point(res_width / 2 + 100, res_height / 2), Scalar(0, 0, 255), 3, 8, 0);
            cv::line(frame, Point(res_width / 2, res_height / 2 - 100), Point(res_width / 2, res_height / 2 + 100), Scalar(0, 0, 255), 3, 8, 0);
        }

        // Saves and streams the image depending on the frequency (frame_skips) set by the user
        if ((frame_count % frame_skips) == 0)
        {

            // Stream image and save image if needed
            if (save_video)
                update_save_frame(frame);
            if (stream_image)
                http_stream.write(frame);

            out_frame_count++;
        }

        logging_flag = false;

        usleep(500);
    }

    piCamera.release();

    std::cout << "Destroying windows\n";
    cv::destroyAllWindows();

    std::cout << "Stopping stream (refresh browser if stuck)\n";
    http_stream.stop();

    std::cout << "Camera over\n";
}

/*********************************************
 *
 *
 *
 *
 **********************************************/
void *CameraServer::socket_handle(void)
{

    printf("TALKER STARTED\n");

    int server_fd, new_socket, valread;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);
    char buffer[1024] = {0};
    int numbytes, len;

    // Sets the name of the thread to see it in htop
    if (pthread_setname_np(pthread_self(), "Socket hanlde\0") != 0)
        std::cout << "namefail\n";

    // Setup some strings used for request identification
    std::string get_pose("get_pose");
    std::string set_atti("set_atti");
    std::string loc_type_str("set_loc_type");
    std::string gimbal_request("set_angle"); // 9

    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
    {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }
    // Forcefully attaching socket to the port DATASOCKET
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt));

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(DATASOCKET);

    // Forcefully attaching socket to the port 8080
    if (bind(server_fd, (struct sockaddr *)&address,
             sizeof(address)) < 0)
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    std::cout << "Bind complete, listen and connect new client\n";

    while (camera_running)
    {
        // Listen on the socket to find a new client
        if (listen(server_fd, 1) != 0)
        {
            perror("listen");
            exit(EXIT_FAILURE);
        }

        fd_set rfds;
        struct timeval tv;
        tv.tv_sec = 5;

        FD_ZERO(&rfds);
        FD_SET(server_fd, &rfds);
        int recVal = select(server_fd + 1, &rfds, NULL, NULL, &tv);

        // Check for clients
        if (recVal == -1 || recVal == 0)
            continue;

        // Client found
        if ((new_socket = accept(server_fd, (struct sockaddr *)&address,
                                 (socklen_t *)&addrlen)) < 0)
        {
            perror("accept");
            exit(EXIT_FAILURE);
        }

        std::cout << "New client accepted\n";

        char in_buf[256];
        char out_buf[256];

        // Client found, started listening for requests
        while (camera_running)
        {
            usleep(50);

            // Listen to the socket and check if the client is still there
            int err = recv(new_socket, in_buf, sizeof(in_buf), MSG_DONTWAIT);

            if (err == -1)
                continue; // No request
            if (err == 0)
                break; // Lost client
            if (err > 150)
                continue; // if the request is longer than 150 char it is invalid

            // err = recv(new_socket, in_buf, sizeof(in_buf), MSG_WAITALL);
            std::string request(in_buf, err);

            // Check what the request has been sent

            // Sends the pose
            if (0 == request.compare(get_pose))
            {
                // std::cout << "Pose requested\n";
                pthread_mutex_lock(&pose_mutex);
                len = sprintf(out_buf, "pose: %.4f %.4f %.4f %.4f %d %d end", estimated_position.x, estimated_position.y, estimated_position.z,
                              estimated_position.th, estimated_position.num_pose, estimated_position.valid);
                pthread_mutex_unlock(&pose_mutex);
                send(new_socket, out_buf, len, 0);
                printf("%s\n", out_buf);
            }
            // Set the gimbal angle if the value is within the valid range
            // Returns a confirmation back to the client
            else if (0 == request.compare(0, 9, gimbal_request))
            {
                std::cout << "Got angle command\n";
                sscanf(in_buf, "set_angle: %f end", &gimbal_angle);
                if (gimbal.set_angle(gimbal_angle))
                {
                    gimbal_angle = gimbal.get_angle();
                    std::cout << "Angle set request recieved and set " << gimbal_angle << std::endl;
                    len = sprintf(out_buf, "angle_set 1 end");
                    send(new_socket, out_buf, len, 0);
                }
                else
                {
                    len = sprintf(out_buf, "angle_set 0 end");
                    send(new_socket, out_buf, len, 0);
                }
            }
            // Set the attitude values
            else if (0 == request.compare(0, 8, set_atti))
            {
                // std::cout << "Setting attitude\n";
                pthread_mutex_lock(&atti_mutex);
                sscanf(in_buf, "set_atti: %f %f %f %f %f end", &roll, &pitch, &yaw, &long_rf, &short_rf);
                // std::cout << roll << " + " << pitch << " + " << yaw << " + " << height << " + " << raw_rf << std::endl;
                pthread_mutex_unlock(&atti_mutex);
            }
            // Sets the clients requested localization type
            // Send a confirmation to the client
            else if (0 == request.compare(0, 12, loc_type_str))
            {
                std::cout << "Setting localizer type\n";
                int loc_type_request;
                sscanf(in_buf, "set_loc_type: %d end", &loc_type_request);
                if (loc_type_request >= NONE && loc_type_request < LAST_LOC_TYPE)
                {
                    pthread_mutex_lock(&loc_type_mutex);
                    used_localizer = loc_type_request;
                    len = sprintf(out_buf, "loc_set %d end", loc_type_request);
                    pthread_mutex_unlock(&loc_type_mutex);
                    send(new_socket, out_buf, len, 0);
                }
                else
                {
                    len = sprintf(out_buf, "loc_set -1 end");
                    send(new_socket, out_buf, len, 0);
                }
            }
            // Unknown request
            else
            {
                std::cout << "Socket error >> " << request << std::endl;
            }
        }
        std::cout << "Lost client. Waiting for new\n";
    }
    close(server_fd);
}
