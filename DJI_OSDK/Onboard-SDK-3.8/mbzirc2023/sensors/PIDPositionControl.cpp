#include "PIDPositionControl.hpp"
#include "GeneralCommands.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;
using namespace std;

pose_t globalPos;

/****************************************************
 *
 *   Initlialize the controller
 *   Pointer to vehicle needed to get attitude and
 *   send control commands
 *
 *****************************************************/
bool PIDPositionControl::init_controller(Vehicle *v)
{
    vehicle = v;

    // Sets the broadcast frequencies from the M100 to the
    // needed values for the controllers frequency
    uint8_t broadcast_freq[16];
    vehicle->broadcast->setVersionDefaults(broadcast_freq);

    broadcast_freq[0] = DataBroadcast::FREQ_0HZ;   // ??
    broadcast_freq[1] = DataBroadcast::FREQ_50HZ;  // attitude
    broadcast_freq[2] = DataBroadcast::FREQ_50HZ;  // acceleration
    broadcast_freq[3] = DataBroadcast::FREQ_0HZ;   // linear velocity
    broadcast_freq[4] = DataBroadcast::FREQ_50HZ;  // gyro
    broadcast_freq[5] = DataBroadcast::FREQ_50HZ;  // gps location
    broadcast_freq[6] = DataBroadcast::FREQ_0HZ;   // magnetometer
    broadcast_freq[7] = DataBroadcast::FREQ_0HZ;   // remote controller data
    broadcast_freq[8] = DataBroadcast::FREQ_0HZ;   // Gimbal
    broadcast_freq[9] = DataBroadcast::FREQ_0HZ;   // flightstatus
    broadcast_freq[10] = DataBroadcast::FREQ_10HZ; // battery
    broadcast_freq[11] = DataBroadcast::FREQ_0HZ;  // control device
    vehicle->broadcast->setBroadcastFreq(broadcast_freq);

    // Reset state vector
    for (int i = 0; i < num_states; i++)
    {
        state[i] = 0.0f;
        last_state[i] = 0.0f;
    }

    // Reset innovation
    for (int i = 0; i < num_meas; i++)
    {
        inno[i] = 0.0f;
    };

    // Reset target pose
    pose_target.x = 0.0f;
    pose_target.y = 0.0f;
    pose_target.z = 0.0f;
    pose_target.th = 0.0f;

    // System matrix
    F[0][0] = 0.8878;
    F[0][1] = 0.0f;
    F[0][2] = 0.0f;
    F[1][0] = 0.0314;
    F[1][1] = 1.0f;
    F[1][2] = 0.0f;
    F[2][0] = 0.0005;
    F[2][1] = 0.0333;
    F[2][2] = 1.0f;

    F[3][3] = 0.8878;
    F[3][4] = 0.0f;
    F[3][5] = 0.0f;
    F[4][3] = 0.0314;
    F[4][4] = 1.0f;
    F[4][5] = 0.0f;
    F[5][3] = 0.0005;
    F[5][4] = 0.0333;
    F[5][5] = 1.0f;

    F[6][6] = 1.0;
    F[6][7] = 0.0f;
    F[6][8] = 0.0f;
    F[7][6] = 0.0333;
    F[7][7] = 1.0f;
    F[7][8] = 0.0f;
    F[8][6] = 0.0;
    F[8][7] = 0.0;
    F[8][8] = 1.0f;

    // Input matrix
    G[0][0] = 1.1021;
    G[1][0] = 0.0187;
    G[2][0] = 0.0002;

    G[3][1] = 1.1021;
    G[4][1] = 0.0187;
    G[5][1] = 0.0002;

    G[6][2] = 0.0333;
    G[7][2] = 0.0006;
    G[8][3] = 0.0333;

    // Reset observer matrix values
    for (int i = 0; i < num_states; i++)
    {
        for (int j = 0; j < num_meas; j++)
        {
            L[i][j] = 0.0f;
        }
    }

    L[0][0] = 0.9403;
    L[0][1] = 0.0238;
    L[1][0] = 0.0016;
    L[1][1] = 0.8728;
    L[2][1] = 0.2267;

    L[3][2] = 0.9403;
    L[3][3] = 0.0238;
    L[4][2] = 0.0016;
    L[4][3] = 0.8728;
    L[5][3] = 0.2267;

    L[6][4] = 0.2941;
    L[7][4] = 0.1352;

    L[8][5] = 0.6180;

    // Sets the linear reference ramp speeds (in m/s, rad/s)
    global_vel_ref.x = 2;
    global_vel_ref.y = 3;
    global_vel_ref.z = 2;
    global_vel_ref.th = 45 * M_PI / 180;

    // Store the starting position / heading
    originGPS_Position = vehicle->broadcast->getGlobalPosition();
    Telemetry::Quaternion quaterion = vehicle->broadcast->getQuaternion();
    Telemetry::Vector3f attitude = toEulerAngle(&quaterion);
    origin_yaw = attitude.z;

    integral_enable = 0;
    controller_type = XY_GPS_PID;

    // Start heartbeat timer
    loopTimer.start_hb(loopFreq);

    // Start the controller
    if (!StartInternalThread())
    {
        std::cout << "Autnav control thread not started\n";
        return false;
    }

    status = NORMAL;
    return true;
}

// void PIDPositionControl::get_innovation()
// {

//     Telemetry::Quaternion quaterion = vehicle->broadcast->getQuaternion();
//     Telemetry::Vector3f attitude = toEulerAngle(&quaterion);
//     Telemetry::Vector3f acc = vehicle->broadcast->getAcceleration();
//     Telemetry::Vector3f gyro = vehicle->broadcast->getAngularRate();

//     if( globalEstimatorUsed == CAMERA ) globalPos = cameraLocalizer->get_camera_position();
//     else if( globalEstimatorUsed == GPS ) globalPos = get_GPS_position();
//     else if( globalEstimatorUsed == FLIR ) globalPos = flir->getPosition( attitude.y, attitude.x, last_state.zpos, last_state.head );
//     else globalPos.valid = 0;

//     if( globalEstimatorUsed == FLIR && globalPos.num_pose > last_num_pos )
//     {
//         last_num_pos = globalPos.num_pose;
//     }
//     else if( globalEstimatorUsed == FLIR )
//     {
//         globalPos.valid = 0;
//     }

//     acc = rotation_pixhawk_to_world(acc.x, acc.y, acc.z, last_state.head);
//     // Innovation using acceleration measurement
//     inno[0] = acc.x - last_state.xacc;
//     inno[2] = acc.y - last_state.yacc;

//     // if( simulation && globalEstimatorUsed != GPS ) globalPos.z = get_GPS_position().z;
//     if( simulation ) globalPos.z = get_GPS_position().z;
//     else if( !simulation ){
//         globalPos.z = rangefinder->getHeightRP(attitude.y, attitude.x);
//         // globalPos.z = rangefinder->getHeight();
//         // std::cout << "rf = " << globalPos.z << std::endl;
//     }
// //     if( use_gps_height ) globalPos.z = get_GPS_position().z;

//     if( globalPos.z > 0.05 ){
//         inno[4] = (globalPos.z - last_state.zpos);
//     }
//     else{
//         inno[4] = 0;
//     }

//     if( globalPos.valid ) {
//         inno[1] = globalPos.x - last_state.xpos;
//         inno[3] = globalPos.y - last_state.ypos;
//         if( cameraLocalizer->get_localizer_type() == RED_BOX  ||  cameraLocalizer->get_localizer_type() == BALL || globalEstimatorUsed == FLIR ) inno[5] = 0;
//         else inno[5] = globalPos.th - last_state.head;
//     }
//     else {
//         inno[1] = 0;
//         inno[3] = 0;
//         inno[5] = 0;
//     }

// }

/**********************************************
 *
 *   Calculates the innovation values needed for
 *   the observer
 *
 ***********************************************/
void PIDPositionControl::get_innovation()
{

    // Get accelerometer and gyroscope data
    Telemetry::Quaternion quaterion = vehicle->broadcast->getQuaternion();
    Telemetry::Vector3f attitude = toEulerAngle(&quaterion);
    Telemetry::Vector3f acc = vehicle->broadcast->getAcceleration();
    Telemetry::Vector3f gyro = vehicle->broadcast->getAngularRate();

    // Geg GPS or camera position
    if (globalEstimatorUsed == CAMERA)
    {
        globalPos = cameraLocalizer->get_camera_position();
        cout << "globalPos pose:\t" << globalPos.x << "\t\t y:" << globalPos.y << "\t h: gps="
             << " \t th:" << globalPos.th << endl;
    }
    else if (globalEstimatorUsed == GPS)
        globalPos = get_GPS_position();
    // else if( globalEstimatorUsed == FLIR ) globalPos = flir->getPosition( attitude.y, attitude.x, last_state.zpos, last_state.head );
    else
        globalPos.valid = 0;

    // Old stuff from when flir was used directly. not really tested enough
    if (globalEstimatorUsed == FLIR && globalPos.num_pose > last_num_pos)
    {
        last_num_pos = globalPos.num_pose;
    }
    else if (globalEstimatorUsed == FLIR)
    {
        globalPos.valid = 0;
    }

    // Rotate accelerometer and calculate innovation
    acc = rotation_pixhawk_to_world(acc.x, acc.y, acc.z, last_state.head);
    // Innovation using acceleration measurement
    inno[0] = acc.x - last_state.xacc;
    inno[2] = acc.y - last_state.yacc;

    // Get correct height depending on simulation or not
    if (simulation)
        globalPos.z = get_GPS_position().z;
    else if (!simulation)
        globalPos.z = rangefinder->getHeightRP(attitude.y, attitude.x);

    // Short rangefinder
    if (rangefinder->get_address() == 0x30)
    {
        // Check if values are valid, if not use barometer
        if (globalPos.z > 0.1 && globalPos.z < 4.8)
        {
            inno[4] = (globalPos.z - last_state.zpos);
        }
        else
        {
            inno[4] = (get_GPS_position().z - last_state.zpos) * 0;
        }
    }
    // Long rangefdienr
    else if (rangefinder->get_address() == 0x31)
    {
        // Check if values are valid, if not use barometer
        if (globalPos.z > 0.1 && globalPos.z < 18.0)
        {
            inno[4] = (globalPos.z - last_state.zpos);
        }
        else
        {
            inno[4] = (get_GPS_position().z - last_state.zpos);
        }
    }

    // Camera and GPS innovation calculation
    if (globalEstimatorUsed != LIDAR_FLIR)
    {
        if (globalPos.valid)
        {

            inno[1] = globalPos.x - last_state.xpos;
            inno[3] = globalPos.y - last_state.ypos;
            if (cameraLocalizer->get_localizer_type() == RED_BOX || cameraLocalizer->get_localizer_type() == BALL || globalEstimatorUsed == FLIR)
                inno[5] = 0;
            else
                inno[5] = globalPos.th - last_state.head;
        }
        // If no new valid measurement is available set innovation to 0 so it will not affect the state update
        else
        {
            inno[1] = 0;
            inno[3] = 0;
            inno[5] = 0;
        }
    }
    // Flir and lidar based innovation update. Should be tested quite a lot before trusted
    else
    {
        // std::cout << "Getting flir lidar inno\n";
        // std::cout << "Getting lirad\n";
        pose_t laser_data = laser->getPosition(attitude.y);
        if (laser_data.valid && old_laser_pose < laser_data.num_pose)
        {
            // std::cout << "Valid \n";
            inno[1] = laser_data.x - last_state.xpos;
            inno[5] = (laser_data.th - last_state.head) - last_state.head;
            old_laser_pose = laser_data.num_pose;
        }
        else
        {
            inno[1] = 0;
            inno[5] = 0;
        }

        pose_t flir_data = flir->getYZDistances(fabs(last_state.xpos), 0, last_state.head);
        // flir_data.valid = 0;
        if (flir_data.valid && old_flir_pose < flir_data.num_pose)
        {
            // std::cout << "Flir valid\n";
            inno[3] = flir_data.y - last_state.ypos;
            old_flir_pose = flir_data.num_pose;
        }
        else
        {
            inno[3] = 0;
        }
    }
}

/***********************************************
 *
 *   The state and controller update function.
 *   Run at 30Hz
 *
 ***********************************************/
void PIDPositionControl::controller_run()
{
    // Reading from sensors
    Telemetry::Quaternion quaterion = vehicle->broadcast->getQuaternion();
    Telemetry::Vector3f attitude = toEulerAngle(&quaterion);
    Telemetry::Vector3f acc = vehicle->broadcast->getAcceleration();
    Telemetry::Vector3f gyro = vehicle->broadcast->getAngularRate();

    // Reset state values
    for (int i = 0; i < num_states; i++)
    {
        state[i] = 0.0f;
    }

    // Kalman model prediction
    for (int i = 0; i < num_states; i++)
    {
        state.xacc += F[0][i] * last_state[i];
        state.xvel += F[1][i] * last_state[i];
        state.xpos += F[2][i] * last_state[i];
        state.yacc += F[3][i] * last_state[i];
        state.yvel += F[4][i] * last_state[i];
        state.ypos += F[5][i] * last_state[i];
        state.zvel += F[6][i] * last_state[i];
        state.zpos += F[7][i] * last_state[i];
        state.head += F[8][i] * last_state[i];
    }

    // Inject input to model The attitude input is ignored as we just rely on accelerometer from innovation
    // state.xacc += G[0][0]*( attitude.x ); //*M_PI/180.0 );
    // state.xvel += G[1][0]*( attitude.x ); //*M_PI/180.0 );
    // state.xpos += G[2][0]*( attitude.x ); //*M_PI/180.0 );
    // state.yacc += G[3][1]*( attitude.y ); //*M_PI/180.0 );
    // state.yvel += G[4][1]*( attitude.y ); //*M_PI/180.0 );
    // state.ypos += G[5][1]*( attitude.y ); //*M_PI/180.0 );

    // INput z acc and yaw gyro input
    state.zvel += G[6][2] * (acc.z);
    state.zpos += G[7][2] * (acc.z);
    state.head += G[8][3] * (-gyro.z);

    state.head = wrap_angle(state.head);

    // Calculate innovation values
    get_innovation();

    // Kalman update
    for (int i = 0; i < num_meas; i++)
    {
        state.xacc += L[0][i] * inno[i];
        state.xvel += L[1][i] * inno[i];
        state.xpos += L[2][i] * inno[i];
        state.yacc += L[3][i] * inno[i];
        state.yvel += L[4][i] * inno[i];
        state.ypos += L[5][i] * inno[i];
        state.zvel += L[6][i] * inno[i];
        state.zpos += L[7][i] * inno[i];
        state.head += L[8][i] * inno[i];
    }

    state.head = wrap_angle(state.head);

    // Update last state
    last_state.xacc = state.xacc;
    last_state.xvel = state.xvel;
    last_state.xpos = state.xpos;
    last_state.yacc = state.yacc;
    last_state.yvel = state.yvel;
    last_state.ypos = state.ypos;
    last_state.zvel = state.zvel;
    last_state.zpos = state.zpos;
    last_state.head = state.head;

    // Update the cameraserver height and heading
    if (globalEstimatorUsed == CAMERA)
    {
        cameraLocalizer->height = last_state.zpos;
        cameraLocalizer->heading = last_state.head;
    }

    // if( !check_outer_bounds() && status != BOUND_REACHED )
    // {
    //     cout << "Bound reached!! Stopping\n";
    //     switch_to_GPS_localizer();
    //     stop_movement();
    //     status = BOUND_REACHED;
    // }

    // if( use_dji_position_control * 0)
    // {
    //     generate_Reference();

    //     yawratecmd = -yaw_pid.calculate(pose_target.th, state.head) * 180/M_PI;

    //     if( pose_target.th - state.head < -M_PI ) yawratecmd *= -1;
    //     else if( pose_target.th - state.head > M_PI ) yawratecmd *= -1;

    //     zvelcmd = height_pid.calculate(pose_target.z, state.zpos);

    //     vehicle->control->positionAndZVelYawRateCtrl(dji_x_cmd, dji_y_cmd, zvelcmd, yawratecmd);

    // }

    // else if( control_action ) {
    // Calculate control values
    if (control_action)
    {

        // Update linear reference target ramping towards global position
        generate_Reference();
        // if( fabs(pose_target.th - state.head) < M_PI ){
        yawratecmd = -yaw_pid.calculate(pose_target.th, state.head) * 180 / M_PI;

        if (pose_target.th - state.head < -M_PI)
            yawratecmd *= -1;
        else if (pose_target.th - state.head > M_PI)
            yawratecmd *= -1;

        zvelcmd = height_pid.calculate(pose_target.z, state.zpos);

        // if( fabs( global_pose_ref.x - state.xpos) > 2 || fabs( global_pose_ref.y - state.ypos) > 2 )
        // {
        //     if( using_PID ){
        //         cout << "Using PD for long distance" << endl;
        //         using_PID = 0;
        //         x_pd.reset_control();
        //         y_pd.reset_control();
        //         using_PD = 1;
        //     }
        //     xcmd = x_pd.calculate(pose_target.x, state.xpos);
        //     ycmd = y_pd.calculate(pose_target.y, state.ypos);
        // }
        // else{
        //     if( using_PD ){
        //         using_PD = 0;
        //         cout << "Using PID for short distance" << endl;
        //         x_pid.reset_control();
        //         y_pid.reset_control();
        //         using_PID = 1;
        //     }
        //     xcmd = x_pid.calculate(pose_target.x, state.xpos);
        //     ycmd = y_pid.calculate(pose_target.y, state.ypos);
        // }

        xcmd = x_pid.calculate(pose_target.x, state.xpos);

        ycmd = y_pid.calculate(pose_target.y, state.ypos);

        float rot_yaw = state.head - 0 * origin_yaw;
        // if( rot_yaw > M_PI ) rot_yaw -= 2*M_PI;
        // else if ( rot_yaw < -M_PI ) rot_yaw += 2*M_PI;

        // Rotate to DJI control frame
        Vector3f control_signal = rotation_world_to_pixhawk(xcmd, ycmd, 0, rot_yaw);
        xcmd = control_signal.x;
        ycmd = -control_signal.y;
        vehicle->control->xyPosZvelAndYawRateCtrl(xcmd, ycmd, zvelcmd, yawratecmd);
        // vehicle->control->attitudeYawRateAndVertVelCtrl(0, 0, yawratecmd, zvelcmd);
    }
}

/**************************************************
 *
 *   Switch to camera position control and reset controllers
 *   and states to camera coordinate frame
 *
 ***************************************************/
void PIDPositionControl::switch_to_camera_localizer(int type)
{

    //     control_action = 0;
    cameraLocalizer->set_localizer_type(type);
    usleep(30000);
    //     control_action = 1;
    reset_position_camera();
    set_global_estimator(CAMERA);
    yaw_pid.reset_control();
    x_pid.reset_control();
    y_pid.reset_control();
}

/**************************************************
 *
 *   Switch to GPS position control and reset controllers
 *   and states to GPS coordinate frame
 *
 ***************************************************/
void PIDPositionControl::switch_to_GPS_localizer()
{

    reset_position_GPS();
    set_global_estimator(GPS);
    yaw_pid.reset_control();
    x_pid.reset_control();
    y_pid.reset_control();
}

/**************************************************
 *
 *   OLD DONT USE BEFORE CHECKING AND TESTING
 *   Switch to camera position control and reset controllers
 *   and states to camera coordinate frame
 *
 ***************************************************/
void PIDPositionControl::switch_to_FLIR_localizer()
{

    reset_position_FLIR();
    set_global_estimator(FLIR);
    x_pid.reset_control();
    y_pid.reset_control();
}

/*********************************************
 *
 *   Sets the stauration limit on the PID controller
 *   used for xy position control
 *
 *********************************************/
void PIDPositionControl::set_PID_limits(double max, double min)
{
    x_pid.set_limits(max, min);
    y_pid.set_limits(max, min);
}

/***********************************************
 *
 *   Started setting up a logging thread but not finished
 *
 *
 ***************************************************/
// void PIDPositionControl::loggingThread()
// {

//     ofstream file;
//     char str[80];
//     char buf[16]; // need a buffer for that
//     int i=1;
//     while (true){
//         sprintf(buf,"%d",i);
//         const char* p = buf;
//         strcpy (str,"/home/local/MBZIRC2020/DJI_OSDK/Log/output");
//         strcat (str,p);
//         strcat (str,".txt");
//         std::ifstream fileExist(str);
//         if(!fileExist)
//         {
//             break;
//         }
//         i++;
//     }
//     std::freopen( str, "w", stderr );
//     file.precision(10);

//     while(controller_running)
//     {

//     }
// }

/*********************************************
 *
 *   The main control thread.
 *   Calls the controller update at the correct rate and prints values
 *   Also has old logging code but it is disabled (file is still created)
 *
 ********************************************/
void PIDPositionControl::controllerMainLoop()
{
    long beats = 0;
    struct timespec gettime_now;

    double last_time;
    double new_time;
    double freq;

    Vector3f velocity;
    Vector3f attitude;
    Quaternion atti_quad;
    float height;
    Telemetry::Vector3f acc;
    pose_t camera_pose;

    // Record start time
    clock_gettime(CLOCK_REALTIME, &gettime_now);
    long start_log_time = gettime_now.tv_sec;

    last_time = (double)(gettime_now.tv_sec - start_log_time) + (double)gettime_now.tv_nsec / (1000000000);

    // Create the logging file
    char str[80];
    char buf[16]; // need a buffer for that
    int i = 1;
    while (true)
    {
        sprintf(buf, "%d", i);
        const char *p = buf;
        strcpy(str, "/home/local/DJI_Matrice100_Ananda/Log/output");
        strcat(str, p);
        strcat(str, ".txt");
        std::ifstream fileExist(str);
        if (!fileExist)
        {
            break;
        }
        i++;
    }
    std::freopen(str, "w", stderr);
    std::cerr.precision(10);

    while (controller_running)
    {

        // Check for hearbeat
        while (!loopTimer.get_hb_flag())
            usleep(500);

        // Update state and control
        controller_run();

        // cameraLocalizer->set_last_pos( last_state.zpos, last_state.head );
        atti_quad = vehicle->broadcast->getQuaternion();
        attitude = toEulerAngle(&atti_quad);
        float h_me = rangefinder->getHeightRP(attitude.y, attitude.x);
        // cameraLocalizer->set_atti( -attitude.y, -attitude.x, last_state.head, last_state.zpos, 0 );

        // std::cout << "rf = " << rangefinder.getHeightRP(attitude.y)

        // Print values every 0.5 seconds
        if (beats % 3 == print)
        {
            if (globalEstimatorUsed == GPS)
            {
                pose_t gps_pos = get_GPS_position();
                std::cout << "GPS:   X = " << gps_pos.x << "\tY = " << gps_pos.y << "\tZ = " << gps_pos.z << "\tTh = " << gps_pos.th * 180 / M_PI << std::endl;
            }
            else if (globalEstimatorUsed == CAMERA)
            {
                // camera_pose = cameraLocalizer->get_camera_position();
                std::cout << "Camer: X = " << camera_pose.x << "\tY = " << camera_pose.y << "\tZ = " << camera_pose.z << "\tTh = " << camera_pose.th * 180 / M_PI << "\tValid = " << camera_pose.valid << std::endl;
            }

            std::cout << "State: X = " << last_state.xpos << "\tY = " << last_state.ypos << "\tZ = " << last_state.zpos << "\tTh = " << last_state.head * 180 / M_PI << std::endl;
            std::cout << "Ref  : X = " << global_pose_ref.x << "\tY = " << global_pose_ref.y << "\tZ = " << global_pose_ref.z << "\tTh = " << global_pose_ref.th * 180 / M_PI << std::endl;
            // std::cout << "Z = " << last_state.zpos << " / " << global_pose_ref.z << " / " << h_me << std::endl;
        }

        clock_gettime(CLOCK_REALTIME, &gettime_now);
        new_time = (double)(gettime_now.tv_sec - start_log_time) + (double)gettime_now.tv_nsec / (1000000000.0);
        freq = 1.0 / (new_time - last_time);

        // std::cout << "f = " << freq << std::endl;
        last_time = new_time;

        // std::cout << "Freq = " << freq << std::endl;

        last_time = new_time;

        // Disabled logging
        if (beats % 1 == -1)
        {
            velocity = vehicle->broadcast->getVelocity();
            atti_quad = vehicle->broadcast->getQuaternion();
            attitude = toEulerAngle(&atti_quad);
            height = vehicle->broadcast->getGlobalPosition().altitude;
            // camera_pose = cameraLocalizer->get_camera_position();

            acc = vehicle->broadcast->getAcceleration();

            std::cerr << new_time << " " << freq
                      << " " << attitude.x << " " << attitude.y << " " << attitude.z << " "
                      << global_pose_ref.x << " " << global_pose_ref.y << " " << global_pose_ref.z << " " << global_pose_ref.th << " "
                      << camera_pose.x << " " << camera_pose.y << " " << camera_pose.z << " " << camera_pose.th << " " << camera_pose.valid << " "
                      << height << " " << rangefinder->getHeight() << " " << rangefinder->getHeightRP(attitude.y, attitude.x) << " "
                      << last_state.xacc << " " << last_state.xvel << " " << last_state.xpos << " " << last_state.yacc << " " << last_state.yvel << " " << last_state.ypos << " " << last_state.head << " " << last_state.zvel << " " << last_state.zpos << " "
                      << acc.x << " " << acc.y << " " << acc.z << " "
                      << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " "
                      << pose_target.x << " " << pose_target.y << " " << pose_target.z << " " << pose_target.th << " "
                      << xcmd << " " << ycmd << " "
                      << globalPos.x << " " << globalPos.y << " " << globalPos.z << " " << globalPos.th << " " << globalPos.valid << " " << globalPos.num_pose << " "
                      << inno[5] << std::endl;
        }

        //         if(camera_pose.valid) std::cout << "##########\n##\n#######\n";

        beats++;
    }
}
