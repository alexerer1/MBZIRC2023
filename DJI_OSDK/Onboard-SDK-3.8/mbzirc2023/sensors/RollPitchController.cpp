#include "RollPitchController.hpp"
#include "GeneralCommands.hpp"


using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

/****************************************************
*
*   Initlialize the controller
*   Pointer to vehicle needed to get attitude and
*   send control commands
*
*****************************************************/
bool RollPitchController::init_controller(Vehicle * v)
{
    vehicle = v;

    // Sets the broadcast frequencies from the M100 to the
    // needed values for the controllers frequency    
    uint8_t broadcast_freq[16];
    vehicle->broadcast->setVersionDefaults(broadcast_freq);
    broadcast_freq[0] = DataBroadcast::FREQ_0HZ; // attitude
    broadcast_freq[1] = DataBroadcast::FREQ_50HZ; // attitude
    broadcast_freq[2] = DataBroadcast::FREQ_50HZ; // acceleration
    broadcast_freq[3] = DataBroadcast::FREQ_0HZ; // linear velocity
    broadcast_freq[4] = DataBroadcast::FREQ_50HZ; // gyro
    broadcast_freq[5] = DataBroadcast::FREQ_10HZ; // gps location
    broadcast_freq[6] = DataBroadcast::FREQ_0HZ; // magnetometer
    broadcast_freq[7] = DataBroadcast::FREQ_0HZ; // remote controller data
    broadcast_freq[8] = DataBroadcast::FREQ_0HZ; // Gimbal
    broadcast_freq[9] = DataBroadcast::FREQ_0HZ; // flightstatus
    broadcast_freq[10] = DataBroadcast::FREQ_0HZ; //battery
    broadcast_freq[11] = DataBroadcast::FREQ_0HZ; //control device
    vehicle->broadcast->setBroadcastFreq(broadcast_freq);
    
    // Reset state vector
    for(int i = 0; i<num_states; i++){
        state[i] = 0.0f;
        last_state[i] = 0.0f;
    }

    // Reset innovation
    for(int i = 0; i<num_meas; i++){
        inno[i] = 0.0f;
    };

    // Reset target pose
    pose_target.x = 0.0f;
    pose_target.y = 0.0f;
    pose_target.z = 0.0f;
    pose_target.th = 0.0f;
    
    // System matrix
    F[0][0]=0.8878; F[0][1]=0.0f;    F[0][2]=0.0f; 
    F[1][0]=0.0314; F[1][1]=1.0f;    F[1][2]=0.0f; 
    F[2][0]=0.0005; F[2][1]=0.0333;  F[2][2]=1.0f; 

    F[3][3]=0.8878; F[3][4]=0.0f;    F[3][5]=0.0f; 
    F[4][3]=0.0314; F[4][4]=1.0f;    F[4][5]=0.0f; 
    F[5][3]=0.0005; F[5][4]=0.0333;  F[5][5]=1.0f; 

    F[6][6]=1.0;    F[6][7]=0.0f;    F[6][8]=0.0f; 
    F[7][6]=0.0333; F[7][7]=1.0f;    F[7][8]=0.0f; 
    F[8][6]=0.0;    F[8][7]=0.0;     F[8][8]=1.0f; 

    // Input matrix
    G[0][0]= 1.1021;
    G[1][0]= 0.0187; 
    G[2][0]= 0.0002; 

    G[3][1]= 1.1021;
    G[4][1]= 0.0187; 
    G[5][1]= 0.0002; 
    
    G[6][2] = 0.0333;
    G[7][2] = 0.0006;
    G[8][3] = 0.0333;

    for( int i = 0; i < num_states; i++ ){
        for( int j = 0; j < num_meas; j++ ){
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



    // Angular position state space model
    F_angle[0][0] =   1.000000000;  F_angle[0][1] =   0.000341804;  F_angle[0][2] =   0.000032105;  F_angle[0][3] =   0.213929556;  F_angle[0][4] =   0.044787424;  F_angle[0][5] =   0.016666667;
    F_angle[1][0] =   0.000000000;  F_angle[1][1] =   0.598804260;  F_angle[1][2] =   0.000000000;  F_angle[1][3] =   0.000000000;  F_angle[1][4] =   0.000000000;  F_angle[1][5] =   0.000000000;
    F_angle[2][0] =   0.000000000;  F_angle[2][1] =   0.657030082;  F_angle[2][2] =   0.678329019;  F_angle[2][3] =  -2.196262886;  F_angle[2][4] =  -0.457142074;  F_angle[2][5] =   0.000000000;
    F_angle[3][0] =   0.000000000;  F_angle[3][1] =   0.006396081;  F_angle[3][2] =   0.013874780;  F_angle[3][3] =   0.980532817;  F_angle[3][4] =  -0.004054657;  F_angle[3][5] =   0.000000000;
    F_angle[4][0] =   0.000000000;  F_angle[4][1] =   0.000038254;  F_angle[4][2] =   0.000123063;  F_angle[4][3] =   0.016555199;  F_angle[4][4] =   0.999976776;  F_angle[4][5] =   0.000000000;
    F_angle[5][0] =   0.000000000;  F_angle[5][1] =   0.049815571;  F_angle[5][2] =   0.070735353;  F_angle[5][3] =  25.654344739;  F_angle[5][4] =   5.367792263;  F_angle[5][5] =   1.000000000;

    // Angular command input model
    G_angle[0] =  -0.000004633;
    G_angle[1] =   0.013038862;
    G_angle[2] =  -0.007478698;
    G_angle[3] =  -0.000084809;
    G_angle[4] =  -0.000000538;
    G_angle[5] =  -0.000638601;

    // 
    C_angle[0][0] =   1.000000000;  C_angle[0][1] =   0.000000000;  C_angle[0][2] =   0.000000000;  C_angle[0][3] =   0.000000000;  C_angle[0][4] =   0.000000000;  C_angle[0][5] =   0.000000000;
    C_angle[1][0] =   0.000000000;  C_angle[1][1] =   3.501415385;  C_angle[1][2] =  -8.549695958;  C_angle[1][3] = 1536.877792449; C_angle[1][4] = 322.131002765;  C_angle[1][5] =   0.000000000;

    // 
    D_angle[0] =   0.000000000;
    D_angle[1] =  -0.056898000;

    // Optimal controller values. Not working
    K_angle[0] =   4.928859989;
    K_angle[1] =  26.205614074;
    K_angle[2] =  14.927874786;
    K_angle[3] = 402.688469628;
    K_angle[4] =  82.179710868;
    K_angle[5] =   2.278305167;
    K_angle[6] =  -5.237104586;

    // Position observer for angular position model
    L_angle[0][0] =   0.990461721;  L_angle[0][1] =   0.000000006;
    L_angle[1][0] =   0.003355892;  L_angle[1][1] =   0.000008483;
    L_angle[2][0] =   0.032310605;  L_angle[2][1] =   0.000034912;
    L_angle[3][0] =   0.090789771;  L_angle[3][1] =   0.000631808;
    L_angle[4][0] =  -0.432334317;  L_angle[4][1] =   0.000090822;
    L_angle[5][0] =   1.624669740;  L_angle[5][1] =   0.000045678;


    // Reset states
    for(int i = 0; i < 6; i++) {
    	x_state[i] = 0;
    	y_state[i] = 0;
    	x_last_state[i] = 0;
    	y_last_state[i] = 0;
    }


    // Sets the linear reference ramp speeds (in m/s, rad/s)
    global_vel_ref.x = 1;
    global_vel_ref.y = 1;
    global_vel_ref.z = 1;
    global_vel_ref.th = 40*M_PI/180;
    
    // Store the starting position / heading
    originGPS_Position = vehicle->broadcast->getGlobalPosition();
    Telemetry::Quaternion quaterion = vehicle->broadcast->getQuaternion();
    Telemetry::Vector3f attitude = toEulerAngle(&quaterion);    
    origin_yaw = attitude.z;
    
    loopFreq = 60;
    Ts = 1/60.0;

    // PID controller for roll (y position)
    roll_pid_matlab.set_params(
        0.214806178818882,
        0.00446243743880562,
        0.22259440859191,
        21.0028858304873,
        0.978065112623045,
        0.00437223722492023
    );
    roll_pid_matlab.reset();

    // PID controller for roll (x position)
    pitch_pid_matlab.set_params(
        0.214806178818882,
        0.00446243743880562,
        0.22259440859191,
        21.0028858304873,
        0.978065112623045,
        0.00437223722492023
    );
    pitch_pid_matlab.reset();

    pitchcmd = 0;
    rollcmd = 0;
    
    integral_enable = 0;

    controller_type = ROLL_PITCH;

    controller_running = true;

    loopTimer.start_hb(loopFreq);
    
    if( !StartInternalThread() ){
        std::cout << "Autnav control thread not started\n";
        return false;
    }
    
    return true;
}


/**********************************************
*
*   Calculates the innovation values needed for
*   the observer
*
***********************************************/
void RollPitchController::get_innovation()
{
    
    // Get accelerometer and gyroscope data
    Telemetry::Quaternion quaterion = vehicle->broadcast->getQuaternion();
    Telemetry::Vector3f attitude = toEulerAngle(&quaterion);
    Telemetry::Vector3f acc = vehicle->broadcast->getAcceleration();
    Telemetry::Vector3f gyro = vehicle->broadcast->getAngularRate();

    // Geg GPS or camera position
    if( globalEstimatorUsed == CAMERA ) globalPos = cameraLocalizer->get_camera_position();
    else if( globalEstimatorUsed == GPS ) globalPos = get_GPS_position();
    // else if( globalEstimatorUsed == FLIR ) globalPos = flir->getPosition( attitude.y, attitude.x, last_state.zpos, last_state.head );
    else globalPos.valid = 0;
    
    // Old stuff from when flir was used directly. not really tested enough
    if( globalEstimatorUsed == FLIR && globalPos.num_pose > last_num_pos )
    {
        last_num_pos = globalPos.num_pose;
    }
    else if( globalEstimatorUsed == FLIR )
    {
        globalPos.valid = 0;
    }
    
    // Rotate accelerometer and calculate innovation
    acc = rotation_pixhawk_to_world(acc.x, acc.y, acc.z, last_state.head);
    // Innovation using acceleration measurement
    inno[0] = acc.x - last_state.xacc;
    inno[2] = acc.y - last_state.yacc;
    
    // Get correct height depending on simulation or not
    if( simulation ) globalPos.z = get_GPS_position().z; 
    else if( !simulation ) globalPos.z = rangefinder->getHeightRP(attitude.y, attitude.x);
    
    // Short rangefinder
    if( rangefinder->get_address() == 0x30 )
    {
        // Check if values are valid, if not use barometer
        if( globalPos.z > 0.1 && globalPos.z < 4.8 )
        {
            inno[4] = (globalPos.z - last_state.zpos);
        }
        else
        {
            inno[4] = (get_GPS_position().z - last_state.zpos);
        }
    }
    // Long rangefdienr
    else if( rangefinder->get_address() == 0x31 )
    {
        // Check if values are valid, if not use barometer
        if( globalPos.z > 0.1 && globalPos.z < 18.0 )
        {
            inno[4] = (globalPos.z - last_state.zpos);
        }
        else
        {
            inno[4] = (get_GPS_position().z - last_state.zpos);
        }
    }
    // if( globalPos.z > 0 ){
    //     inno[4] = (globalPos.z - last_state.zpos);
    // }
    // else{
    //     inno[4] = 0;
    // }

    // Camera and GPS innovation calculation
    if( globalEstimatorUsed != LIDAR_FLIR )
    {
        if( globalPos.valid ) {

            inno[1] = globalPos.x;// - last_state.xpos;
            inno[3] = globalPos.y;// - last_state.ypos;
            if( cameraLocalizer->get_localizer_type() == RED_BOX  ||  cameraLocalizer->get_localizer_type() == BALL || globalEstimatorUsed == FLIR ) inno[5] = 0;
            else inno[5] = globalPos.th - last_state.head;
        }
        else {

            inno[1] = 0;
            inno[3] = 0; 
            inno[5] = 0;
        }    
    }
    // Flir and lidar based innovation update. Should be tested quite a lot before trusted
    else
    {   
        // std::cout << "Getting flir lidar inno\n";
        if( trig ) {
            // std::cout << "Getting lirad\n";
            pose_t laser_data = laser->getPosition(attitude.y);
            if( laser_data.valid && old_laser_pose < laser_data.num_pose )
            {
                // std::cout << "Valid \n";
                inno[1] = laser_data.x;
                inno[5] = laser_data.th - last_state.head;
                old_laser_pose = laser_data.num_pose;
            }
            else
            {
                inno[1] = 0;
                inno[5] = 0;
            }
        }

        pose_t flir_data = flir->getYZDistances( fabs(last_state.xpos) , 0, last_state.head ) ;
        // flir_data.valid = 0;
        if( flir_data.valid && old_flir_pose < flir_data.num_pose )
        {
            // std::cout << "Flir valid\n";
            inno[3] = flir_data.y;
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
*   Run at 60Hz
*
***********************************************/
void RollPitchController::controller_run()
{

    // Reading from sensors
    Telemetry::Quaternion quaterion = vehicle->broadcast->getQuaternion();
    Telemetry::Vector3f attitude = toEulerAngle(&quaterion);
    Telemetry::Vector3f acc = vehicle->broadcast->getAcceleration();
    Telemetry::Vector3f gyro = vehicle->broadcast->getAngularRate();

    // Calculate innovation
    get_innovation();

    // Only runs every second time becuase the original model is 30Hz
    if( trig ) {

        for(int i = 0; i<num_states; i++){
            state[i] = 0.0f;
        }

        
        // Kalman prediction
        for(int i = 0; i<num_states; i++){
            state.xacc += F[0][i]*last_state[i];
            state.xvel += F[1][i]*last_state[i];
            state.xpos += F[2][i]*last_state[i];
            state.yacc += F[3][i]*last_state[i];
            state.yvel += F[4][i]*last_state[i];
            state.ypos += F[5][i]*last_state[i];
            state.zvel += F[6][i]*last_state[i];
            state.zpos += F[7][i]*last_state[i];
            state.head += F[8][i]*last_state[i];
        }

        state.zvel += G[6][2]*(acc.z); 
        state.zpos += G[7][2]*(acc.z);           
        state.head += G[8][3]*(-gyro.z);   
        
        state.head = wrap_angle(state.head);

        
        // get_innovation();
        // Kalman update
        for(int i = 0; i<num_meas; i++){
            state.xacc += L[0][i]*inno[i];
            state.xvel += L[1][i]*inno[i];
            state.xpos += L[2][i]*inno[i];
            state.yacc += L[3][i]*inno[i];
            state.yvel += L[4][i]*inno[i];
            state.ypos += L[5][i]*inno[i];
            state.zvel += L[6][i]*inno[i];
            state.zpos += L[7][i]*inno[i];
            state.head += L[8][i]*inno[i];
        }
     
        state.head = wrap_angle(state.head);

        // Update last state
        last_state.xacc = state.xacc;
        last_state.xvel = state.xvel;
        // last_state.xpos = state.xpos;
        last_state.yacc = state.yacc;
        last_state.yvel = state.yvel;
        // last_state.ypos = state.ypos;
        last_state.zvel = state.zvel;
        last_state.zpos = state.zpos;
        last_state.head = state.head;

    }
    trig = !trig;    
  
    // Update angle controller
    for( int i = 0; i < 6; i++ ){
	x_state[i] = 0;
	y_state[i] = 0;
        for( int j = 0; j < 6; j++) {
            x_state[i] += F_angle[i][j]*x_last_state[j];
            y_state[i] += F_angle[i][j]*y_last_state[j];
        }
        x_state[i] += G_angle[i]*-pitchcmd;
        y_state[i] += G_angle[i]*-rollcmd;
    }

    // Calculate output
    x_out[0] = 0;
    x_out[1] = 0;
    y_out[0] = 0;
    y_out[1] = 0;

    for(int i = 0; i < 6; i++ ){
        x_out[0] += C_angle[0][i]*x_last_state[i];
        x_out[1] += C_angle[1][i]*x_last_state[i];
        y_out[0] += C_angle[0][i]*y_last_state[i];
        y_out[1] += C_angle[1][i]*y_last_state[i];
    }

    x_out[1] += D_angle[1]*-pitchcmd;
    y_out[1] += D_angle[1]*-rollcmd;

    acc = rotation_pixhawk_to_world(acc.x, acc.y, acc.z, last_state.head);
    
    inno_angle_x[1] = acc.x - x_out[1];
    inno_angle_y[1] = acc.y - y_out[1];

    // Calculate the correct innovation
    if( globalEstimatorUsed == LIDAR_FLIR ) {
        if( inno[1] == 0 ){
            inno_angle_x[0] = 0;
        }
        else{
            inno_angle_x[0] = inno[1] - x_out[0];
        }
        if( inno[3] == 0 ){
            inno_angle_y[0] = 0;
        }
        else{
            inno_angle_y[0] = inno[3] - y_out[0];
        }
    }
    else {
        if( inno[1] == 0 && inno[3] == 0 ){
            inno_angle_x[0] = 0;
            inno_angle_y[0] = 0;
        }
        else{
            inno_angle_x[0] = inno[1] - x_out[0];
            inno_angle_y[0] = inno[3] - y_out[0];
        }
    }
    
    // std::cout << inno[1] << "   " << inno[3] << std::endl;
    // std::cout << inno_angle_x[0] << "  " << inno_angle_x[1] << std::endl;

    // Update states based on the innovation calculated
    for( int i = 0; i < 6; i++ ){
        x_state[i] += L_angle[i][0]*inno_angle_x[0] + L_angle[i][1]*inno_angle_x[1];
        y_state[i] += L_angle[i][0]*inno_angle_y[0] + L_angle[i][1]*inno_angle_y[1];
    }

    for( int i = 0; i < 6; i++) {
	    x_last_state[i] = x_state[i];
	    y_last_state[i] = y_state[i];
    }

    last_state.xpos = x_out[0];
    last_state.ypos = y_out[0];
    
    // Update the cameraserver height and heading 
    if( globalEstimatorUsed == CAMERA ){
        cameraLocalizer->height = last_state.zpos;
        cameraLocalizer->heading = last_state.head;
    }

    // if( control_action ) {

    generate_Reference();
    // force_pose_target();
    // if( fabs(pose_target.th - state.head) < M_PI ){
    yawratecmd = -yaw_pid.calculate(pose_target.th, state.head) * 180/M_PI;

    if( pose_target.th - state.head < -M_PI ) yawratecmd *= -1;
    else if( pose_target.th - state.head > M_PI ) yawratecmd *= -1;

    zvelcmd = height_pid.calculate(pose_target.z, state.zpos);


    // Optimal control update but hasn't been able to get it working    
    // rollcmd = 0;
    // pitchcmd = 0;

    // for( int i = 0; i < 6; i++ ) {
    //     rollcmd += -K_angle[i]*y_state[i];
    //     pitchcmd += -K_angle[i]*x_state[i];
    // }

    // x_int += Ts*( pose_target.x - x_state[0] );
    // y_int += Ts*( pose_target.y - y_state[0] );

    // rollcmd += -K_angle[6]*y_int;
    // pitchcmd += -K_angle[6]*x_int;

    // rollcmd = roll_pid.calculate(pose_target.y, y_out[0]);

    // Update with and without optimal control for the running the controller
    // even when the control commands are not set
    if( integral_enable ) {
        roll_pid_matlab.use_int = 1;
        pitch_pid_matlab.use_int = 1;
    }
    else {
        roll_pid_matlab.use_int = 0;
        pitch_pid_matlab.use_int = 0;   
    }

    // Update the PID controller fro roll and pitch
    rollcmd = roll_pid_matlab.update(pose_target.y, y_out[0]);
    pitchcmd = pitch_pid_matlab.update(pose_target.x, x_out[0]);
    // pitchcmd = pitch_pid.calculate(pose_target.x, x_out[0]);

    float rot_yaw = state.head - 0*origin_yaw;
    //if( rot_yaw > M_PI ) rot_yaw -= 2*M_PI;
    //else if ( rot_yaw < -M_PI ) rot_yaw += 2*M_PI;


    // Rotate it to DJI M100 frame
    Vector3f control_signal = rotation_world_to_pixhawk(pitchcmd, rollcmd, 0, rot_yaw );
    pitchcmd = -control_signal.x;
    rollcmd = -control_signal.y;
    pitchcmd = limit_pm(pitchcmd, 35*M_PI/180);
    rollcmd = limit_pm(rollcmd, 35*M_PI/180);
    // std::cout << rollcmd*180/M_PI << "  P = " << pitchcmd*180/M_PI << std::endl;
    // vehicle->control->xyPosZvelAndYawRateCtrl(xcmd, ycmd, zvelcmd, yawratecmd);

    if( control_action ) {


        vehicle->control->attitudeYawRateAndVertVelCtrl(rollcmd*180/M_PI, pitchcmd*180/M_PI, yawratecmd, zvelcmd);

        // std::cout << "Roll = " << rollcmd*180/M_PI << "//" << attitude.y*180/M_PI << "  |   Pitch = " << pitchcmd*180/M_PI << "//" << attitude.x*180/M_PI << std::endl;
        // std::cout << rollcmd*180/M_PI << "  P = " << pitchcmd*180/M_PI << std::endl;


    }

}


/**************************************************
*
*   Switch to camera position control and reset controllers
*   and states to camera coordinate frame
*
***************************************************/
void RollPitchController::switch_to_camera_localizer(int type){
    
//     control_action = 0;
    cameraLocalizer->set_localizer_type(type);
    usleep(30000);
//     control_action = 1;
    reset_position_camera();
    set_global_estimator(CAMERA);
    yaw_pid.reset_control();
    
    
}


/**************************************************
*
*   Switch to GPS position control and reset controllers
*   and states to GPS coordinate frame
*
***************************************************/
void RollPitchController::switch_to_GPS_localizer(){
    
    reset_position_GPS();
    set_global_estimator(GPS);
    yaw_pid.reset_control();
    
    
}


/**************************************************
*
*   OLD DONT USE BEFORE CHECKING AND TESTING
*   Switch to camera position control and reset controllers
*   and states to camera coordinate frame
*
***************************************************/
void RollPitchController::switch_to_FLIR_localizer(){
    
    reset_position_FLIR();
    set_global_estimator(FLIR);
    
}


/*********************************************
*
*   The main control thread. 
*   Calls the controller update at the correct rate and prints values
*   Also has old logging code but it is disabled (file is still created)
*
********************************************/
void RollPitchController::controllerMainLoop()
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
    
    clock_gettime(CLOCK_REALTIME, &gettime_now);
    long start_log_time = gettime_now.tv_sec;
    
    last_time = (double)( gettime_now.tv_sec - start_log_time) + (double)gettime_now.tv_nsec/(1000000000); 
    
    char str[80];
	char buf[16]; // need a buffer for that
	int i=1;
	while (true){
        sprintf(buf,"%d",i);
		const char* p = buf;
		strcpy (str,"/home/local/DJI_Matrice100_Ananda/Log/output");
		strcat (str,p);
		strcat (str,".txt");
		std::ifstream fileExist(str);
		if(!fileExist)
        {
			break;
		}
		i++;
	}
    std::freopen( str, "w", stderr );
    std::cerr.precision(10);
    
    std::cout << "Controller runnign\n";

    while( controller_running ) {
        
        while( !loopTimer.get_hb_flag() ) usleep(50);
        
        // std::cout << "Controller run\n";
        controller_run();
        // std::cout << "Controller run 1\n";
//         cameraLocalizer->set_last_pos( last_state.zpos, last_state.head );
        atti_quad = vehicle->broadcast->getQuaternion();
        attitude = toEulerAngle( &atti_quad );
        // cameraLocalizer->set_atti( -attitude.y, -attitude.x, last_state.head, last_state.zpos, 0 );
    
        if( beats % (loopFreq/2) == print ){
            if(globalEstimatorUsed == GPS){
                pose_t gps_pos = get_GPS_position();
                std::cout << "GPS:   X = " << gps_pos.x << "\tY = " << gps_pos.y << "\tZ = " << gps_pos.z << "\tTh = " << gps_pos.th*180/M_PI << std::endl;
            }
            else if( globalEstimatorUsed == CAMERA ){
                camera_pose = cameraLocalizer->get_camera_position();
                std::cout << "Camer: X = " << camera_pose.x << "\tY = " << camera_pose.y << "\tZ = " << camera_pose.z << "\tTh = " << camera_pose.th*180/M_PI << "\tValid = " << camera_pose.valid << std::endl;
            }
            
            std::cout << "\n#####\n";
            std::cout << "State: X = " << x_out[0] << "\tY = " << y_out[0] << "\tZ = " << last_state.zpos << "\tTh = " << last_state.head *180/M_PI << std::endl; //"  f = " << freq << std::endl;
            std::cout << "Ref  : X = " << global_pose_ref.x << "\tY = " << global_pose_ref.y << "\tZ = " << global_pose_ref.z << "\tTh = " << global_pose_ref.th *180/M_PI << std::endl;
            std::cout << "Ref  : X = " << pose_target.x << "\tY = " << pose_target.y << "\tZ = " << pose_target.z << "\tTh = " << pose_target.th *180/M_PI << std::endl;
            std::cout << "\n#####\n";
            // std::cout << "RF = " << rangefinder->getHeightRP(attitude.y, attitude.x) << std::endl;
            
        }

        clock_gettime(CLOCK_REALTIME, &gettime_now);
        new_time = (double)( gettime_now.tv_sec - start_log_time) + (double)gettime_now.tv_nsec/(1000000000.0); 
        freq = 1.0/(new_time - last_time);
        
        // std::cout << "Freq = " << freq << std::endl;

        last_time = new_time;
        
        if( beats % 1 == -1 ) {
            velocity = vehicle->broadcast->getVelocity();

            //atti_quad = vehicle->broadcast->getQuaternion();
            //attitude = toEulerAngle( &atti_quad );
            height = vehicle->broadcast->getGlobalPosition().altitude;
            // camera_pose = cameraLocalizer->get_camera_position();
            
            acc = vehicle->broadcast->getAcceleration();
            
            std::cerr << new_time << " " << freq << " "
            << attitude.x << " " << attitude.y << " " << attitude.z << " "
            //<< global_pose_ref.x << " " << global_pose_ref.y << " " << global_pose_ref.z << " " << global_pose_ref.th << " "
            //<< camera_pose.x << " " << camera_pose.y << " " << camera_pose.z << " " << camera_pose.th << " " << camera_pose.valid << " "
            //<< height << " " << rangefinder->getHeight() << " " << rangefinder->getHeightRP(attitude.y, attitude.x) << " "
            //<< last_state.xacc << " " << last_state.xvel << " " << last_state.xpos << " " << last_state.yacc << " " << last_state.yvel << " " << last_state.ypos << " " << last_state.head << " " << last_state.zvel << " " << last_state.zpos << " "
            << last_state.xpos << " " << last_state.ypos << " "
            //<< acc.x << " " << acc.y << " " << acc.z << " "
            << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " "
            //<< pose_target.x << " " << pose_target.y << " " << pose_target.z << " " << pose_target.th << " "
            << rollcmd << " " << pitchcmd << " "
            << globalPos.x << " " << globalPos.y << " " << globalPos.z << " " << globalPos.th << " " << globalPos.valid << " " << globalPos.num_pose << " " 
            //<< inno[5]
            << std::endl;

        }
        
//         if(camera_pose.valid) std::cout << "##########\n##\n#######\n";
        
        beats++;

    }   

    std::cout << "Controller stopped\n";

}
    
