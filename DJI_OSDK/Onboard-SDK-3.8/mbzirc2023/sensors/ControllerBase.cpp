#include "ControllerBase.hpp"
#include "loop-heartbeat.hpp"
#include "time.h"
#include "GeneralCommands.hpp"
#include <vector>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <bits/stdc++.h>

using namespace std;
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

/************************************************
*
*   Starts the main loop
*
*************************************************/
void ControllerBase::InternalThreadEntry()
{
    controller_running = true;
    std::cout << "InternalThreadEntry\n";
    controllerMainLoop();
    
}


/************************************************
*
*   Stops the controller thread
*
*************************************************/
bool ControllerBase::stop_controller(){
    controller_running = false;
    WaitForInternalThreadToExit();
    loopTimer.stop_hb();
}


/************************************************
*
*   Used for setting the camera server pointer so 
*   data from the server can be requested
*
*************************************************/
void ControllerBase::set_camera_localizer(CameraServer* cam)
{
    cameraLocalizer = cam;
}

/************************************************
*
*   Used for setting the flir server pointer so 
*   data from the server can be requested
*
*************************************************/
void ControllerBase::set_flir( FlirCamera * fl )
{
    flir = fl;
}


/************************************************
*
*   Used for setting the rangefinder pointer so 
*   data can be read from the rangefinder
*
*************************************************/
void ControllerBase::set_rangefinder(Rangefinder* rf)
{
    rangefinder = rf;
    
}


/************************************************
*
*   Used for setting the ulmsserver pointer so 
*   data from the laserscanner can be gotten
*
*************************************************/
void ControllerBase::set_laserscanner(LaserScanner * ls)
{
    laser = ls;    
}


/************************************************
*
*   Set the simulation parameter so the rangefinder
*   is not used when getting height
*
*************************************************/
void ControllerBase::set_simulation(bool sim)
{
    simulation = sim;
}


/************************************************
*
*   Enables control action and sets the integral
*   option
*
*************************************************/
void ControllerBase::enable_control_action(bool integral){
    control_action = 1;
    integral_enable = integral;
}

/************************************************
*
*   Disables the control action and sends a stop
*   movement command
*
*************************************************/
void ControllerBase::disable_control_action(){
    control_action = 0;
    vehicle->control->xyPosZvelAndYawRateCtrl(0, 0, 0, 0);
}


/************************************************
*
*   Reads GPS positions from a file to setup
*   a GPS boundery. Only tested a bit in simulation.
*   The Boundery is a box defined by 4 points
*
*************************************************/
int ControllerBase::get_bounds_from_file( const char * filename )
{

    cout << filename << endl;
    ifstream file;
    file.open( filename );
    if( !file.is_open() ) return 0;

    Telemetry::GlobalPosition position_2, position_4;
    float p1_lat, p1_log;
    float p2_lat, p2_log;
    float p3_lat, p3_log;
    float p4_lat, p4_log;

    float64_t file_data[8];

    cout.precision(12);
    // usleep(1000000);
    string line;
    int i = 0;
    while( getline(file, line) )
    {
        // cout << line << endl;
        line.append("\n");
        float64_t fval = stod(line);
        file_data[i] = fval;
        // cout << fval << " " << file_data[i] << endl;
        i++;
    }
    // for(int i = 0; i < 8; i++ )
    // {
        // getline(file, line);
        // cout << i << " " << line << "  ";
        // file_data[i] = stof(line);
        // cout << file_data[i] << endl;
        // usleep(1000);
    // }

    file.close();

    position_1.latitude = file_data[0] * DEG2RAD;
    position_1.longitude = file_data[1] * DEG2RAD;
    position_1.altitude = 0;
    position_1.height = 0;
    position_1.health = 0;

    position_2.latitude = file_data[2] * DEG2RAD;
    position_2.longitude = file_data[3] * DEG2RAD;
    position_2.altitude = 0;
    position_2.height = 0;
    position_2.health = 0;

    position_3.latitude = file_data[4] * DEG2RAD;
    position_3.longitude = file_data[5] * DEG2RAD;
    position_3.altitude = 0;
    position_3.height = 0;
    position_3.health = 0;

    position_4.latitude = file_data[6] * DEG2RAD;
    position_4.longitude = file_data[7] * DEG2RAD;
    position_4.altitude = 0;
    position_4.height = 0;
    position_4.health = 0;

    Telemetry::Vector3f deltaPos;
    cout << position_1.latitude << " " << position_1.longitude << endl;
    cout << position_2.latitude << " " << position_2.longitude << endl;
    localOffsetFromGpsOffset( vehicle, deltaPos,
                         static_cast<void*>(&position_2), static_cast<void*>(&position_1));
    cout << deltaPos.x << " " << deltaPos.y << " " << deltaPos.z << endl;

    corner_1_2 = atan( deltaPos.x / deltaPos.y ) * RAD2DEG;

    cout << "position 1 - 2 angle = " << atan( deltaPos.x / deltaPos.y ) * RAD2DEG << endl;

    localOffsetFromGpsOffset( vehicle, deltaPos,
                         static_cast<void*>(&position_4), static_cast<void*>(&position_1));
    cout << deltaPos.x << " " << deltaPos.y << " " << deltaPos.z << endl;

    cout << "position 1 - 4 angle = " << atan( deltaPos.x / deltaPos.y ) * RAD2DEG << endl;

    corner_1_4 = atan( deltaPos.x / deltaPos.y ) * RAD2DEG;

    localOffsetFromGpsOffset( vehicle, deltaPos,
                         static_cast<void*>(&position_2), static_cast<void*>(&position_3));
    cout << deltaPos.x << " " << deltaPos.y << " " << deltaPos.z << endl;

    cout << "position 3 - 2 angle = " << atan( -deltaPos.x / deltaPos.y ) * RAD2DEG << endl;

    corner_3_2 = atan( -deltaPos.x / deltaPos.y ) * RAD2DEG;

    localOffsetFromGpsOffset( vehicle, deltaPos,
                         static_cast<void*>(&position_4), static_cast<void*>(&position_3));
    cout << deltaPos.x << " " << deltaPos.y << " " << deltaPos.z << endl;

    cout << "position 3 - 4 angle = " << atan( -deltaPos.x / deltaPos.y ) * RAD2DEG << endl;

    corner_3_4 = atan( -deltaPos.x / deltaPos.y ) * RAD2DEG;
}

/************************************************
*
*   Checks if the current position is within the
*   4 points set by the GPS boundery. It checks if the angles between the corner points and the position
*   is within the angles that the 4 outer points
*   make up
*
*************************************************/
int ControllerBase::check_outer_bounds() {
    // cout << "Checking bounds\n";
    Telemetry::GlobalPosition currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
    Telemetry::Vector3f deltaPos;
    localOffsetFromGpsOffset( vehicle, deltaPos,
                     static_cast<void*>(&currentBroadcastGP), static_cast<void*>(&position_1));
    float corner_1_drone = atan( deltaPos.x / deltaPos.y ) * RAD2DEG;
    localOffsetFromGpsOffset( vehicle, deltaPos,
                     static_cast<void*>(&currentBroadcastGP), static_cast<void*>(&position_3));
    float corner_3_drone = atan( -deltaPos.x / deltaPos.y ) * RAD2DEG;

    // cout << corner_1_2 << " < " << corner_1_drone << " < " << corner_1_4 << endl;
    // cout << corner_3_2 << " < " << corner_3_drone << " < " << corner_3_4 << endl;

    bool check_1 = (corner_1_drone < corner_1_4) && (corner_1_drone > corner_1_2);
    bool check_3 = (corner_3_drone < corner_3_4) && (corner_3_drone > corner_3_2);

    // cout << check_1 << " | " << check_3 << endl;

    if( check_1 && check_3 )
    {
            return 1;
    }
    else
    {
        return 0;
    }
}


/************************************************
*
*   This function generates the references used by the
*   controller. The generation is simply a linear
*   ramp from the current reference towards the
*   endpoint target
*
*************************************************/
void ControllerBase::generate_Reference( ) {

    float x_thresh = (global_vel_ref.x * Ts) * 1.1 ;
    float y_thresh = (global_vel_ref.y * Ts) * 1.1 ;
    float z_thresh = (global_vel_ref.z * Ts) * 1.1 ;
    float yaw_thresh = (global_vel_ref.th * Ts) * 1.1 ;
    
    // x reference
    if(fabs(pose_target.x - global_pose_ref.x) <= x_thresh){
        pose_target.x = global_pose_ref.x;
    }
    else{
        if(pose_target.x < global_pose_ref.x){
            pose_target.x = pose_target.x + global_vel_ref.x*Ts;
        }
        else{
            pose_target.x = pose_target.x - global_vel_ref.x*Ts;
        }
    }

    // y reference
    if(fabs(pose_target.y - global_pose_ref.y) <= y_thresh ){
        pose_target.y = global_pose_ref.y;
    }
    else{
        if(pose_target.y < global_pose_ref.y){
            pose_target.y = pose_target.y + global_vel_ref.y*Ts;
        }
        else{
            pose_target.y = pose_target.y - global_vel_ref.y*Ts;
        }
    }

    // z reference
    if(fabs(pose_target.z - global_pose_ref.z) <= z_thresh){
        pose_target.z = global_pose_ref.z;
    }
    else{
        if(pose_target.z < global_pose_ref.z){
            pose_target.z = pose_target.z + global_vel_ref.z*Ts;
        }
        else{
            pose_target.z = pose_target.z - global_vel_ref.z*Ts;
        }
    }

    // th reference
    if(fabs(pose_target.th - global_pose_ref.th) <= yaw_thresh){
        pose_target.th = global_pose_ref.th;
    }
    else{
        if( pose_target.th - global_pose_ref.th > M_PI){
            pose_target.th = wrap_angle(pose_target.th + global_vel_ref.th*Ts);
        }
        else if( pose_target.th - global_pose_ref.th < -M_PI){
            pose_target.th = wrap_angle(pose_target.th - global_vel_ref.th*Ts);
        }
        else if( pose_target.th < global_pose_ref.th ){
            pose_target.th = wrap_angle(pose_target.th + global_vel_ref.th*Ts);
        }
        else if( pose_target.th > global_pose_ref.th ){
            pose_target.th = wrap_angle(pose_target.th - global_vel_ref.th*Ts);
        }
    }
}


/*************************************************
*
*   Overrides the reference generation so that the
*   reference change becomes an instant step
*
**************************************************/
void ControllerBase::force_pose_target(){
    
    pose_target.x = global_pose_ref.x;
    pose_target.y = global_pose_ref.y;
    pose_target.z = global_pose_ref.z;
    pose_target.th = global_pose_ref.th;
    
}


/*************************************************
*
*   Function for performing a autonomous takeoff
*   It will fly straight up to target_height
*   Is quite aggresive so should maybe just
*   be used to get up to 1 - 4 meters up
*   Integral control is not enabled after takeoff
*   if this option matters for the controller
*   it must be set manually afterwards
*
**************************************************/
bool ControllerBase::takeoff(float target_height){
    
    std::cout << "Starting takeoff\n";
    // vehicle->control->takeoff(2);

    std::cout << "Arming motors\n";
    if( vehicle->control->armMotors(1).data ){
        std::cout << "Motors not armed\n";
        return 0;
    }

    int counter = 0;
    
    // Fly up until high enough to start position control
    while( get_GPS_position().z < 0.8 ) {

        if( controller_type == ROLL_PITCH ) vehicle->control->attitudeYawRateAndVertVelCtrl(0, 0, 0, 1);
        else vehicle->control->xyPosZvelAndYawRateCtrl(0,0,0.5,0);

        counter++;
        if(counter == 200 ) return 0;

        usleep(50000);
    }

    std::cout << "Takeoff done in " << counter/20.0 << " seconds. Go to target height: " <<target_height <<"m\n";
    set_reference(last_state.xpos, last_state.ypos, target_height, last_state.head);
    force_pose_target();
    usleep(100000);
    enable_control_action(false);
    waitForPositionReached(5);

    return 1;

}


/*************************************************
*
*   Return sthe GPS position in the coordinate system
*   with the starting location being (0,0,0,0)
*
**************************************************/
pose_t ControllerBase::get_GPS_position(){
    
    Telemetry::Vector3f localOffset;
    
    Telemetry::GlobalPosition currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
    
    localOffsetFromGpsOffset(vehicle, localOffset,
                             static_cast<void*>(&currentBroadcastGP),
                             static_cast<void*>(&originGPS_Position));
    
    Telemetry::Quaternion broadcastQ         = vehicle->broadcast->getQuaternion();
    
    float yawInRad           = toEulerAngle((static_cast<void*>(&broadcastQ))).z;
    
    pose_t pos;
    pos.x = cos(origin_yaw) * localOffset.x + sin(origin_yaw) * localOffset.y;
    pos.y = sin(origin_yaw) * localOffset.x - cos(origin_yaw) * localOffset.y;
//     pos.y = -localOffset.y;
    pos.z = localOffset.z + 0.2;
    pos.th = -(yawInRad - origin_yaw);
    pos.valid = 0;
    if( pos.th > M_PI ) pos.th -= 2*M_PI;
    else if (pos.th < -M_PI ) pos.th += 2*M_PI;

    if( old_gps_meas_num < vehicle->broadcast->gps_meas_num ){
        pos.valid = 1;
        old_gps_meas_num = vehicle->broadcast->gps_meas_num;
    }

    
    return pos;
}



/*************************************************
*
*   DONT USE THIS LAND FUNCTION USE THE DJI BUILD IN
*   LANDING FUNCTION
*   DOESN'T WORK AS INTENDED IN OFTEN FAILS TO LAND
*
**************************************************/
bool ControllerBase::land_copter()
{
    set_reference( last_state.xpos, last_state.ypos, last_state.zpos, last_state.head );
    sleep(1);
    //set_reference( last_state.xpos, last_state.ypos, 0.0, last_state.head );
    set_desired_height(0.0);
    
    std::cout << "Desending\n";
    while(1){
        usleep(1000);
        if( last_state.zpos < 0.5 ){
            std::cout << "Disabling integral controller" << std::endl;
            integral_enable = 0;
            break;
        }
    }
    
    int count = 0;
    std::cout << "Landing" << std::endl;
    while( count < 50 ){
        if( last_state.zpos < 0.35 ) count++;
        usleep(1000);
    }
    
    status = LANDED;
    
    disable_control_action();
    
    for(int i = 0; i < 15; i ++){
        vehicle->control->xyPosZvelAndYawRateCtrl(0,0,-0.5,0);
        usleep(100000);
    }
    std::cout << "Disarming motors\n";
    for(int i = 0; i < 10; i ++){
        vehicle->control->disArmMotors(1);
        usleep(10000);
    }
    
}


/*************************************************
*
*   Set the global target point
*
**************************************************/
void ControllerBase::set_reference(float x, float y, float z, float th){
    global_pose_ref.x = x;
    global_pose_ref.y = y;
    global_pose_ref.z = z;
    global_pose_ref.th = wrap_angle(th);
}


/*************************************************
*
*   Set the global target height (in meters)
*
**************************************************/
void ControllerBase::set_desired_height(float z){
    global_pose_ref.z = z;
}


/*************************************************
*
*   Set the global target heading (in radians)
*
**************************************************/
void ControllerBase::set_desired_yaw(float th){
    global_pose_ref.th = wrap_angle(th);
}


/*************************************************
*
*   Set the global target heading to the current
*   heading plus the delta_th provided the function
*
**************************************************/
void ControllerBase::rotate( float delta_th )
{
    global_pose_ref.th = wrap_angle( last_state.head + delta_th );
}

/*************************************************
*
*   Move the global target height up and down
*   according to delta_z
*   THIS IS NOT RELATIVE TO CURRENT HEIGHT
*   BUT RELATIVE TO CURRENT REFERENCE
*
**************************************************/
void ControllerBase::move_z_ref( float delta_z )
{
    global_pose_ref.z += delta_z;
}


/*************************************************
*
*   Set the global height to current height
*   plus the delta_z
*
**************************************************/
void ControllerBase::change_height( float delta_z )
{
    global_pose_ref.z = last_state.zpos + delta_z;
}


/*************************************************
*
*   Set the xy global reference in global frame
*   (relative to starting point)
*
**************************************************/
void ControllerBase::set_desired_xy(float x, float y){
    global_pose_ref.x = x;
    global_pose_ref.y = y;
}


/*************************************************
*
*   Stops movement by forcing all reference targets 
*   to the current position
*
**************************************************/
void ControllerBase::stop_movement()
{
    global_pose_ref.x = last_state.xpos;
    global_pose_ref.y = last_state.ypos;
    global_pose_ref.z = last_state.zpos;
    global_pose_ref.th = last_state.head;
    force_pose_target();
}


/*************************************************
*
*   Sets the global target reference relative to the current
*   position. 
*
**************************************************/
void ControllerBase::relative_move(float x, float y, float z, float th)
{
    float x_g, y_g, z_g, th_g;

    if( x != 0 || y != 0 ) x_g = last_state.xpos + x * cos(last_state.head) - y * sin(last_state.head);
    else x_g = global_pose_ref.x;

    if( y != 0 || x != 0 ) y_g = last_state.ypos + x * sin(last_state.head) + y * cos(last_state.head);
    else y_g = global_pose_ref.y;

    if( z != 0 ) z_g = last_state.zpos + z;
    else z_g = global_pose_ref.z;

    if( th != 0 ) th_g = last_state.head + th;
    else th_g = global_pose_ref.th;

    set_reference( x_g, y_g, z_g, th_g );
}


/*************************************************
*
*   Sets the global target reference relative to
*   the current position but the x and y targets are 
*   rotated based on the current heading target and
*   not the current like relative_move() would
*
**************************************************/
void ControllerBase::relative_move_w_th_ref(float x, float y, float z, float th)
{
    float x_g, y_g, z_g, th_g;

    if( x != 0 || y != 0 ) x_g = last_state.xpos + x * cos(global_pose_ref.th) - y * sin(global_pose_ref.th);
    else x_g = global_pose_ref.x;

    if( y != 0 || x != 0 ) y_g = last_state.ypos + x * sin(global_pose_ref.th) + y * cos(global_pose_ref.th);
    else y_g = global_pose_ref.y;

    if( z != 0 ) z_g = last_state.zpos + z;
    else z_g = global_pose_ref.z;

    if( th != 0 ) th_g = last_state.head + th;
    else th_g = global_pose_ref.th;

    set_reference( x_g, y_g, z_g, th_g );
}

/*************************************************
*
*   Move forward along the x axis rotated by th
*   Corresponds to a rotation followed by a forward 
*   movement along the new orientation
*
**************************************************/
void ControllerBase::forward_w_rotation(float x, float th)
{
    float x_g, y_g, th_g;
    
    x_g = last_state.xpos + x * cos(last_state.head + th);

    y_g = last_state.ypos + x * sin(last_state.head + th);

    th_g = last_state.head + th;

    set_reference( x_g, y_g, global_pose_ref.z, th_g );
}

/*************************************************
*
*   Set the global target reference relative to the
*   current global target point
*
**************************************************/
void ControllerBase::set_relative_reference(float x, float y, float z, float th)
{
    global_pose_ref.x += x * cos(global_pose_ref.th) - y * sin(global_pose_ref.th);
    global_pose_ref.y += x * sin(global_pose_ref.th) + y * cos(global_pose_ref.th);
    global_pose_ref.z += z;
    global_pose_ref.th = wrap_angle( global_pose_ref.th + th );
}


/*************************************************
*
*   Calculates the euler distance to the global target point
*   (xyz)
*
**************************************************/
float ControllerBase::distance_to_goal()
{
    float x_err = global_pose_ref.x - last_state.xpos;
    float y_err = global_pose_ref.y - last_state.ypos;
    float z_err = global_pose_ref.z - last_state.zpos;
    
    return sqrt(x_err*x_err + y_err*y_err + z_err*z_err);
}


/*************************************************
*
*   Tells if a camerapose is valid but not necessary
*   new
*
**************************************************/
bool ControllerBase::balloonFound()
{
    pose_t camera_pose;

    camera_pose = cameraLocalizer->get_camera_position();

    if (camera_pose.valid) {
        return true;
    } else {
        return false;
    }

}



/*************************************************
*
*   Not finished function. Looks for a valid camera
*   measurement until timeout_s is reached
*
**************************************************/
bool ControllerBase::lookForBalloons(int timeout_s)
{
    //Hvad type data modtages der fra jetson og hvordan?
    //Er det overhovedet en jetson der leder efter ballonerne?
    //Hvordan undgårs der false positives? Hvor den bare blinker at der er en ballon, men den er væk på næste frame?

    float dist_err = 0;
    float th_err = 0;
    
    float elapsed_time = 0;
    
    int count = 0;

    bool balloon_found = false;

    //float balloon_x = 0;
    //float balloon_y = 0;
    
    while( elapsed_time < timeout_s ){
        
        dist_err = distance_to_goal();
        th_err = fabs(global_pose_ref.th - last_state.head);
        
        //if( dist_err < 0.08 && th_err < 3*M_PI/180 ) count++;
        if( dist_err < 0.50 && th_err < 3*M_PI/180 ) count++;
        
        if( count > 5 ) return false;
        
        balloon_found = balloonFound();

        //Skal dette være en counter for at undgå false positives?
        //Kommer dronen til at bevæge sig for langt væk hvis det er en counter?
        //Skal den stoppe op og tjekke igen hvis den ser en ballon?
        if (balloon_found) return true;

        //Hvor hurtigt kan jetson analyserer?
        elapsed_time += 0.1;
        
        usleep(100000);
        
    }
    
    return false;
}


/*************************************************
*
*   Stall program until a position is reached plus minus
*   8 cm and the angular error is less than 3 degrees
*   Quits out of the wait if timeout_s time is reached
*   so the program want stall forever
*
**************************************************/
bool ControllerBase::waitForPositionReached(int timeout_s)
{
    float dist_err = distance_to_goal();
    float th_err = global_pose_ref.th - last_state.head;
    
    float elapsed_time = 0;
    
    int count = 0;
    
    while( elapsed_time < timeout_s ){
        
        dist_err = distance_to_goal();
        th_err = fabs(global_pose_ref.th - last_state.head);
        
        if( dist_err < 0.08 && th_err < 3*M_PI/180 ) count++;
        //if( dist_err < 0.16 && th_err < 3*M_PI/180 ) count++;
        
        if( count > 5 ) return 0;
        
        elapsed_time += 0.2;
        
        usleep(200000);
        
    }
    
    return false;
}


/*************************************************
*
*   Returns the current battery percentage
*   
**************************************************/
unsigned int ControllerBase::check_battery()
{
    Telemetry::Battery battery = vehicle->broadcast->getBatteryInfo();
    // uint32_t capacity;
    // int32_t  voltage;
    // int32_t  current;
    // uint8_t  percentage;
    cout << "Cap = " << (unsigned int) battery.capacity << endl;
    cout << "Vol = " << (int) battery.voltage << endl;
    cout << "Cur = " << (int) battery.current << endl;
    cout << "Per = " << (unsigned int) battery.percentage << endl;

    return (unsigned int) battery.percentage;
}



/*************************************************
*
*   Resets the state estimator to the camera position
*   coordinate system. Will wait until a valid camera
*   position is found
*
**************************************************/
void ControllerBase::reset_position_camera(){
    pose_t camera_pose;// = cameraLocalizer->get_camera_position();
    
    while(true) {
        camera_pose = cameraLocalizer->get_camera_position();
        if( camera_pose.valid ) break;
        usleep(50000);
    }
    
    last_state.xpos = camera_pose.x;
    last_state.xvel = 0;
    last_state.xacc = 0;
    last_state.ypos = camera_pose.y;
    last_state.yvel = 0;
    last_state.yacc = 0;
    last_state.head = camera_pose.th;
    //last_state.zvel = 0;
    //last_state.zpos = rangefinder->getHeight();
    
    set_reference(camera_pose.x, camera_pose.y, last_state.zpos, camera_pose.th);
    force_pose_target();
}


/*************************************************
*
*   Resets the state estimator to the FLIR position
*   coordinate system. Will wait until a valid camera
*   position is found
*   DONT USE
*
**************************************************/
void ControllerBase::reset_position_FLIR(){
    Telemetry::Quaternion quaterion = vehicle->broadcast->getQuaternion();
    Telemetry::Vector3f attitude = toEulerAngle(&quaterion);
    
    pose_t flir_pose;
    while(true) {
        flir_pose = flir->getPosition(attitude.y, attitude.x, last_state.zpos, last_state.head);
        if( flir_pose.valid ) break;
        usleep(100000);
    }
    
    last_state.xpos = flir_pose.x;
    last_state.xvel = 0;
    last_state.xacc = 0;
    last_state.ypos = flir_pose.y;
    last_state.yvel = 0;
    last_state.yacc = 0;
    //last_state.zvel = 0;
    //last_state.zpos = rangefinder->getHeight();
    
    set_reference(flir_pose.x, flir_pose.y, last_state.zpos, last_state.head);
    force_pose_target();
}


/*************************************************
*
*   Resets the state estimator to the GPS position
*   coordinate system. Will wait until a valid camera
*   position is found (Position is relative to start)
*
**************************************************/
void ControllerBase::reset_position_GPS(){
    pose_t gps_pose = get_GPS_position();
    
    last_state.xpos = gps_pose.x;
    last_state.xvel = 0;
    last_state.xacc = 0;
    last_state.ypos = gps_pose.y;
    last_state.yvel = 0;
    last_state.yacc = 0;
    last_state.head = gps_pose.th;
    //last_state.zvel = 0;
    //last_state.zpos = rangefinder->getHeight();
    
    set_reference(gps_pose.x, gps_pose.y, last_state.zpos, gps_pose.th);
    force_pose_target();
}


/*************************************************
*
*   Returns the current esetimated position
*
**************************************************/
pose_t ControllerBase::get_position(){
    pose_t position;
    position.x = last_state.xpos;
    position.y = last_state.ypos;
    position.z = last_state.zpos;
    position.th = last_state.head;
    return position;
}


/*************************************************
*
*   Returns the current global reference target
*
**************************************************/
target_pos ControllerBase::get_reference(){
    target_pos reference;
    reference.x = global_pose_ref.x;
    reference.y = global_pose_ref.y;
    reference.z = global_pose_ref.z;
    reference.th = global_pose_ref.th;
    return reference;
}


/*************************************************
*
*   Sets the global localization scheme to be used
*
**************************************************/
void ControllerBase::set_global_estimator(int globalEst){
    globalEstimatorUsed = globalEst;
}

