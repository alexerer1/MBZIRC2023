#ifndef ROLLPITCHCONTROLLER_HPP
#define ROLLPITCHCONTROLLER_HPP

#include "ControllerBase.hpp"
#include "type_defines.hpp"
#include "pid.h"
#include "PIDController.hpp"

class RollPitchController : public ControllerBase {

private:
    
    void controller_run();
    void controllerMainLoop();
    void get_innovation();
    


    // Angle control state space model    
    // float F_angle[6][6] = {{0.0}};
    // float G_angle[6] = {{0.0}};
    // float C_angle[2][6] = {{0.0}};
    // float D_angle[2] = {{0.0}};
    
    // float K_angle[7] = {{0.0}};
    // float L_angle[6][2] = {{0.0}};
    
    
    // float inno_angle_x[2] = {{0.0}};
    // float inno_angle_y[2] = {{0.0}};

    // float x_state[6] = {{0.0}};
    // float y_state[6] = {{0.0}};
    // float x_out[2] = {{0.0}};
    // float y_out[2] = {{0.0}};
    // float x_last_state[6] = {{0.0}};
    // float y_last_state[6] = {{0.0}};
    // float x_int = 0;
    // float y_int = 0;

    // float F[9][9] = {{0.0}};
    // float G[9][4] = {{0.0}};
    // float L[9][6] = {{0.0}};
    
    float F_angle[6][6] = {0.0};
    float G_angle[6] = {0.0};
    float C_angle[2][6] = {0.0};
    float D_angle[2] = {0.0};
    
    float K_angle[7] = {0.0};
    float L_angle[6][2] = {0.0};
    
    
    float inno_angle_x[2] = {0.0};
    float inno_angle_y[2] = {0.0};

    float x_state[6] = {0.0};
    float y_state[6] = {0.0};
    float x_out[2] = {0.0};
    float y_out[2] = {0.0};
    float x_last_state[6] = {0.0};
    float y_last_state[6] = {0.0};
    float x_int = 0;
    float y_int = 0;

    float F[9][9] = {0.0};
    float G[9][4] = {0.0};
    float L[9][6] = {0.0};

    float inno[6] = {0.0};


    float rollcmd = 0;
    float pitchcmd = 0;
    float yawratecmd= 0;
    float zvelcmd = 0;

    float xcmd, ycmd;

    int num_states = 9;
    int num_meas   = 6;
    

    // PIDController roll_pid_matlab = PIDController(0.0167, pid_param);
    PIDController roll_pid_matlab = PIDController(0.0167);
    PIDController pitch_pid_matlab = PIDController(0.0167);
    // roll_pid_matlab.param.P = 0.2;    
    
    
    PID height_pid = PID(Ts, 3, -3, 0.9, 0.0, 0.0);
    PID yaw_pid = PID(Ts, 40, -40, 0.9, 0.0, 0.0);
    
    PID roll_pid = PID(0.0167, 35, -35, 0.038, 0.0028, 0.136);
    PID pitch_pid = PID(0.0167, 35, -35, 0.9, 0.0, 0.0);

    int last_num_pos = 0;
    
    bool trig = false;

public:
    pose_t globalPos;

    int print = 0;

    bool use_gps_height = 0;
    
    bool init_controller(Vehicle * v);
    
    void switch_to_camera_localizer(int type);
    void switch_to_GPS_localizer();
    void switch_to_FLIR_localizer();
    
};

#endif
