#ifndef PIDPOSITIONCONTROL_HPP
#define PIDPOSITIONCONTROL_HPP

#include "ControllerBase.hpp"
#include "type_defines.hpp"
#include "pid.h"

class PIDPositionControl : public ControllerBase
{

private:
    void controller_run();
    void controllerMainLoop();
    void get_innovation();

    float F[9][9] = {{0.0}};
    float G[9][4] = {{0.0}};
    float L[9][6] = {{0.0}};

    float inno[6] = {0.0};

    float yawratecmd = 0;
    float xcmd = 0;
    float ycmd = 0;
    float zvelcmd = 0;

    int num_states = 9;
    int num_meas = 6;

    // PID height_pid = PID(Ts, 2, -2, 2, 0.2, 0.1);
    PID height_pid = PID(Ts, 2, -2, 2, 0.5, 0.0);
    PID yaw_pid = PID(Ts, 40, -40, 2, 0.2, 0.0);

    PID x_pid = PID(Ts, 1.7, -1.7, 1.2, 0.5, 0.1);
    PID y_pid = PID(Ts, 1.7, -1.7, 1.2, 0.5, 0.1);
    // PID x_pid = PID(Ts, 1.7, -1.7, 1.2, 0.5, 0.2);
    // PID y_pid = PID(Ts, 1.7, -1.7, 1.2, 0.5, 0.2);

    // PID x_pid = PID(Ts, 1.7, -1.7, 1.5, 1, 0.04);
    // PID y_pid = PID(Ts, 1.7, -1.7, 1.5, 1, 0.04);

    // PID x_pid = PID(Ts, 1.7, -1.7, 2.5, 1.5, 0.05);
    // PID y_pid = PID(Ts, 1.7, -1.7, 2.5, 1.5, 0.05);

    PID x_pd = PID(Ts, 3, -3, 1.0, 0.3, 0.0);
    PID y_pd = PID(Ts, 3, -3, 1.0, 0.3, 0.0);

    // Ch2 box
    // PID x_pid = PID(Ts, 1.0, -1.0, 1.2, 0.5, 0.2);
    // PID y_pid = PID(Ts, 1.0, -1.0, 1.2, 0.5, 0.2);

    // Ch 1 balloon
    // PID x_pid = PID(Ts, 1.5, -1.5, 1.2, 0.5, 0.2);
    // PID y_pid = PID(Ts, 1.5, -1.5, 1.2, 0.5, 0.2);

    int using_PD = 0;
    int using_PID = 1;

    int last_num_pos = 0;

public:
    int print = 0;

    bool use_gps_height = 0;

    bool init_controller(Vehicle *v);

    void switch_to_camera_localizer(int type);
    void switch_to_GPS_localizer();
    void switch_to_FLIR_localizer();
    void set_PID_limits(double max, double min);
};

#endif
