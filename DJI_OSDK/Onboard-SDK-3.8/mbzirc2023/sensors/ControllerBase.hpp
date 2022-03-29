#ifndef CONTROLLER_BASE_HPP
#define CONTROLLER_BASE_HPP

// System Includes
#include <cmath>
#include <pthread.h>

// DJI OSDK includes
#include "dji_status.hpp"
#include <dji_vehicle.hpp>

// Helpers
#include <dji_linux_helpers.hpp>

#include "loop-heartbeat.hpp"

#include "CameraServerInterface.hpp"

#include "Rangefinder.hpp"

#include "type_defines.hpp"

#include "FlirCamera.hpp"

#include "LaserScanner.hpp"

#define LANDED 0
#define FLYING 1

enum GlobalEstimator {
    GPS,
    CAMERA,
    LASERSCANNER,
    FLIR,
    LIDAR_FLIR
};

enum PID_CONTOL_STATUS {
    NORMAL,
    BOUND_REACHED
};

enum CONTROLLER_TYPE {
    ROLL_PITCH,
    XY_GPS_PID
};

class ControllerBase : public InternalThread{

protected:
    
    HeartbeatTimer loopTimer;
    Vehicle * vehicle;
    Rangefinder * rangefinder;
    FlirCamera * flir;
    CameraServer * cameraLocalizer;
    LaserScanner * laser;

    virtual void controller_run() = 0;
    virtual void controllerMainLoop() = 0;
    // Private function
    
    void generate_Reference();
    
    void InternalThreadEntry();
    
    int loopFreq = 30;
    float Ts = 1.0/30.0;
    
    int old_gps_meas_num = 0;

    int old_laser_pose = 0;
    int old_flir_pose = 0;

    bool controller_running = false;
    bool control_action = false;
    bool integral_enable;
    
    state_t state;
    state_t last_state;

    target_pos pose_target;
    target_pos global_vel_ref;
    target_pos global_pose_ref;

    int controller_type = XY_GPS_PID;


    bool simulation = 0;
    
    int globalEstimatorUsed = GPS;
    

    float corner_1_2, corner_1_4, corner_3_2, corner_3_4;

    
public:
    
    virtual bool init_controller(Vehicle * v) = 0;
    
    // Standard useful function
    

    Telemetry::GlobalPosition originGPS_Position, position_1, position_3;
    float origin_yaw = 0;
    int use_dji_position_control = 0;
    float dji_x_cmd = 0;
    float dji_y_cmd = 0;

    unsigned int check_battery();

    void set_global_estimator(int globalEst);
    void set_reference(float x, float y, float z, float th);
    void set_desired_height(float z);
    void set_desired_yaw(float th);
    void set_desired_xy(float x, float y);
    void relative_move(float x, float y, float z, float th);
    void set_relative_reference(float x, float y, float z, float th);
    void rotate( float delta_th );
    void move_z_ref( float delta_z );
    void forward_w_rotation(float x, float th);
    void change_height( float delta_z );

    int get_bounds_from_file( const char * filename );

    int check_outer_bounds();

    void relative_move_w_th_ref(float x, float y, float z, float th);

    void stop_movement();

    void set_simulation(bool sim);
    
    void force_pose_target();
    void reset_position_camera();
    void reset_position_GPS();
    void reset_position_FLIR();
    
    float distance_to_goal();
    bool balloonFound();
    bool lookForBalloons(int timeout_s);
    bool waitForPositionReached(int timeout_s);
    
    pose_t get_position();
    target_pos get_reference();
    pose_t get_GPS_position();
    
    bool takeoff(float takeoff_height);
    bool land_copter();
    
    
    void enable_control_action(bool integral);
    void disable_control_action();
    bool stop_controller();
    
    
//     void set_camera_localizer( CameraLocalizer * cam );
    void set_camera_localizer( CameraServer * cam );
    void set_rangefinder( Rangefinder * rf );
    void set_flir( FlirCamera * fl );
    void set_laserscanner( LaserScanner * ls );
    
    int status = 0;

};

#endif
