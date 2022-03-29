#include "autnav_control.hpp"
#include "loop-heartbeat.hpp"
#include "time.h"
#include <vector>
#include <cmath>

int num_states = 9;
int num_meas   = 6;
int num_inputs = 4;

float Ts = 0.0025f;

state_t state;
state_t last_state;

target_pos pose_target;
target_pos global_vel_ref;
target_pos global_pose_ref;


static float F[9][9] = {{0.0}};
static float G[9][4] = {{0.0}};
static float L[9][6] = {{0.0}};
static float M[3][3] = {{0.0}};
static float K[3][7] = {{0.0}};
static float Ki[2][2] = {{0.0}};
static float Kt[2] = {0.0};
static float inno[6] = {0.0};

static float ix;
static float iy;

float pitchcmd_after_limit = 0;
float rollcmd_after_limit = 0;
float yawratecmd_after_limit = 0;

int old_num_pose = 0;

char intergral_enable = 0;

uint32_t time_last_pose_update = 0;

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

Telemetry::GlobalPosition currentBroadcastGP;
Telemetry::GlobalPosition originBroadcastGP;
Telemetry::Vector3f localOffset;

Telemetry::Vector3f rotation_imu_to_pixhawk(float x, float y, float z, float pitch, float roll)
{
    Telemetry::Vector3f output_vector;

    output_vector.x = (cosf(pitch))*x   +  (sinf(pitch)*sinf(roll))*y   +  (sinf(pitch)*cosf(roll))*z;
    output_vector.y = (0.0f)*x          +  (cosf(roll))*y               +  (-sinf(roll))*z;
    output_vector.z = (-sinf(pitch))*x  +  (cosf(pitch)*sinf(roll))*y   +  (cosf(pitch)*cosf(roll))*z;

    return output_vector;
} 

Telemetry::Vector3f rotation_world_to_pixhawk(float x, float y, float z, float yaw)
{
    Telemetry::Vector3f output_vector;

    output_vector.x = (cosf(yaw))*x   +  (sinf(yaw))*y   +  (0.0f)*z;
    output_vector.y = (-sinf(yaw))*x  +  (cosf(yaw))*y   +  (0.0f)*z;
    output_vector.z = (0.0f)*x        +  (0.0f)*y        -  (1.0f)*z;

    return output_vector;
} 

Telemetry::Vector3f rotation_pixhawk_to_world(float x, float y, float z, float yaw)
{
    Telemetry::Vector3f output_vector;

    output_vector.x = (cosf(yaw))*x   +  (sinf(yaw))*y   +  (0.0f)*z;
    output_vector.y = (sinf(yaw))*x   -  (cosf(yaw))*y   +  (0.0f)*z;
    output_vector.z = (0.0f)*x        +  (0.0f)*y        -  (1.0f)*z;

    return output_vector;
} 

// newmode _init - initialise althold controller and variables
bool autnav_init(Vehicle * vehicle)
{

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
    pose_target.th = 0.0f;

    // Reset integral action
    ix = 0.0f;
    iy = 0.0f;

    // System matrix
    F[0][0]=0.9834715f; F[0][1]=0.0f;    F[0][2]=0.0f; F[0][3]=0.0f;       F[0][4]=0.0f;    F[0][5]=0.0f; F[0][6]=0.0f; F[0][7]=0.0f;    F[0][8]=0.0f;
    F[1][0]=0.0024793f; F[1][1]=1.0f;    F[1][2]=0.0f; F[1][3]=0.0f;       F[1][4]=0.0f;    F[1][5]=0.0f; F[1][6]=0.0f; F[1][7]=0.0f;    F[1][8]=0.0f;
    F[2][0]=0.0000031f; F[2][1]=0.0025f; F[2][2]=1.0f; F[2][3]=0.0f;       F[2][4]=0.0f;    F[2][5]=0.0f; F[2][6]=0.0f; F[2][7]=0.0f;    F[2][8]=0.0f;
    F[3][0]=0.0f;       F[3][1]=0.0f;    F[3][2]=0.0f; F[3][3]=0.9834715f; F[3][4]=0.0f;    F[3][5]=0.0f; F[3][6]=0.0f; F[3][7]=0.0f;    F[3][8]=0.0f;
    F[4][0]=0.0f;       F[4][1]=0.0f;    F[4][2]=0.0f; F[4][3]=0.0024793f; F[4][4]=1.0f;    F[4][5]=0.0f; F[4][6]=0.0f; F[4][7]=0.0f;    F[4][8]=0.0f;
    F[5][0]=0.0f;       F[5][1]=0.0f;    F[5][2]=0.0f; F[5][3]=0.0000031f; F[5][4]=0.0025f; F[5][5]=1.0f; F[5][6]=0.0f; F[5][7]=0.0f;    F[5][8]=0.0f;
    F[6][0]=0.0f;       F[6][1]=0.0f;    F[6][2]=0.0f; F[6][3]=0.0f;       F[6][4]=0.0f;    F[6][5]=0.0f; F[6][6]=1.0f; F[6][7]=0.0f;    F[6][8]=0.0f;
    F[7][0]=0.0f;       F[7][1]=0.0f;    F[7][2]=0.0f; F[7][3]=0.0f;       F[7][4]=0.0f;    F[7][5]=0.0f; F[7][6]=0.0f; F[7][7]=1.0f;    F[7][8]=0.0f;
    F[8][0]=0.0f;       F[8][1]=0.0f;    F[8][2]=0.0f; F[8][3]=0.0f;       F[8][4]=0.0f;    F[8][5]=0.0f; F[8][6]=0.0f; F[8][7]=0.0025f; F[8][8]=1.0f;


    // Input matrix
    G[0][0]=-0.1623103f;  G[0][1]=0.0f;         G[0][2]=0.0f;     G[0][3]=0.0f;
    G[1][0]=-0.0002035f;  G[1][1]=0.0f;         G[1][2]=0.0f;     G[1][3]=0.0f;
    G[2][0]=-0.0000002f;  G[2][1]=0.0f;         G[2][2]=0.0f;     G[2][3]=0.0f;
    G[3][0]=0.0f;         G[3][1]=-0.1623103f;  G[3][2]=0.0f;     G[3][3]=0.0f;
    G[4][0]=0.0f;         G[4][1]=-0.0002035f;  G[4][2]=0.0f;     G[4][3]=0.0f;
    G[5][0]=0.0f;         G[5][1]=-0.0000002f;  G[5][2]=0.0f;     G[5][3]=0.0f;
    G[6][0]=0.0f;         G[6][1]=0.0f;         G[6][2]=0.0025f;  G[6][3]=0.0f;
    G[7][0]=0.0f;         G[7][1]=0.0f;         G[7][2]=0.0f;     G[7][3]=0.0025f;
    G[8][0]=0.0f;         G[8][1]=0.0f;         G[8][2]=0.0f;     G[8][3]=0.0000031f;

    for( int i = 0; i < 9; i++ ){
        for( int j = 0; j < 6; j++ ){
            L[i][j] = 0.0f;
        }
    }
    
    L[0][0] = 0.2080390;
	L[0][1] = 0.0011796;
	L[1][0] = 0.0020201;
    L[1][1] = 1.1611719;
    L[2][0] = 0.0000118;
    L[2][1] = 0.1374169;

    L[3][2] = 0.2080390;
    L[3][3] = 0.0011796;
    L[4][2] = 0.0020201;
    L[4][3] = 1.1611719;
    L[5][2] = 0.0000118;
    L[5][3] = 0.1374169;
    
    L[6][4] = 0.0246895;
    L[7][5] = 0.2342877;
    
    L[8][5] = 0.1217487;
    
    M[0][0] = -0.3615456;
    M[1][1] = -0.3615456;
    M[2][2] = 1.4117158;

    
    K[0][0] = -0.0390579;
    K[0][1] = -0.3103568;
    K[0][2] = -0.3615456;

    K[1][3] = -0.0390579;
    K[1][4] = -0.3103568;
    K[1][5] = -0.3615456;

    K[2][6] = 1.4117158;

	intergral_enable = 1;
    Ki[0][0] = -0.0004458;
    Ki[1][1] = -0.0004458;
    
    Kt[0] = -10;
    Kt[1] = -10;
    
    global_vel_ref.x = 7;
    global_vel_ref.y = 7;
    global_vel_ref.th = 45 * 3.14/180.0;
    
    currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
    originBroadcastGP  = currentBroadcastGP;
    localOffsetFromGpsOffset(vehicle, localOffset,                     static_cast<void*>(&currentBroadcastGP),                            static_cast<void*>(&originBroadcastGP));
    
    return true;
}

// autnav_run - runs the autonomous navigation routine, which gets a global position from a companion computer send on
// serial4/5. The control loop runs at 400 Hz
void autnav_run(Vehicle * vehicle)
{
    
    
    if(vehicle->broadcast->gps_meas_num > 0){
        
        Telemetry::Quaternion quaterion = vehicle->broadcast->getQuaternion();
        
        Telemetry::Vector3f attitude = toEulerAngle(&quaterion);
        
        Telemetry::Vector3f acc = vehicle->broadcast->getAcceleration();
        
        Telemetry::Vector3f gyro = vehicle->broadcast->getAngularRate();

        
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
            state.head += F[6][i]*last_state[i];
            state.zvel += F[7][i]*last_state[i];
            state.zpos += F[8][i]*last_state[i];
        }

        // Inject input to model
        state.xacc += G[0][0]*attitude.y;
        state.xvel += G[1][0]*attitude.y;
        state.xpos += G[2][0]*attitude.y;
        state.yacc += G[3][1]*attitude.x;
        state.yvel += G[4][1]*attitude.x;
        state.ypos += G[5][1]*attitude.x;
        state.head += G[6][2]*(-gyro.z);   
        state.zvel += G[7][3]*(acc.z); 
        state.zpos += G[8][3]*(acc.z);           

        
        acc = rotation_pixhawk_to_world(acc.x, acc.y, acc.z, state.head);
        // Innovation using acceleration measurement
        inno[0] = acc.x - state.xacc;
        inno[2] = acc.y - state.yacc;
        
        // Innovation using global pose - only when new localization data is ready from odroid,
        // otherwise set to zero
        if (vehicle->broadcast->gps_meas_num > old_num_pose ) { // new pose from localizer
            //std::cout << "Update from global position\n";
            currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
            localOffsetFromGpsOffset(vehicle, localOffset,                     static_cast<void*>(&currentBroadcastGP),                            static_cast<void*>(&originBroadcastGP));
            inno[1] = localOffset.x  - state.xpos;
            inno[3] = -localOffset.y  - state.ypos;
            inno[4] = -attitude.z - state.head;
            old_num_pose = vehicle->broadcast->gps_meas_num;
            //time_last_pose_update = millis();
        }
        else{
            inno[1] = 0.0f;
            inno[3] = 0.0f; 
            inno[4] = 0.0f;
        }

        // Innovation of altitude
        if (1){
            inno[5] = vehicle->broadcast->getGlobalPosition().height - state.zpos;
        }
        else{
            inno[5] = 0.0f;
        }
        
        // Kalman update
        for(int i = 0; i<num_meas; i++){
            state.xacc += L[0][i]*inno[i];
            state.xvel += L[1][i]*inno[i];
            state.xpos += L[2][i]*inno[i];
            state.yacc += L[3][i]*inno[i];
            state.yvel += L[4][i]*inno[i];
            state.ypos += L[5][i]*inno[i];
            state.head += L[6][i]*inno[i];
            state.zvel += L[7][i]*inno[i];
            state.zpos += L[8][i]*inno[i];
        }
        
        // Update last state
        last_state.xacc = state.xacc;
        last_state.xvel = state.xvel;
        last_state.xpos = state.xpos;
        last_state.yacc = state.yacc;
        last_state.yvel = state.yvel;
        last_state.ypos = state.ypos;
        last_state.head = state.head;
        last_state.zvel = state.zvel;
        last_state.zpos = state.zpos;
        
        // Reference generator
        if(fabs(pose_target.x - global_pose_ref.x)<=0.01f){
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
        if(fabs(pose_target.y - global_pose_ref.y)<=0.01f){
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
        if(fabs(pose_target.th - global_pose_ref.th)<=0.001f){
            pose_target.th = global_pose_ref.th;
        }
        else{
            if(pose_target.th < global_pose_ref.th){
                pose_target.th = pose_target.th + global_vel_ref.th*Ts;
            }
            else{
                pose_target.th = pose_target.th - global_vel_ref.th*Ts;
            }
        }
        
        float pitchcmd = 0;
        float rollcmd = 0;
        float yawratecmd= 0;
        
        // Control action with anti-wind-up
        if(intergral_enable){
            pitchcmd    =   ix*Ki[0][0] - (K[0][0]*state.xacc + K[0][1]*state.xvel + K[0][2]*state.xpos);
            rollcmd     =   iy*Ki[1][1] - (K[1][3]*state.yacc + K[1][4]*state.yvel + K[1][5]*state.ypos);
            yawratecmd  =   M[2][2]*pose_target.th - (K[2][6]*state.head);
            if(1){ //if flying
                ix = ix + (pose_target.x  - state.xpos) - Kt[0]*(pitchcmd - pitchcmd_after_limit);
                iy = iy + (pose_target.y  - state.ypos) - Kt[1]*(rollcmd  - rollcmd_after_limit);
            }
        }
        else{
            pitchcmd    =   M[0][0]*pose_target.x  - (K[0][0]*state.xacc + K[0][1]*state.xvel + K[0][2]*state.xpos);
            rollcmd     =   M[1][1]*pose_target.y  - (K[1][3]*state.yacc + K[1][4]*state.yvel + K[1][5]*state.ypos);
            yawratecmd  =   M[2][2]*pose_target.th - (K[2][6]*state.head);
        }

        // Add control action to attitude controller variables and convert from rad to centidegrees: 180/pi*100 = 5730
        float target_roll, target_pitch, target_yaw_rate;
        target_pitch    = pitchcmd*180/M_PI;
        target_roll     = rollcmd*180/M_PI;
        target_yaw_rate = yawratecmd*180/M_PI;

        // Constrain control signal in centidegrees
        float pitchmax = 30;
        float rollmax = 30;
        float yawratemax = 45;
        
        if(target_pitch > pitchmax){
            target_pitch =  pitchmax;   
            pitchcmd_after_limit =  pitchmax;
        }
        else if(target_pitch < -pitchmax){
            target_pitch = -pitchmax;  
            pitchcmd_after_limit = -pitchmax;
        }
        else{ 
            pitchcmd_after_limit = pitchcmd;
        }

        if(target_roll  >  rollmax){
            target_roll   =  rollmax;    
            rollcmd_after_limit  =  rollmax;
        }
        else if(target_roll  < -rollmax){
            target_roll   = -rollmax;    
            rollcmd_after_limit  = -rollmax;
        }
        else{
            rollcmd_after_limit = rollcmd;
        }

        if(target_yaw_rate >  yawratemax){target_yaw_rate =  yawratemax;}
        if(target_yaw_rate < -yawratemax){target_yaw_rate = -yawratemax;}   

        pitchcmd   = target_pitch;
        rollcmd    = target_roll;
        yawratecmd = target_yaw_rate;
        
        Vector3f control_signal = rotation_world_to_pixhawk(target_pitch, target_roll, target_yaw_rate, state.head);
        target_pitch    = control_signal.x;
        target_roll     = control_signal.y;
        target_yaw_rate = control_signal.z;
        
        vehicle->control->attitudeYawRateAndVertVelCtrl(target_roll, target_pitch, target_yaw_rate, 0);
        
    }
}


Telemetry::Vector3f
toEulerAngle(void* quaternionData)
{
  Telemetry::Vector3f    ans;
  Telemetry::Quaternion* quaternion = (Telemetry::Quaternion*)quaternionData;

  double q2sqr = quaternion->q2 * quaternion->q2;
  double t0    = -2.0 * (q2sqr + quaternion->q3 * quaternion->q3) + 1.0;
  double t1 =
    +2.0 * (quaternion->q1 * quaternion->q2 + quaternion->q0 * quaternion->q3);
  double t2 =
    -2.0 * (quaternion->q1 * quaternion->q3 - quaternion->q0 * quaternion->q2);
  double t3 =
    +2.0 * (quaternion->q2 * quaternion->q3 + quaternion->q0 * quaternion->q1);
  double t4 = -2.0 * (quaternion->q1 * quaternion->q1 + q2sqr) + 1.0;

  t2 = (t2 > 1.0) ? 1.0 : t2;
  t2 = (t2 < -1.0) ? -1.0 : t2;

  ans.x = asin(t2);
  ans.y = atan2(t3, t4);
  ans.z = atan2(t1, t0);

  return ans;
}

void test_timer(){
    
    //HeartbeatTimer hb_Timer = new HeartbeatTimer();
    struct timespec gettime_now; 
    
    long int last_heartbeat = 0;
    long int new_heartbeat = 0;
    long int time_diff = 0;
    
    HeartbeatTimer hb_Timer;
    
    int ms = 400;
    
    hb_Timer.start_hb(ms);
    
    int beats = 0;
    
    int sum = 0;
    
    std::cout << "Target freq = " << ms << " Hz" << std::endl;
    
    clock_gettime(CLOCK_REALTIME, &gettime_now);
    last_heartbeat = gettime_now.tv_nsec;
    
    while( beats < 30 ){
        
        while( !hb_Timer.get_hb_flag() );
        
        for(int i = 0; i < 50000; i++){
            sum += i;
        }
        
        beats++;
        
        clock_gettime(CLOCK_REALTIME, &gettime_now);
        new_heartbeat = gettime_now.tv_nsec;
        
        time_diff = new_heartbeat - last_heartbeat;
        
        std::cout << "Freq = " << 1.0/(time_diff / 1000000000.0) << std::endl;
        
        
        clock_gettime(CLOCK_REALTIME, &gettime_now);
        last_heartbeat = gettime_now.tv_nsec;
        
    }
    
    hb_Timer.stop_hb();
    
}

void set_reference(float x, float y, float th){
    global_pose_ref.x = x;
    global_pose_ref.y = y;
    global_pose_ref.th = th*M_PI/180.0;
    
    
}

void test_filter(Vehicle * vehicle){
    
    //HeartbeatTimer hb_Timer = new HeartbeatTimer();
    struct timespec gettime_now; 
    
    long int last_heartbeat = 0;
    long int new_heartbeat = 0;
    long int time_diff = 0;
    
    HeartbeatTimer hb_Timer;
    
    int freq_des = 400;
    
    hb_Timer.start_hb(freq_des);
    
    int beats = 0;
    
    int sum = 0;
    uint8_t broadcast_freq[16];
    vehicle->broadcast->setVersionDefaults(broadcast_freq);
    broadcast_freq[1] = DataBroadcast::FREQ_400HZ; // attitude
    broadcast_freq[2] = DataBroadcast::FREQ_400HZ; // acceleration
    broadcast_freq[4] = DataBroadcast::FREQ_400HZ; // gyro
    broadcast_freq[5] = DataBroadcast::FREQ_10HZ;
    vehicle->broadcast->setBroadcastFreq(broadcast_freq);
    
    std::cout << "Target freq = " << freq_des << std::endl;
    
    clock_gettime(CLOCK_REALTIME, &gettime_now);
    last_heartbeat = gettime_now.tv_nsec;
    
    autnav_init(vehicle);
    
    set_reference(5,-10,-90);
    
    while( beats < 400*20 ){
        
        while( !hb_Timer.get_hb_flag() );
        //printf("Passflag = 0x%X  !!\n",vehicle->broadcast->getPassFlag());
        autnav_run(vehicle);
        
        if( beats % 100 == 0 ) {
            std::cout << "States:" << std::endl;
            std::cout << "X: " << last_state.xpos << "  |  ";
            std::cout << "Y: " << last_state.ypos << "  |  ";
            std::cout << "Z: " << last_state.zpos << "  |  ";
            std::cout << "h: " << last_state.head << std::endl << std::endl;
        }
        beats++;
        
        clock_gettime(CLOCK_REALTIME, &gettime_now);
        new_heartbeat = gettime_now.tv_nsec;
        
        time_diff = new_heartbeat - last_heartbeat;
        
        std::cout << "Freq = " << 1.0/(time_diff / 1000000000.0) << std::endl;
         
        clock_gettime(CLOCK_REALTIME, &gettime_now);
        last_heartbeat = gettime_now.tv_nsec;
        
    }
    
    hb_Timer.stop_hb();
    
}

// Helper Functions

/*! Very simple calculation of local NED offset between two pairs of GPS
/coordinates.
    Accurate when distances are small.
!*/
void
localOffsetFromGpsOffset(Vehicle* vehicle, Telemetry::Vector3f& deltaNed,
                         void* target, void* origin)
{
  Telemetry::GPSFused*       subscriptionTarget;
  Telemetry::GPSFused*       subscriptionOrigin;
  Telemetry::GlobalPosition* broadcastTarget;
  Telemetry::GlobalPosition* broadcastOrigin;
  double                     deltaLon;
  double                     deltaLat;

  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    subscriptionTarget = (Telemetry::GPSFused*)target;
    subscriptionOrigin = (Telemetry::GPSFused*)origin;
    deltaLon   = subscriptionTarget->longitude - subscriptionOrigin->longitude;
    deltaLat   = subscriptionTarget->latitude - subscriptionOrigin->latitude;
    deltaNed.x = deltaLat * C_EARTH;
    deltaNed.y = deltaLon * C_EARTH * cos(subscriptionTarget->latitude);
    deltaNed.z = subscriptionTarget->altitude - subscriptionOrigin->altitude;
  }
  else
  {
    broadcastTarget = (Telemetry::GlobalPosition*)target;
    broadcastOrigin = (Telemetry::GlobalPosition*)origin;
    deltaLon        = broadcastTarget->longitude - broadcastOrigin->longitude;
    deltaLat        = broadcastTarget->latitude - broadcastOrigin->latitude;
    deltaNed.x      = deltaLat * C_EARTH;
    deltaNed.y      = deltaLon * C_EARTH * cos(broadcastTarget->latitude);
    deltaNed.z      = broadcastTarget->altitude - broadcastOrigin->altitude;
  }
}
