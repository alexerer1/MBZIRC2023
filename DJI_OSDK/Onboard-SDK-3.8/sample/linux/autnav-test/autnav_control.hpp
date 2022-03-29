#ifndef AUTNAV_CONTROL_HPP
#define AUTNAV_CONTROL_HPP

// System Includes
#include <cmath>

// DJI OSDK includes
#include "dji_status.hpp"
#include <dji_vehicle.hpp>

// Helpers
#include <dji_linux_helpers.hpp>

#define C_EARTH (double)6378137.0
#define DEG2RAD 0.01745329252

/*
struct Vector3f{
    float x;
    float y;
    float z;
};*/

struct target_pos{
    float x;
    float y;
    float th;
};

struct state_t{
    float xacc;
    float xvel;
    float xpos;
    float yacc;
    float yvel;
    float ypos;
    float head;
    float zvel;
    float zpos;                

    float &operator[](size_t idx){
        switch(idx){
            case 0: return xacc;
            case 1: return xvel;
            case 2: return xpos;
            case 3: return yacc;
            case 4: return yvel;
            case 5: return ypos;
            case 6: return head;
            case 7: return zvel;
            case 8: return zpos;                                
            default: return xacc; // Need to throw a pointer to a double
        };
    };
};

Telemetry::Vector3f rotation_pixhawk_to_world(float x, float y, float z, float yaw);
Telemetry::Vector3f rotation_world_to_pixhawk(float x, float y, float z, float yaw);
Telemetry::Vector3f rotation_imu_to_pixhawk(float x, float y, float z, float pitch, float roll);

bool autnav_init(Vehicle * vehicle);

Telemetry::Vector3f toEulerAngle(void* quaternionData);

void test_timer();

void set_reference(float x, float y, float th);

void test_filter(Vehicle * vehicle);

void localOffsetFromGpsOffset(Vehicle* vehicle, Telemetry::Vector3f& deltaNed, void* target, void* origin);

#endif
