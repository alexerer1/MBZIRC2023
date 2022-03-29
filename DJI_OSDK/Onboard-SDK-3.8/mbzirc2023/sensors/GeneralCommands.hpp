/*! @file flight_control_sample.hpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  Flight Control API usage in a Linux environment.
 *  Provides a number of helpful additions to core API calls,
 *  especially for position control, attitude control, takeoff,
 *  landing.
 *
 *  @Copyright (c) 2016-2017 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#ifndef DJIOSDK_GENERALCOMMADS_HPP
#define DJIOSDK_GENERALCOMMADS_HPP

// System Includes
#include <cmath>
#include <vector>
#include <bitset>

// #include <unistd.h>				//Needed for I2C port
// #include <fcntl.h>				//Needed for I2C port
// #include <sys/ioctl.h>			//Needed for I2C port
// #include <linux/i2c-dev.h>		//Needed for I2C port

// DJI OSDK includes
#include "dji_status.hpp"
#include <dji_vehicle.hpp>

// // Lidar includes
// #include "componentserver.h"
// #include <sys/socket.h>
// #include <netinet/in.h>
// #include <arpa/inet.h>
// #include "xmlio.h"

// Raspicam
// #include <raspicam/raspicam_cv.h> 

// Helpers
#include <dji_linux_helpers.hpp>

#define C_EARTH (double)6378137.0
#define DEG2RAD 0.01745329252
#define RAD2DEG 180/M_PI

/*! Is transforming the quaternions to an angle
!*/
DJI::OSDK::Telemetry::Vector3f toEulerAngle(void* quaternionData);

Telemetry::Vector3f rotation_pixhawk_to_world(float x, float y, float z, float yaw);
Telemetry::Vector3f rotation_world_to_pixhawk(float x, float y, float z, float yaw);

float wrap_angle( float angle );

float limit_pm( float val, float limit);

void localOffsetFromGpsOffset(Vehicle* vehicle, Telemetry::Vector3f& deltaNed,void* target, void* origin);


//!@note: All the default timeout parameters are for acknowledgement packets
//! from the aircraft.

/*! Monitored Takeoff
    This implementation of takeoff  with monitoring makes sure your aircraft
    actually took off and only returns when takeoff is complete.
    Use unless you want to do other stuff during takeoff - this will block
    the main thread.
!*/
// bool monitoredTakeoff(DJI::OSDK::Vehicle* vehiclePtr, int timeout = 1);

/*! Velocity control using the drone as point of reference. 
!*/
// bool setDroneVelocity(DJI::OSDK::Vehicle *vehicle, float xVelocity,
//                      float yVelocity, float zVelocity=0,
//                      float yawDesired=0);

					 
/*! Angle control. Is used  manually setting the roll/pitch values of the drone

!*/
// bool setDroneAngles(Vehicle *vehicle, float roll, float pitch, float yaw);


/*! Rotates the drone by a given amount. The function will counteract drifting should it be called in windy conditions
!*/
// bool rotateDroneBy(DJI::OSDK::Vehicle *vehicle,double yawDesired, float yawThresholdInDeg = 1.0);

/*! Monitored Landing (Blocking API call). Return status as well as ack.
    This version of takeoff makes sure your aircraft actually took off
    and only returns when takeoff is complete.

!*/
// bool monitoredLanding(DJI::OSDK::Vehicle* vehiclePtr, int timeout = 1);

// Helper Functions

/*! Very simple calculation of local NED offset between two pairs of GPS
 * coordinates.
 *
 * Accurate when distances are small.
!*/
// void localOffsetFromGpsOffset(DJI::OSDK::Vehicle*             vehicle,
//                               DJI::OSDK::Telemetry::Vector3f& deltaNed,
//                               void* target, void* origin);
							  
/*! Is transforming polar coordinates into rectangular coordinates
!*/
// std::vector< std::vector<double> >  fromPolarToRect(std::vector< std::vector<double> >  lidarData);

/*! Is used for rotating x and y coordinates with a given angle.
!*/
// Telemetry::Vector3f transformCoordinates(DJI::OSDK::Vehicle* vehicle, double angleInRad, float x, float y);

/*! Initializing the Terraranger which is comunicating using I2C protocal
!*/
// void initializeI2C(char * enabled);

/*! If connected to the Terraranger then getHeight will return the Terraranger height reading
	If not then it will return the height measurement from the integrated Matrice sensors
!*/
// float32_t getHeight(char * enabled );

/*! Given a start and endpoint the function returns the flown distance from a rotated standpoint.
!*/
// Telemetry::Vector3f flownDistance(DJI::OSDK::Vehicle* vehicle, DJI::OSDK::Telemetry::GlobalPosition position, DJI::OSDK::Telemetry::GlobalPosition origin);

/*! Used for connecting with the ULMS-server
!*/
// void xml_proca(struct xml_in *x);

/*! Used for receiving the data transmitted by the ULMS-server
!*/
// void getLidarData();

/*! Logs data into the latest log file that has been created
!*/
// void logData( DJI::OSDK::Vehicle *vehicle);

/*! Creates a log file and gives is a unike number. Maximum number of log files stores on the Raspberry is set to 100
!*/
// void createLogFile();

// ######################
// int init_cam(raspicam::RaspiCam_Cv *cap);

#endif // DJIOSDK_GENERALCOMMADS_HPP
