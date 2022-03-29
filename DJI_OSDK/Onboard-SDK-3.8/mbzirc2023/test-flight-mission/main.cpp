/*! @file flight-control/main.cpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  main for Flight Control API usage in a Linux environment.
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
//K-develope

// DJI OSDK includes
#include "dji_status.hpp"
#include <dji_vehicle.hpp>
// Helpers
#include <dji_linux_helpers.hpp>

#include <cmath>
#include <csignal>
#include <cstdio>

#include "Rangefinder.hpp"
#include "PIDPositionControl.hpp"
#include "RollPitchController.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;
using namespace std;

bool RUNNING = true;

void ctrl_c_handle(int s)
{
    std::cout << "ctrl_c_handle\n";
    RUNNING = false;
}
/*! main
 *
 */
int main(int argc, char **argv)
{

    // Initialize variables
    int functionTimeout = 1;

    // Setup OSDK.
    LinuxSetup linuxEnvironment(argc, argv);
    Vehicle *vehicle = linuxEnvironment.getVehicle();
    if (vehicle == NULL)
    {
        std::cout << "Vehicle not initialized, exiting.\n";
        return -1;
    }

    // Obtain Control Authority
    vehicle->obtainCtrlAuthority(functionTimeout);

    // RC sticky test
    Telemetry::RC rc;

    uint8_t broadcast_freq[16];
    vehicle->broadcast->setVersionDefaults(broadcast_freq);
    broadcast_freq[0] = DataBroadcast::FREQ_0HZ;   // ??
    broadcast_freq[1] = DataBroadcast::FREQ_50HZ;  // attitude
    broadcast_freq[2] = DataBroadcast::FREQ_50HZ;  // acceleration
    broadcast_freq[3] = DataBroadcast::FREQ_0HZ;   // linear velocity
    broadcast_freq[4] = DataBroadcast::FREQ_50HZ;  // gyro
    broadcast_freq[5] = DataBroadcast::FREQ_50HZ;  // gps location
    broadcast_freq[6] = DataBroadcast::FREQ_0HZ;   // magnetometer
    broadcast_freq[7] = DataBroadcast::FREQ_1HZ;   // remote controller data
    broadcast_freq[8] = DataBroadcast::FREQ_0HZ;   // Gimbal
    broadcast_freq[9] = DataBroadcast::FREQ_0HZ;   // flightstatus
    broadcast_freq[10] = DataBroadcast::FREQ_10HZ; //battery
    broadcast_freq[11] = DataBroadcast::FREQ_0HZ;  //control device
    vehicle->broadcast->setBroadcastFreq(broadcast_freq);

    // Wait for pilot input to start the program from the controller
    std::cout << "WAITING FOR CONTROLLER INPUT !!\n";
    int x = 0;
    for (x = 0; x < 3; x++)
    {
        do
        {
            rc = vehicle->broadcast->getRC();
            //std::cout << "Sticks = " << rc.roll << ", " << rc.pitch << ", " << rc.yaw << ", " << rc.throttle << "\n";
        } while ((rc.throttle < 9000));
        sleep(1);
    }

    // Setup the rangefidner
    Rangefinder rangefinder;
    if (!rangefinder.initializeI2C(0x31))
    {
        std::cout << "Rangefinder not connected\n";
        return -1;
    }
    rangefinder.set_in_use(true);
    rangefinder.set_offset(-0.32, 0);

    // Initialize the controller and Kalman filter model
    // RollPitchController controller;
    PIDPositionControl controller;
    controller.init_controller(vehicle);

    controller.set_rangefinder(&rangefinder);
    controller.set_reference(0, 0, 0, 0);

    pose_t position = controller.get_position();
    pose_t gps_pos = controller.get_GPS_position();
    target_pos reference = controller.get_reference();

    // Display interactive prompt

    controller.set_simulation(0);

    controller.set_PID_limits(1.5, -1.5);

    controller.switch_to_GPS_localizer();

    // Set the track dependent variables (height for popping ballons and starting area a or b)
    float target_h = 3.4;

    controller.takeoff(target_h);
    controller.waitForPositionReached(6);

    std::cout << "Ready to find an move to \"Balloon\"" << std::endl;

    controller.set_reference(8.0, 0, target_h + 1, 0);
    controller.waitForPositionReached(10);
    controller.set_reference(2.0, 3, target_h + 4, 0);
    controller.waitForPositionReached(10);
    controller.set_reference(0.0, 0, target_h + 2, 0);
    controller.waitForPositionReached(10);

    // Close everything once the search is over and hover to let the pilot land it
    controller.stop_movement();
    controller.stop_controller();
    std::cout << "Controller stopped\n";

    usleep(5000000);

    vehicle->releaseCtrlAuthority(functionTimeout);

    return 0;
}
