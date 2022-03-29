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
// K-develope

// DJI OSDK includes
#include "dji_status.hpp"
#include <dji_vehicle.hpp>
// Helpers
#include <dji_linux_helpers.hpp>

#include <cmath>
#include <csignal>
#include <cstdio>

#include "boxGripper.hpp"
#include "CameraServerInterface.hpp"
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
    broadcast_freq[10] = DataBroadcast::FREQ_10HZ; // battery
    broadcast_freq[11] = DataBroadcast::FREQ_0HZ;  // control device
    vehicle->broadcast->setBroadcastFreq(broadcast_freq);

    // Wait for pilot input to start the program from the controller
    std::cout << "WAITING FOR CONTROLLER INPUT !!\n";
    int x = 0;
    for (x = 0; x < 3; x++)
    {
        do
        {
            rc = vehicle->broadcast->getRC();
            // std::cout << "Sticks = " << rc.roll << ", " << rc.pitch << ", " << rc.yaw << ", " << rc.throttle << "\n";
        } while ((rc.throttle < 9000));
        sleep(1);
    }

    // Setup the camera server
    std::cout << "Initializing camera server\n";
    CameraServer camera;
    if (camera.init_server(0))
        std::cout << "Camera Connected\n";
    else
        return -1;

    // Setup the rangefidner
    Rangefinder rangefinder;
    if (!rangefinder.initializeI2C(0x31))
    {
        std::cout << "Rangefinder not connected\n";
        return -1;
    }
    rangefinder.set_in_use(true);
    rangefinder.set_offset(-0.32, 0);

    BoxGripper boxGripper;
    // Initialize the controller and Kalman filter model
    // RollPitchController controller;
    boxGripper.init();
    PIDPositionControl controller;
    controller.init_controller(vehicle);

    // Add sensors to the controller and reset position to camera

    // amera.set_localizer_type(FIND_BOX);
    // camera.set_localizer_type(FIND_NAVAL_BOX);
    controller.set_camera_localizer(&camera);
    controller.set_rangefinder(&rangefinder);
    controller.set_reference(0, 0, 0, 0);
    controller.switch_to_camera_localizer(FIND_NAVAL_BOX);

    pose_t camera_pose = camera.get_camera_position();
    pose_t position = controller.get_position();
    pose_t gps_pos = controller.get_GPS_position();
    pose_t converted_position;
    target_pos reference = controller.get_reference();

    // Display interactive prompt

    controller.set_simulation(0);

    controller.set_PID_limits(1.5, -1.5);

    controller.print = 1;

    usleep(10000);
    int angleSet = 0;
    std::cout << "Setting Gimbal angle\n";
    while (angleSet < 1)
    {
        angleSet = camera.set_gimbal_angle(17);
        usleep(10000);
    }

    // Set the track dependent variables (height for popping ballons and starting area a or b)
    float takeoff_height = 5;
    float height = 88;
    controller.takeoff(takeoff_height);
    // usleep(200000);
    // rangefinder.set_filter_h(takeoff_height);
    controller.waitForPositionReached(6);

    std::cout << "Checking area for \"Box\"" << std::endl;

    // While there are still areas on the course not checked
    while (true)
    {
        // std::cout <<  camera.get_localizer_type();
    scan_for_box:
        // camera_pose = camera.get_camera_position();
        if (camera_pose.valid)
        {
            gps_pos = controller.get_GPS_position();
            height = rangefinder.getHeight();
            converted_position = boxGripper.image_to_position(camera_pose, height);

            cout << "Found box.\n";
            cout << "campera pose:\t" << camera_pose.x << "\t\t y:" << camera_pose.y << "\t h: gps=" << gps_pos.z << " RF= " << height << " \t th:" << camera_pose.th << endl;
            cout << "converted pose:\t" << converted_position.x << "\t y:" << converted_position.y << endl;
            controller.set_reference(camera_pose.x, camera_pose.y, takeoff_height, 0);
            controller.waitForPositionReached(5);
        }
        else
        {
            cout << "Lost box.\n";
            controller.set_reference(0, 0, takeoff_height, 0);
            controller.waitForPositionReached(5);
        }
    }

finish_mission:

    // Close everything once the search is over and hover to let the pilot land it
    controller.stop_movement();
    controller.waitForPositionReached(5);
    controller.stop_controller();
    std::cout << "Controller stopped\n";

    usleep(3000000);

    vehicle->releaseCtrlAuthority(functionTimeout);

    return 0;
}
