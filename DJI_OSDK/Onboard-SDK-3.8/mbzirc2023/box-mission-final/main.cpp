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

#include "CameraServerInterface.hpp"

#include "Rangefinder.hpp"
#include "FlirCamera.hpp"
#include "PickupArm.hpp"

#include "RollPitchController.hpp"
#include "PIDPositionControl.hpp"

#include <cmath>

#include "GeneralCommands.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;
using namespace std;

//
//	Finds the average of 10 old values in an array
//	@param: head is the position of the newest value
//	@param: values[] is the array (of size 20) to find the old average
//	@return the average of the 10 old values
//
float average_old(int head, float values[])
{
    float sum = 0;
    int local_head = head + 1;
    if (local_head == 20)
        local_head = 0;
    for (int i = 0; i < 10; i++)
    {
        sum += values[local_head];
        local_head++;
        if (local_head == 20)
            local_head = 0;
    }
    sum /= 10.0;
    return sum;
}

//
//	Finds the average of the 10 newest values in an array
//	@param: head is the position of the newest value
//	@param: values[] is the array (of size 20) to find the new average
//	@return the average of the 10 newest values
//
float average_new(int head, float values[])
{
    float sum = 0;
    int local_head = head;
    for (int i = 0; i < 10; i++)
    {
        sum += values[local_head];
        local_head--;
        if (local_head == -1)
            local_head = 19;
    }
    sum /= 10.0;
    return sum;
}

/*! main
 *	the main mission program for Challenge 2 box pickup and placement
 */
int main(int argc, char **argv)
{

    // Initialize variables
    int functionTimeout = 1;
    int x = 0;

    // RC sticky test
    Telemetry::RC rc;

    // Setup OSDK.
    LinuxSetup linuxEnvironment(argc, argv);
    Vehicle *vehicle = linuxEnvironment.getVehicle();
    if (vehicle == NULL)
    {
        std::cout << "Vehicle not initialized, exiting.\n";
        return -1;
    }

    // Get control authority from the M100 so the RPi can control it
    vehicle->obtainCtrlAuthority(functionTimeout);

    // Set the data stream frequencies needed from the M100
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


    // Wait for the correct stick position from the pilot to start the autonomous program (throttle up for ~3 seconds)
    std::cout << "WAITING FOR CONTROLLER INPUT !!\n";
    for (x = 0; x < 3; x++)
    {
        do
        {
            rc = vehicle->broadcast->getRC();
            //std::cout << "Sticks = " << rc.roll << ", " << rc.pitch << ", " << rc.yaw << ", " << rc.throttle << "\n";
        } while ((rc.throttle < 9000));
        sleep(1);
    }

    // Initialize camera
    CameraServer downCamera;
    downCamera.init_server(0);

    // Initialize rangefinder (short 0x30)
    Rangefinder rangefinder;
    if (!rangefinder.initializeI2C(0x30))
    {
        std::cout << "Rangefinder not connected\n";
    }
    rangefinder.set_offset(0.3, 0);
    rangefinder.set_in_use(true);
    rangefinder.set_filter_h(3.0);
    rangefinder.use_h_filter = false;

    // Initialize the controller and Kalman filter model
    // RollPitchController controller;
    PIDPositionControl controller;
    controller.set_reference(0, 0, 0, 0);
    controller.init_controller(vehicle);
    controller.print = -1;

    // Add sensors to the controller and reset position to camera
    controller.set_camera_localizer(&downCamera);
    controller.set_rangefinder(&rangefinder);

    // controller.set_global_estimator(GPS);
    // controller.set_global_estimator(CAMERA);
    controller.set_simulation(0);
    controller.set_PID_limits(1.0,-1.0);

    cout << "Controller setup done\n";
    
    // Initlize the arm pins
    std::cout << "Init arm\n";
    PickupArm arm;
    arm.init_pins();


    // Initilize variables to be used later
    float rf_value;

    Telemetry::Vector3f attitude;
    Telemetry::Quaternion quaterion;

    float old_rf = rangefinder.getHeight();
    float old_average = 0;
    float new_average = 0;
    int num_change = 0;

    float old_value[20] = {0.20};
    int head = 0;

    int in_air = 0;
    bool over_wall = 0;
    int cnt = 0;
    bool box_caught = false;

    pose_t gps_pose;
    pose_t camera_pose;

    // Set camera localization to box
    downCamera.set_localizer_type(FIND_BOX);



    // Start flight and takeoff to 3 meters up
    controller.takeoff(3);
    controller.waitForPositionReached(3);

    // controller.set_desired_height(3.0);
    // controller.waitForPositionReached(3);


    // Set the approximate values for the position of the boxes
    float waypoint = 4.0;
    float forwardDistance = 7.5;
    float position1 = 7.0;
    float position2 = 10.0;


    // Fly towards the boxes based on the guessed values
    controller.set_reference(0, waypoint, 3.0, 0);
    controller.waitForPositionReached(5);

    controller.set_reference(forwardDistance, waypoint, 3.0, 0);
    controller.waitForPositionReached(4);

    controller.set_reference(forwardDistance, position1, 3.0, 0);
    controller.waitForPositionReached(6);

    int oldNumberOfPoses = 0, frame_count = 0;
    float detectionHeight = 3.0;
    float detectionY;

    // Loop the checks if a good box can be found at the current position
    for(;;)
    {
    	// Check for new frames
        camera_pose = downCamera.get_camera_position();
        if( camera_pose.num_pose > oldNumberOfPoses ) {
            oldNumberOfPoses = camera_pose.num_pose;
            frame_count++;
        }

        // Check if box is found
        if (camera_pose.valid) {
            detectionHeight = 3.0;
            detectionY = position1;
            goto start_pickup;
        }
        // If 9 frames are checked without finding a box move to the next position
        if (frame_count >= 9)
        {
            controller.set_reference(forwardDistance, position1, 2.0, 0);
            controller.waitForPositionReached(5);
            break;
        }
        usleep(42000);
    }

    // Same loop as before but a the next position to test
    frame_count = 0;
    for(;;)
    {
        camera_pose = downCamera.get_camera_position();
        if( camera_pose.num_pose > oldNumberOfPoses ) {
            oldNumberOfPoses = camera_pose.num_pose;
            frame_count++;
        }

        if (camera_pose.valid) {
            detectionHeight = 2.0;
            detectionY = position1;
            goto start_pickup;
        }
        if (frame_count >= 9)
        {
            controller.set_reference(forwardDistance, position2, 3.0, 0);
            controller.waitForPositionReached(5);
            break;
        }
        usleep(42000);
    }

    // Same loop as before but a the next position to test
    frame_count = 0;
    for(;;)
    {
        camera_pose = downCamera.get_camera_position();
        if( camera_pose.num_pose > oldNumberOfPoses ) {
            oldNumberOfPoses = camera_pose.num_pose;
            frame_count++;
        }

        if (camera_pose.valid) {
            detectionHeight = 3.0;
            detectionY = position2;
            goto start_pickup;
        }
        if (frame_count >= 9)
        {
            controller.set_reference(forwardDistance, position2, 2.0, 0);
            controller.waitForPositionReached(5);
            break;
        }
        usleep(42000);
    }

    // Same loop as before but a the next position to test
    // If this fails the copter is landed
    frame_count = 0;
    for(;;)
    {
        camera_pose = downCamera.get_camera_position();
        if( camera_pose.num_pose > oldNumberOfPoses ) {
            oldNumberOfPoses = camera_pose.num_pose;
            frame_count++;
        }

        if (camera_pose.valid) {
            detectionHeight = 2.0;
            detectionY = position2;
            goto start_pickup;
        }
        if (frame_count == 9)
        {
            controller.set_relative_reference(-3.0, 0, 0, 0);
            controller.waitForPositionReached(5);
            controller.stop_controller();
            std::cout << "Controller stopped\n";
            vehicle->control->land(30);
            usleep(10000000);
            vehicle->releaseCtrlAuthority(functionTimeout);
            return 1;
        }
        usleep(42000);
    }

    // controller.enable_control_action(1);
    start_pickup:
    box_caught = false;
    rangefinder.use_h_filter = false;
    while (!box_caught)
    {
        arm.magnet_off();

        // switch to box localization and start the decent towards the box
        controller.switch_to_camera_localizer(FIND_BOX);
        controller.waitForPositionReached(3);
        controller.set_reference(0.0, 0.03, detectionHeight, 0);
        controller.waitForPositionReached(5);

        controller.set_reference(0.0, 0.03, 1.8, 0);
        controller.waitForPositionReached(5);

        controller.set_reference(0.10, 0.03, 1.2, 0);
        controller.waitForPositionReached(3);

        controller.set_reference(0.10, 0.03, 0.8, 0);
        controller.waitForPositionReached(3);

        // Turn on the magnets before the final dip to pick up the box
        arm.magnet_on();
        controller.set_reference(0.10, 0.03, 0.20, 0);
        controller.force_pose_target();

        // Check if it touches the box with the switch so it can fly up again
        cnt = 0;
        while (!arm.switch_pressed() && cnt < 3000)
        {
            usleep(1000);
            cnt++;
        }

        // Switch back to GPS localization to try again or fly towrads the wall
        controller.switch_to_GPS_localizer();
        usleep(100000);

        controller.set_desired_height(1.8);
        controller.waitForPositionReached(7);

        usleep(1000000);

        // The box is on the magnets
        if (arm.switch_pressed())
        {
            box_caught = true;
        }
        // No box, needs to retry
        else
        {
            arm.magnet_off();
            controller.set_reference(forwardDistance, detectionY, detectionHeight, 0);
            controller.waitForPositionReached(5);
        }
    }

    // Fly up and start the flight towards the box
    controller.set_desired_height(3.0);
    controller.waitForPositionReached(5);

    gps_pose = controller.get_GPS_position();


    // Setup the height filtering on the rangefinder so it doesn't fly up when flying over the wall
    rangefinder.set_filter_h(3.0);
    rangefinder.use_h_filter = true;

    in_air = 0;
    rangefinder.getHeightRP(0,0);
    old_rf = rangefinder.getUnfilteredHeight();
    old_average = 0;
    new_average = 0;
    num_change = 0;

    old_value[20] = {0.20};
    head = 0;

    cnt = 0;

    controller.set_relative_reference(15, 0, 0, 0);

    // Loop for checking if the wall has been passed
    while (cnt < 1000)
    {

        quaterion = vehicle->broadcast->getQuaternion();
        attitude = toEulerAngle(&quaterion);

        // Get rangefinder reading
        rf_value = rangefinder.getUnfilteredHeight();

        // Record the rangefinder value to find the old and new average
        old_value[head] = rf_value;
        old_average = average_old(head, old_value);
        new_average = average_new(head, old_value);

        // cout << "Old = " << old_average << "  |  New = " << new_average << endl;

        // Checks if the copter is flying high enough to start the checking for wall
        if (rf_value > 2.0 && in_air <= 5)
        {
            in_air++;
            cout << "Copter in air " << in_air << endl;
            // cout << "in_air: " << in_air << endl;
        }

        // When it is high enough check the rangefinder values
        if (in_air > 5)
        {
            // cout << fabs(old_average - new_average) << endl;
            // Check if the difference between the old and new average is large enough to read as a wall
            if (fabs(old_average - new_average) > 1.0)
            {
                num_change++;
                // cout << "Change detected | " << num_change << endl;
            }
            else
                num_change = 0;

            // When the change in rf values has been there 5 times in a row check if it is flying over the wall or past the wall
            // If past the wall break of the loop
            if (num_change == 5)
            {
                if (new_average < old_average && !over_wall)
                {
                    cout << "Flying over wall now\n";
                    over_wall = 1;
                }
                else if (new_average > old_average && over_wall)
                {
                    cout << "Flew past the wall\n";
                    over_wall = 0;
                    break;
                }

                num_change = 0;
            }
        }

        head++;
        if (head == 20)
            head = 0;

        old_rf = rf_value;
        // cout << rf_value << endl;
        cnt++;
        usleep(30000);
    }

    // Turn off magnets and stop movement 
    arm.magnet_off();

    usleep(1000000);

    controller.stop_movement();
    controller.waitForPositionReached(5);


    // Fly back to the point where the last box was found and repeat
    rangefinder.use_h_filter = false;

    controller.set_desired_height(4.0);
    controller.waitForPositionReached(3);

    controller.set_reference(gps_pose.x, gps_pose.y, 4.0, gps_pose.th);
    controller.waitForPositionReached(15);

    goto start_pickup;
    
    // Unreachable landing code
    // Pilot needs to land it
    controller.set_reference(0, 0, 4.0, 0);
    controller.waitForPositionReached(20);

    controller.stop_controller();
    std::cout << "Controller stopped\n";

    vehicle->control->land(30);
    usleep(1000000);

    vehicle->releaseCtrlAuthority(functionTimeout);
    // downCamera.set_localizer_type(ARUCO);

    // downCamera.stop_camera();
    std::cout << "Camera stopped\n";

    vehicle->releaseCtrlAuthority(functionTimeout);

    return 0;
}
