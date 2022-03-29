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

#include <iostream>

// DJI OSDK includes
#include "dji_status.hpp"
#include <dji_vehicle.hpp>
// Helpers
#include <dji_linux_helpers.hpp>

#include "CameraServerInterface.hpp"

#include <csignal>
#include <cstdio>

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

  // Setup OSDK.
  // LinuxSetup linuxEnvironment(argc, argv);
  // Vehicle*   vehicle = linuxEnvironment.getVehicle();
  // if (vehicle == NULL)
  // {
  //     std::cout << "################################\n";
  //     std::cout << "Vehicle not initialized NO PITCH ROLL AWAILABLE.\n";
  //     std::cout << "################################\n";
  //     //return -1;
  // }
  // sleep(1);

  std::signal(SIGINT, ctrl_c_handle);

  // Display interactive prompt
  cout << "Remeber to start MBZIRC/CameraServer/build/camera_server  \n";
  std::cout << "| Available commands:                   |" << std::endl;
  std::cout << "| Press [a]: Get position               |" << std::endl;
  std::cout << "| Press [b]: Test gimbal                |" << std::endl;

  char inputChar;

  int counter = 0;
  float temp = -1;
  std::cin >> inputChar;

  CameraServer camera;

  pose_t camera_pose;

  switch (inputChar)
  {
    case 'a':

      camera.init_server(0);

      while (true)
        camera_pose = camera.get_camera_position();
      std::cout << camera_pose.num_pose << std::endl;
      usleep(100000);
      break;

    case 'b':

      camera.init_server(0);

      cout << "Ready to change gimbal angle\n";
      cin >> inputChar;
      if (camera.set_gimbal_angle(20))
        cout << "Angle set, ready to return to 0\n";
      else
      {
        cout << "Failed to set angle\n";
        break;
      }           

      cin >> inputChar;

      if (camera.set_gimbal_angle(0))
        cout << "Angle set, done\n";
      else
      {
        cout << "Failed to set angle\n";
        break;
      }

      break;

    default:
      break;
  }
  /*
    if( !shutdown(lmssrv.sockfd,SHUT_RDWR) )
      std::cout << "Socket closed\n";
  */
  printf("\nprogram failed successfully! ");
  return 0;
}
