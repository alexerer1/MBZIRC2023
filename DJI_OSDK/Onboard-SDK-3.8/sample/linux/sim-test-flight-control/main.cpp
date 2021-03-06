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

#include "orig-flight-control-sample.hpp"
#include "attitude-control.hpp"
#include "velocity-control.hpp"
#include "virtual-rc-control.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

/*! main
 *
 */
int
main(int argc, char** argv)
{
  // Initialize variables
  int functionTimeout = 1;

  // Setup OSDK.
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle*   vehicle = linuxEnvironment.getVehicle();
  if (vehicle == NULL)
  {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }

  // Obtain Control Authority
  vehicle->obtainCtrlAuthority(functionTimeout);

  // Display interactive prompt
  std::cout
    << "| Available commands:                                            |"
    << std::endl;
  std::cout
    << "| [a] Monitored Takeoff + Landing                                |"
    << std::endl;
  std::cout
    << "| [b] Monitored Takeoff + Position Control + Landing             |"
    << std::endl;
  std::cout
    << "| [c] Monitored Takeoff + Attitude Control + Landing             |"
    << std::endl;
  std::cout
    << "| [d] Monitored Takeoff + Velocity + Landing             |"
    << std::endl;
  std::cout
    << "| [e] Virtual RC  |"
    << std::endl;
  char inputChar;
  std::cin >> inputChar;

  switch (inputChar)
  {
    case 'a':
      monitoredTakeoff(vehicle);
      sleep(2);
      monitoredLanding(vehicle);
      break;
    case 'b':
      monitoredTakeoff(vehicle);
      moveByPositionOffset(vehicle, 0, 6, 6, 30);
      moveByPositionOffset(vehicle, 6, 0, -3, -30);
      moveByPositionOffset(vehicle, -6, -6, 0, 0);
      monitoredLanding(vehicle);
      break;
    case 'c':
        monitoredTakeoff(vehicle);
        //vehicle->control->armMotors();
        //sleep(1);
        //controlSetAttitudeRPY(vehicle,0,0,0,1);
        controlSetAttitudeRPY(vehicle,-7,0,0,1);
        std::cout << "Done" << std::endl;
        controlSetAttitudeRPY(vehicle,4,-6,0,1);
        sleep(5);
        monitoredLanding(vehicle);
        break;
    case 'd':
        monitoredTakeoff(vehicle);
        sleep(2);
        controlSetVelocityYawRate(vehicle,1,0,0,0);
        std::cout << "Sleeping" << std::endl;
        sleep(1);
        controlSetAttitudeRPY(vehicle,0,0,70,1);
        std::cout << "Vel" << std::endl;
        controlSetVelocityYawRate(vehicle,0,1,0,0);
        std::cout << "RPY" << std::endl;
        controlSetAttitudeRPY(vehicle,0,0,0,1);
        sleep(2);
        monitoredLanding(vehicle);
        break;
    case 'e':
        testVirtualRCControl(vehicle);
        break;
    default:
      break;
  }

  return 0;
}
