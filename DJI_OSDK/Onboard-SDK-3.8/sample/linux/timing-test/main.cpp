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

#include "flight_control_sample.hpp"
#include "loop-heartbeat.hpp"

#include <wiringPi.h>
#include <stdlib.h>
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

  
      uint8_t broadcast_freq[16];
    vehicle->broadcast->setVersionDefaults(broadcast_freq);
    broadcast_freq[0] = DataBroadcast::FREQ_0HZ; // timestamp
    broadcast_freq[1] = DataBroadcast::FREQ_100HZ; // attitude
    broadcast_freq[2] = DataBroadcast::FREQ_100HZ; // acceleration
    broadcast_freq[3] = DataBroadcast::FREQ_0HZ; // linear velocity
    broadcast_freq[4] = DataBroadcast::FREQ_100HZ; // gyro
    broadcast_freq[5] = DataBroadcast::FREQ_50HZ; // gps location
    broadcast_freq[6] = DataBroadcast::FREQ_0HZ; // magnetometer
    broadcast_freq[7] = DataBroadcast::FREQ_0HZ; // remote controller data
    broadcast_freq[8] = DataBroadcast::FREQ_0HZ; // Gimbal
    broadcast_freq[9] = DataBroadcast::FREQ_100HZ; // flightstatus
    broadcast_freq[10] = DataBroadcast::FREQ_0HZ; //battery
    broadcast_freq[11] = DataBroadcast::FREQ_100HZ; //control device
    broadcast_freq[11] = DataBroadcast::FREQ_0HZ; //control device
    vehicle->broadcast->setBroadcastFreq(broadcast_freq);
  
    HeartbeatTimer timer;
    
      //wiringPiSetup();

  int magnet_pin = 27;

  //pinMode(magnet_pin, OUTPUT);

  //digitalWrite(magnet_pin, LOW);

    
  struct timespec gettime_now; 
    
    double last_time;
    double new_time;
    double freq;
    long start_log_time;
    
    float old_v = 24;
    float new_v = 10;
    float v_freq = 0;
    float v_count = 0;
  // Display interactive prompt
  std::cout
    << "| Available commands:                                            |"
    << std::endl;
  std::cout
    << "| [a] Monitored Takeoff + Landing                                |"
    << std::endl;

  char inputChar;
  std::cin >> inputChar;

  timer.start_hb(60);
  Vector3f attitude;
  Quaternion atti_quad;
  Telemetry::Vector3f acc;
  Telemetry::TimeStamp drone_time;
  
  std::freopen( "test.txt", "w", stderr );
  std::cerr.precision(10);
  

  Telemetry::Vector3f localOffset;
  
  Telemetry::GlobalPosition originGPS_Position = vehicle->broadcast->getGlobalPosition();

  Telemetry::GlobalPosition currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
  
  localOffsetFromGpsOffset(vehicle, localOffset,
                           static_cast<void*>(&currentBroadcastGP),
                           static_cast<void*>(&originGPS_Position));

  long control_time;
  float pitch_cmd = 0;
  float roll_cmd = 0;
  int count = 0;
  int limit = 200;
  VirtualRCData vrcData;
  int elapsedTime = 0;
  double timeOut;
  Telemetry::SDKInfo sdkStatus;
  Telemetry::Status   status;
  Telemetry::Battery battery;

  int toggle = 3;

  switch (inputChar)
  {
    case 'a':
    usleep(200000);
    clock_gettime(CLOCK_REALTIME, &gettime_now);
    
    start_log_time = gettime_now.tv_sec;
    vehicle->virtualRC->setVehicle(vehicle);
    last_time = (double)( gettime_now.tv_sec - start_log_time) + (double)gettime_now.tv_nsec/(1000000000); 
    
    // sdkStatus = vehicle->broadcast->getSDKInfo();
    
    // std::cout << "ControlMode = " << (unsigned) sdkStatus.controlMode << std::endl;
    // std::cout << "FlightStatus = " << (unsigned) sdkStatus.flightStatus << std::endl;
    // std::cout << "DeviceStatus = " << (unsigned) sdkStatus.deviceStatus << std::endl;
    // std::cout << "VRCStatus = " << (unsigned) sdkStatus.vrcStatus << std::endl;

  // vehicle->virtualRC->resetVRCData();
      
  // //     std::cout << vrcData.throttle << "  " << vrcData.roll << std::endl;
  //     std::cout << "Turning on virtual\n";
  //     vehicle->virtualRC->setControl(true, VirtualRC::CutOff_ToRealRC);
      
  //     std::cout << "Checking if virtual is on\n";
  //     timeOut  = 1000000;
  //     while( elapsedTime < timeOut ){
          
  //         if( vehicle->virtualRC->isVirtualRC() ){
  //             std::cout << "Virtual ON" << std::endl;
  //             break;
  //         }
  //         else printf("Off\n");
  //         // vehicle->virtualRC->setControl(true, VirtualRC::CutOff_ToRealRC);
  //         usleep(1000);
  //         elapsedTime += 1000;
          
  //     }
  //     if( elapsedTime >= timeOut ){
  //         std::cout << "Virtual timeout reached" << std::endl;
  //         return 0;   
  //     }

  //   vrcData = vehicle->virtualRC->getVRCData();
  //   vrcData.mode = 1552;
  //   vehicle->virtualRC->sendData(vrcData);
  //     vehicle->obtainCtrlAuthority(functionTimeout);



      // vehicle->control->takeoff();

      while( true ){
       
        while( !timer.get_hb_flag() ) usleep(2);
        
//         drone_time = vehicle->broadcast->getTimeStamp();
        // acc = vehicle->broadcast->getAcceleration();
        atti_quad = vehicle->broadcast->getQuaternion();
        attitude = toEulerAngle( &atti_quad );
      
        
        clock_gettime(CLOCK_REALTIME, &gettime_now);
        new_time = (double)( gettime_now.tv_sec - start_log_time) + (double)gettime_now.tv_nsec/(1000000000.0); 
        freq = 1.0/(new_time - last_time);
        last_time = new_time;
        new_v = attitude.x;
        
//         std::cout << abs(new_v - old_v) << new_v << old_v << std::endl;
        
        if( fabs(new_v-old_v) > 0.0000001 ){
            // std::cout << "Change\n";
                old_v = new_v;
                v_freq = 1.0/(new_time - v_count);
                v_count = new_time;
        }

            sdkStatus = vehicle->broadcast->getSDKInfo();
    
    // std::cout << "ControlMode = " << (unsigned) sdkStatus.controlMode << std::endl;
    // std::cout << "FlightStatus = " << (unsigned) sdkStatus.flightStatus << std::endl;
    // std::cout << "DeviceStatus = " << (unsigned) sdkStatus.deviceStatus << std::endl;
    // std::cout << "VRCStatus = " << (unsigned) sdkStatus.vrcStatus << std::endl;
  
    // status = vehicle->broadcast->getStatus()  ;

    // std::cout << "Flight = " << (unsigned) status.flight << std::endl;
    // std::cout << "Mode = " << (unsigned) status.mode << std::endl;
    // std::cout << "Gear = " << (unsigned) status.gear << std::endl;
    // std::cout << "Error = " << (unsigned) status.error << std::endl;


        count++;

        if( count == 2) {
            if( last_time > 3 && last_time < 7 && pitch_cmd == 0) {
              pitch_cmd = 1;
              // vrcData.pitch = 1024 - 300;
              std::cout << "Command sent\n";
              // digitalWrite(magnet_pin, HIGH);
              // usleep(2000);
                  
            }
            else if( last_time > 9 && pitch_cmd == 1) {
              pitch_cmd = 0;
              std::cout << "Command sent\n";
            }
            // else if( last_time > 2 && toggle == 0) {
            //       vehicle->broadcast->setVersionDefaults(broadcast_freq);
            //     broadcast_freq[0] = DataBroadcast::FREQ_0HZ; // Timestamp
            //     broadcast_freq[1] = DataBroadcast::FREQ_0HZ; // attitude
            //     broadcast_freq[2] = DataBroadcast::FREQ_0HZ; // acceleration
            //     broadcast_freq[3] = DataBroadcast::FREQ_0HZ; // linear velocity
            //     broadcast_freq[4] = DataBroadcast::FREQ_0HZ; // gyro
            //     broadcast_freq[5] = DataBroadcast::FREQ_0HZ; // gps location
            //     broadcast_freq[6] = DataBroadcast::FREQ_0HZ; // magnetometer
            //     broadcast_freq[7] = DataBroadcast::FREQ_0HZ; // remote controller data
            //     broadcast_freq[8] = DataBroadcast::FREQ_0HZ; // Gimbal
            //     broadcast_freq[9] = DataBroadcast::FREQ_0HZ; // flightstatus
            //     broadcast_freq[10] = DataBroadcast::FREQ_0HZ; //battery
            //     broadcast_freq[11] = DataBroadcast::FREQ_0HZ; //control device
            //     vehicle->broadcast->setBroadcastFreq(broadcast_freq);

            //               toggle = 1;
            //   std::cout << "Deactivate\n";
            // }
            //             else if( last_time > 5 && toggle == 1) {
            //               toggle = 2;
            //       vehicle->broadcast->setVersionDefaults(broadcast_freq);
            //     broadcast_freq[0] = DataBroadcast::FREQ_0HZ; // timestamp 
            //     broadcast_freq[1] = DataBroadcast::FREQ_0HZ; // attitude
            //     broadcast_freq[2] = DataBroadcast::FREQ_100HZ; // acceleration
            //     broadcast_freq[3] = DataBroadcast::FREQ_0HZ; // linear velocity
            //     broadcast_freq[4] = DataBroadcast::FREQ_0HZ; // gyro
            //     broadcast_freq[5] = DataBroadcast::FREQ_0HZ; // gps location
            //     broadcast_freq[6] = DataBroadcast::FREQ_0HZ; // magnetometer
            //     broadcast_freq[7] = DataBroadcast::FREQ_0HZ; // remote controller data
            //     broadcast_freq[8] = DataBroadcast::FREQ_0HZ; // Gimbal
            //     broadcast_freq[9] = DataBroadcast::FREQ_0HZ; // flightstatus
            //     broadcast_freq[10] = DataBroadcast::FREQ_0HZ; //battery
            //     broadcast_freq[11] = DataBroadcast::FREQ_0HZ; //control device
            //     vehicle->broadcast->setBroadcastFreq(broadcast_freq);
                          
            //   std::cout << "Activated again\n";
            // }

            // sdkStatus = vehicle->broadcast->getSDKInfo();
            // status =   vehicle->broadcast->getStatus();
            // battery = vehicle->broadcast->getBatteryInfo();
            // printf("\033c\n\n");
            // std::cout << "SDKInfo:" << std::endl;
            // std::cout << "ControlMode  = " << (unsigned) sdkStatus.controlMode << std::endl;
            // std::cout << "FlightStatus = " << (unsigned) sdkStatus.flightStatus << std::endl;
            // std::cout << "DeviceStatus = " << (unsigned) sdkStatus.deviceStatus << std::endl;
            // std::cout << "VRCStatus    = " << (unsigned) sdkStatus.vrcStatus << std::endl;               
            // std::cout << "Status:" << std::endl;
            // std::cout << "Flight       = " << (unsigned) status.flight << std::endl;
            // std::cout << "Mode         = " << (unsigned) status.mode << std::endl;
            // std::cout << "Error        = " << (unsigned) status.error << std::endl;
            // std::cout << "Battery:" << std::endl;
            // std::cout << "Capacity     = " << (unsigned) battery.capacity << std::endl;
            // std::cout << "Voltage      = " << battery.voltage << std::endl;
            // std::cout << "Current      = " << battery.current << std::endl;
            // std::cout << "Percentage   = " << (unsigned) battery.percentage << std::endl;               
            // if( last_time > 3 ){
            //   if( pitch_cmd == 0 ) pitch_cmd = 10;
            //   else pitch_cmd = 0;

            // }
            clock_gettime(CLOCK_REALTIME, &gettime_now);
            control_time = gettime_now.tv_nsec;
//             std::cout << "Send\n";
            // vehicle->virtualRC->sendData(vrcData);
            // vehicle->control->attitudeYawRateAndVertVelCtrl(pitch_cmd, 0, 0, 0);
            vehicle->control->xyPosZvelAndYawRateCtrl(0, 0, pitch_cmd, 0);
            // digitalWrite(magnet_pin, LOW);
            // std::cout << vehicle->control->armMotors(1).data << std::endl;
            // std::cout << vehicle->control->disArmMotors(1).data << std::endl;
//             vehicle->control->disArmMotors(0);
            clock_gettime(CLOCK_REALTIME, &gettime_now);
            //std::cout << "Time = " << (gettime_now.tv_nsec - control_time)/1000000.0 << std::endl;
            count = 0;
        }

            // std::cout << "Last Time = " << last_time << std::endl;
        
        
        // count++;
        // if( count == limit) {
        //     if( roll_cmd < 0.5 ) roll_cmd = 1;
        //     else roll_cmd = 1;
        //     limit += 4;
        //   // vehicle->control->attitudeYawRateAndVertVelCtrl(roll_cmd, 0, 0, 0);  
        //     vehicle->control->xyPosZvelAndYawRateCtrl(0,0,0,0);
        // }
        // if( count == 200){
        //   roll_cmd = 1;
        //   vehicle->control->attitudeYawRateAndVertVelCtrl(roll_cmd, 0, 0, 0);
        // }
        // if( count == 204){
        //   roll_cmd = 0;
        //   vehicle->control->attitudeYawRateAndVertVelCtrl(roll_cmd, 0, 0, 0);
        // }
        // if( count == 208){
        //   roll_cmd = 1;
        //   vehicle->control->attitudeYawRateAndVertVelCtrl(roll_cmd, 0, 0, 0);
        // }
        // if( count == 212){
        //   roll_cmd = 0;
        //   vehicle->control->attitudeYawRateAndVertVelCtrl(roll_cmd, 0, 0, 0);
        // }
        // if( count == 216){
        //   roll_cmd = 1;
        //   vehicle->control->attitudeYawRateAndVertVelCtrl(roll_cmd, 0, 0, 0);
        // }
        // if( count == 220){
        //   roll_cmd = 0;
        //   vehicle->control->attitudeYawRateAndVertVelCtrl(roll_cmd, 0, 0, 0);
        // }
        // if( count == 224){
        //   roll_cmd = 1;
        //   vehicle->control->attitudeYawRateAndVertVelCtrl(roll_cmd, 0, 0, 0);
        // }
        // if( count == 228){
        //   roll_cmd = 0;
        //   vehicle->control->attitudeYawRateAndVertVelCtrl(roll_cmd, 0, 0, 0);
        // }

        Telemetry::GlobalPosition currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
        
        localOffsetFromGpsOffset(vehicle, localOffset,
                                 static_cast<void*>(&currentBroadcastGP),
                                 static_cast<void*>(&originGPS_Position));

        std::cerr << new_time << " " << freq << " " << " " << roll_cmd << " " << pitch_cmd << " " << localOffset.x << " " << localOffset.y << " " << localOffset.z << "\n";
        
        // std::cout << "Freq = " << freq << "   " << v_freq << "   " << new_v << "   " << acc.z << std::endl;
        
          
      }
      break;

    default:
      break;
  }

  
  return 0;
}
