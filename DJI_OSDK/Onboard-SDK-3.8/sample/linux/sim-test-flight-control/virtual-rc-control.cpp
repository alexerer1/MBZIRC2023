
#include "virtual-rc-control.hpp"
#include "orig-flight-control-sample.hpp"
#include "dji_virtual_rc.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

bool testVirtualRCControl(Vehicle *vehicle){
 
    
    double loopTime = 1000/20;
    double takeoffTime = 5000;
    double elapsedTime = 0;
    
    Telemetry::SDKInfo sdkStatus = vehicle->broadcast->getSDKInfo();
    
    std::cout << "ControlMode = " << (unsigned) sdkStatus.controlMode << std::endl;
    std::cout << "FlightStatus = " << (unsigned) sdkStatus.flightStatus << std::endl;
    std::cout << "DeviceStatus = " << (unsigned) sdkStatus.deviceStatus << std::endl;
    std::cout << "VRCStatus = " << (unsigned) sdkStatus.vrcStatus << std::endl;
    
    //monitoredTakeoff(vehicle);
    
    //vehicle->control->armMotors();
    
    std::cout << "Takeoff done. Switch to virtual RC\n";
    
    std::cout << vehicle->virtualRC->getVehicle() << std::endl;
    
//     vehicle->virtualRC->setVehicle(vehicle);
//     std::cout << vehicle->virtualRC->getVehicle() << std::endl;
    
    vehicle->virtualRC->resetVRCData();
    
//     std::cout << vrcData.throttle << "  " << vrcData.roll << std::endl;
    std::cout << "Turning on virtual\n";
    vehicle->virtualRC->setControl(true, VirtualRC::CutOff_ToRealRC);
    
    std::cout << "Checking if virtual is on\n";
    double timeOut = 1000000;
    while( elapsedTime < timeOut ){
        
        if( vehicle->virtualRC->isVirtualRC() ){
            std::cout << "Virtual ON" << std::endl;
            break;
        }
        usleep(1000);
        elapsedTime += 1000;
        
    }
    if( elapsedTime > timeOut ){
        std::cout << "Virtual timeout reached" << std::endl;
        return 0;   
    }
    
    
    
    std::cout << "Arming drone\n"; 
    VirtualRCData vrcData = vehicle->virtualRC->getVRCData();
    
    vrcData.throttle = 1024 - 660;
    vrcData.roll = 1024 - 660;
    vrcData.pitch = 1024 - 660;
    vrcData.yaw = 1024 + 660;
    vrcData.mode = 1024;
    
    for(int i = 0; i < 10; i++){
        vehicle->virtualRC->sendData(vrcData);
        usleep(1000*loopTime);
    }
     
    vrcData.throttle = 1024 + 600;
    vrcData.roll = 1024;
    vrcData.pitch = 1024;
    vrcData.yaw = 1024;
    
    std::cout << "Starting flight\n";
//     std::cout << (unsigned) OpenProtocolCMD::CMDSet::VirtualRC::data[0] << " # # # " << std::endl;
//     std::cout << (unsigned) OpenProtocolCMD::CMDSet::VirtualRC::data[1] << " # # # " << std::endl;
//     std::cout << (unsigned) OpenProtocolCMD::CMDSet::VirtualRC::settings[0] << " # # # " << std::endl;
//     std::cout << (unsigned) OpenProtocolCMD::CMDSet::VirtualRC::settings[1] << " # # # " << std::endl;
//     std::cout << (unsigned) OpenProtocolCMD::CMDSet::Control::control[0] << " # # # " << std::endl;
//     std::cout << (unsigned) OpenProtocolCMD::CMDSet::Control::control[1] << " # # # " << std::endl;
    
    elapsedTime = 0;
    std::cout << "Starting takeoff\n";
    while( elapsedTime < takeoffTime ){
     
        vehicle->virtualRC->sendData(vrcData);
        
        if( vehicle->broadcast->getGlobalPosition().height > 1.2) break;
        
        usleep(1000*loopTime);
        elapsedTime += loopTime;
    }
    std::cout << "Takeoff complete\n";
    
    elapsedTime = 0;
    vrcData.throttle = 1024;
    while( elapsedTime < 1000 ){
     
        vehicle->virtualRC->sendData(vrcData);
        
        usleep(1000*loopTime);
        elapsedTime += loopTime;
    }
    
    
    elapsedTime = 0;
    vrcData.throttle = 1024-660; 
    while( elapsedTime < 10000 ){
     
        vehicle->virtualRC->sendData(vrcData);

        usleep(1000*loopTime);
        elapsedTime += loopTime;
    }
    
    elapsedTime = 0;
    vrcData.pitch = 1024;
    while( elapsedTime < 5000 ){
     
        vehicle->virtualRC->sendData(vrcData);
        
        usleep(1000*loopTime);
        elapsedTime += loopTime;
    }
    
    vrcData.mode = 496;
    vehicle->virtualRC->sendData(vrcData);
    usleep(100000);
    
    std::cout << "Turning off virtual RC\n";
    vehicle->virtualRC->setControl(false, VirtualRC::CutOff_ToRealRC);

    elapsedTime = 0;
    while( elapsedTime < timeOut ){
        
        if( !vehicle->virtualRC->isVirtualRC() ){
            std::cout << "Virtual OFF" << std::endl;
            break;
        }
        usleep(1000);
        elapsedTime += 1000;
        
    }
    
    vehicle->obtainCtrlAuthority(10000);
    vehicle->obtainCtrlAuthority(10000);    
    
    std::cout << "Pre landing sleep\n";
    sdkStatus = vehicle->broadcast->getSDKInfo();
    
    std::cout << "ControlMode = " << (unsigned) sdkStatus.controlMode << std::endl;
    std::cout << "FlightStatus = " << (unsigned) sdkStatus.flightStatus << std::endl;
    std::cout << "DeviceStatus = " << (unsigned) sdkStatus.deviceStatus << std::endl;
    std::cout << "VRCStatus = " << (unsigned) sdkStatus.vrcStatus << std::endl;
    
    elapsedTime = 0;
    while(elapsedTime < 10){
        if( monitoredLanding(vehicle) ) break;
        elapsedTime += 1;
        sleep(1);
    }
    
    return ACK::SUCCESS;
}

