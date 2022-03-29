
#include "velocity-control.hpp"
#include "orig-flight-control-sample.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

bool
controlSetVelocityYawRate(Vehicle *vehicle, float Vx, float Vy, float Vz, float yawRate)
{

    Telemetry::GlobalPosition currentPosition;
    Telemetry::Vector3f localOffset;
    Telemetry::Quaternion quaternionAttitude;
    Telemetry::Vector3f attitudeAngles;
    
    double loopTime = 1000 / 50; // ~ 50 Hz
    double elapsedTime = 0;
    double timeLimit = 10000;
    
    std::cout << "Setting Vx, Vy and Vz:\n" << Vx << "\t" << Vy << "\t" << Vz << "\n";
    
    currentPosition = vehicle->broadcast->getGlobalPosition();
    
    std::cout << "Height = " << currentPosition.height << std::endl;
    
    while( elapsedTime < timeLimit ) {
        
        vehicle->control->velocityAndYawRateCtrl(Vx, Vy, Vz, yawRate);
        
        usleep( loopTime*1000 );
        elapsedTime += loopTime;
        
    }
    
    return ACK::SUCCESS;
}
