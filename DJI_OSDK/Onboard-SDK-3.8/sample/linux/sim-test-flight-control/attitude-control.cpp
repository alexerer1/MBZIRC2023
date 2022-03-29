
#include "attitude-control.hpp"
#include "orig-flight-control-sample.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

bool
controlSetAttitudeRPY(Vehicle *vehicle, float roll, float pitch, float yaw, float height)
{

    Telemetry::GlobalPosition currentPosition;
    Telemetry::Vector3f localOffset;
    Telemetry::Quaternion quaternionAttitude;
    Telemetry::Vector3f attitudeAngles;
    
    double loopTime = 1000 / 50; // ~ 50 Hz
    double elapsedTime = 0;
    double timeLimit = 4000;
    
    std::cout << "Setting roll, pitch and yaw:\n" << roll << "\t" << pitch << "\t" << yaw << "\n";
    
    currentPosition = vehicle->broadcast->getGlobalPosition();
    
    std::cout << "Height = " << currentPosition.height << std::endl;
    
    while( elapsedTime < timeLimit ) {
        vehicle->control->attitudeAndVertVelCtrl(roll, pitch, yaw, 0);
        
        usleep( loopTime*1000 );
        elapsedTime += loopTime;
        
    }
    
    return ACK::SUCCESS;
}
