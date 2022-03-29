#include "PickupArm.hpp"
#include <wiringPi.h>

void PickupArm::init_pins()
{
    wiringPiSetup();
        
	pinMode(switch_pin, INPUT);
	pinMode(magnet_pin, OUTPUT);

	digitalWrite(magnet_pin, LOW);
    
}

bool PickupArm::switch_pressed()
{
    int status = digitalRead(switch_pin);
    
    if( status ) return false;
    else return true;
}
    
void PickupArm::magnet_on()
{
    digitalWrite(magnet_pin, HIGH);
    magnet_status = 1;
}
    
void PickupArm::magnet_off()
{
    digitalWrite(magnet_pin, LOW);
    magnet_status = 0;
}

int PickupArm::get_magnet_status()
{
    return magnet_status;
}
