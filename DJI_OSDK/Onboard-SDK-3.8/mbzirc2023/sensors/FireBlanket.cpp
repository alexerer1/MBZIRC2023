#include "FireBlanket.hpp"
#include <wiringPi.h>

void FireBlanket::init_pins()
{
    wiringPiSetup();
        
	pinMode(blanket_pin, OUTPUT);
	pinMode(release_pin, OUTPUT);

	digitalWrite(blanket_pin, LOW);
    digitalWrite(release_pin, LOW);
    
}

void FireBlanket::release_blanket()
{
    digitalWrite(release_pin, LOW);
}


void FireBlanket::drop_blanket()
{
    digitalWrite(release_pin, LOW);
    digitalWrite(blanket_pin, LOW);
}

void FireBlanket::turn_on_magnets()
{
    digitalWrite(release_pin, HIGH);
    digitalWrite(blanket_pin, HIGH);    
}