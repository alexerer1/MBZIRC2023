#include "GimbalControl.hpp"
#include <iostream>
#include <stdio.h>
#include <iostream>
#include <wiringPi.h>
#include <csignal>
#include <cstdio>
#include <unistd.h>
#include <cmath>

int GimbalControl::init()
{

    auto myprivs = geteuid();
    
    if( myprivs != 0) {
        std::cout << "Not run as SUDO!!!\n";
        return 0;
    }
    init_done = 1;
    
	wiringPiSetup();
	
	int pwm_val = 0;

	pinMode( pin, PWM_OUTPUT );

	pwmWrite( pin, pwm_val );

	pwmSetMode(PWM_MODE_MS);
	pwmSetClock(375);

	return 1;
}

int GimbalControl::set_angle(float desired_angle)
{
	if( !init_done ) return 0;

	if( desired_angle > max_angle || desired_angle < -max_angle ) return 0;

	angle = desired_angle;

	int pwm_val = (int) (pwm_middle + desired_angle * deg2pwm);

	pwmWrite( pin, pwm_val );

	return 1;
}

float GimbalControl::get_angle()
{
	return angle;
}