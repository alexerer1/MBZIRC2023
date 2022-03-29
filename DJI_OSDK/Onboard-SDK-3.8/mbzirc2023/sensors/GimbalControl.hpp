#ifndef GIMBAL_CONTROL_HPP
#define GIMBAL_CONTROL_HPP

class GimbalControl {

private:
	float angle;
	int init_done = 0;
	int pin = 1;
	float deg2pwm = 1;
	int pwm_middle = 75;
	int max_angle = 45;

public:

	int init();
	int set_angle(float desired_angle);
	float get_angle();

};

#endif