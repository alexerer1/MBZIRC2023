#ifndef GIMBAL_CONTROL_HPP
#define GIMBAL_CONTROL_HPP

class Gimbal {

private:
	float angle;
	int init_done = 0;
	int pin = 1;
	float deg2pwm = 0.27778; // 5;
	int pwm_middle = 77;
	int max_angle = 45;

public:

	int init();
	int set_angle(float desired_angle);
	float get_angle();

};

#endif