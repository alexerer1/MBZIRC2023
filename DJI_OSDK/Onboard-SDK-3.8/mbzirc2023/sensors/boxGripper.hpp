#ifndef BALLOONPOPPER_HPP
#define BALLOONPOPPER_HPP
#include "type_defines.hpp"

class BoxGripper
{

public:
	int init();
	pose_t image_to_position(pose_t, float);

private:
	int fd;
	int status;
};

#endif