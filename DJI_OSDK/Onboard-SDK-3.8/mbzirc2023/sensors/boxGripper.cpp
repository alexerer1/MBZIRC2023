// Uses POSIX functions to send and receive data from a jrk.
// NOTE: The jrk's input mode must be "Serial".
// NOTE: The jrk's serial mode must be set to "USB Dual Port".
// NOTE: You must change the 'const char * device' line below.
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#ifdef _WIN32
#define O_NOCTTY 0
#else
#include <termios.h>
#endif

#include "boxGripper.hpp"

pose_t BoxGripper::image_to_position(pose_t image_position, float height)
{

	pose_t new_reference;
	// Try to do logic here
	new_reference.x = (height / 1000) * image_position.x / 10;
	new_reference.y = (height / 1000) * image_position.y / 10;
	new_reference.th = image_position.th;
	return new_reference;
}

int BoxGripper::init()
{
	// close(fd);

	return 1;
}