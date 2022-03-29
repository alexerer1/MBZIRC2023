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

#include "balloonPopper.hpp"

// Reads a variable from the jrk.
// The 'command' argument must be one of the two-byte variable-reading
// commands documented in the "Variable Reading Commands" section of
// the jrk user's guide.
int jrkGetVariable(int fd, unsigned char command)
{
	if(write(fd, &command, 1) == -1)
	{
		perror("error writing");
		return -1;
	}

	unsigned char response[2];
	
	if(read(fd,response,2) != 2)
	{
		perror("error reading");
		return -1;
	}
	return response[0] + 256*response[1];
}

// Gets the value of the jrk's Feedback variable (0-4095).
int jrkGetFeedback(int fd)
{
	return jrkGetVariable(fd, 0xA5);
}
// Gets the value of the jrk's Target variable (0-4095).
int jrkGetTarget(int fd)
{
	return jrkGetVariable(fd, 0xA3);
}
// Sets the jrk's Target variable (0-4095).
int jrkSetTarget(int fd, unsigned short target)
{
	unsigned char command[] = {0xC0 + (target & 0x1F), (target >> 5) & 0x7F};
	if (write(fd, command, sizeof(command)) == -1)
	{
		perror("error writing");
		return 0;
	}
	return 1;
}

int balloonPopper::start_motor()
{
	int newTarget = 2200;
	if( jrkSetTarget(fd, newTarget) ) {
		status = 1;
		return 1;
	}
	return 0;
}

int balloonPopper::get_status() {return status;}

int balloonPopper::stop_motor()
{
	int newTarget = 2047;
	if( jrkSetTarget(fd, newTarget) ) {
		status = 0;
		return 1;
	}
	return 0;
}

int balloonPopper::close_device()
{
	close(fd);
}

int balloonPopper::init()
{
	// Open the Jrk's virtual COM port.
	// const char * device = "\\\\.\\USBSER000"; // Windows, "\\\\.\\COM6" also works
	const char * device = "/dev/ttyACM0"; // Linux
	//const char * device = "/dev/cu.usbmodem00000041"; // Mac OS X
	fd = open(device, O_RDWR | O_NOCTTY);
	if (fd == -1)
	{
		perror(device);
		return 0;
	}
	
	#ifndef _WIN32
		struct termios options;
		tcgetattr(fd, &options);
		options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
		options.c_oflag &= ~(ONLCR | OCRNL);
		tcsetattr(fd, TCSANOW, &options);
	#endif
	
	int newTarget = 2047;

	jrkSetTarget(fd, newTarget);
	status = 0;

	//close(fd);

	return 1;
}