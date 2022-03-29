#ifndef BALLOONPOPPER_HPP
#define BALLOONPOPPER_HPP

class balloonPopper {

public:
	int init();
	int start_motor();
	int stop_motor();
	int close_device();
	int get_status();

private:
	int fd;
	int status;

};

#endif