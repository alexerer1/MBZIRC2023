#include "Rangefinder.hpp"
#include <iostream>
#include <bitset>
#include <cmath>

bool Rangefinder::is_enabled(){
    return enabled;
}

/************************************************
*
*	Open the I2C port 
*	id = 0x30 for TeraRanger One
*	id = 0x31 for TeraRanger Evo60
*
*************************************************/
bool Rangefinder::initializeI2C(int id){
    
		addr = id;          //<<<<<The I2C address of the Terraranger

		//----- OPEN THE I2C BUS -----
		char *filename = (char*)"/dev/i2c-1";
		if ((file_i2c = open(filename, O_RDWR)) < 0)
		{
			std::cout << "Failed to open the i2c bus" << std::endl;
			enabled = 0;
		} 
		else if (ioctl(file_i2c, I2C_SLAVE, addr) < 0)
		{
			std::cout << "Failed to acquire bus access and/or talk to slave.\n" << std::endl;
			enabled = 0;
		}
		else {
			std::cout << "I2C device linked" << std::endl;
			enabled = 1;
		}

		return enabled;
}


/************************************************
*
*	Set the position offset of the rangefidner on 
*	the drone
*
*************************************************/
void Rangefinder::set_offset(float x, float y)
{
	x_offset = x;
	y_offset = y;
}

void Rangefinder::set_in_use(bool val){
    in_use = val;
}

int Rangefinder::get_address()
{
	return addr;
}

/************************************************
*
*	Performs a who am i test on the I2C address initialized
*
*************************************************/
void Rangefinder::who_am_I(){
        
		int length=1; //<<< Number of bytes to read
		unsigned char buffer[10] = {0};
	
		if( enabled ){
			//read() returns the number of bytes actually read, if it doesn't
			//match then an error occurred (e.g. no response from the device)
			unsigned char cmd1[1];
			cmd1[0] = 0x01;
			if( write(file_i2c, cmd1, length) != length) std::cout << "FAIL\n";
			usleep(500);
            int resp = read(file_i2c, buffer, length);
            if( resp == length )
            	printf("I am = 0x%X\n", buffer[0]);
            else
            	printf("Wrong nubmer of bytes returned (%d)\n", resp);
        }
}


/************************************************
*
*	Read the height from the rangefinder
*	Returns 0 if the rangefidner is set to not in use
*	This function doesn't account for roll and pitch
*	The value is not filtered with the height filter
*
*************************************************/
float Rangefinder::getHeight( ){
        if( !in_use) return 0;
        
		int length=3; //<<< Number of bytes to read
		unsigned char buffer[10] = {0};
		unsigned long distance;
		float distanceFloat = 0;
		// Check if initialized
		if( enabled ){
			//read() returns the number of bytes actually read, if it doesn't
			//match then an error occurred (e.g. no response from the device)
			// TeraRanger Evo60
			if( addr == 0x31 ){
				unsigned char cmd1[1];
				cmd1[0] = 0x00;
				if( write(file_i2c, cmd1, 1) != 1) std::cout << "FAIL\n";
				usleep(500);
			}
            int resp = read(file_i2c, buffer, length);
            if( resp != length )
// 			if (read(file_i2c, buffer, length) != length)		
			{
				std::cout<<"Lost connection to Terraranger. Using Barometer instead : " << resp << std::endl;
// 				enabled = 0;
			}
			else
			{
				// std::cout << (char)buffer[0] << "|" << (char)buffer[1] << "|" <<  (char)buffer[2] << "|" << (char)0xFF << std::endl;
				// printf("0x%X 0x%X 0x%X\n", buffer[0], buffer[1], buffer[2]);
				// Perform the conversion from 3 I2C bytes to a float
				std::bitset<8> x(buffer[0]);
				distance=x.to_ulong();
				distance=distance<<8;
				x=buffer[1];
				distance=distance|x.to_ulong();
				distanceFloat = ((float)distance)/1000;
			}
		}
		else {
			std::cout << "Rangefinder not connected, run initializeI2C()\n";
		}

	return distanceFloat;
}


/************************************************
*
*	Return the height even if in_use is not set
*	Same as the function above
*
*************************************************/
float Rangefinder::overrideInUseHeight( ){
        
		int length=3; //<<< Number of bytes to read
		unsigned char buffer[10] = {0};
		unsigned long distance;
		float distanceFloat = 0;
		if( enabled ){
			//read() returns the number of bytes actually read, if it doesn't
			//match then an error occurred (e.g. no response from the device)
			if( addr == 0x31 ){
				unsigned char cmd1[1];
				cmd1[0] = 0x00;
				if( write(file_i2c, cmd1, 1) != 1) std::cout << "FAIL\n";
				usleep(500);
			}

            int resp = read(file_i2c, buffer, length);
            if( resp != length )
// 			if (read(file_i2c, buffer, length) != length)		
			{
				std::cout<<"Lost connection to Terraranger. Using Barometer instead : " << resp << std::endl;
// 				enabled = 0;
			}
			else
			{
				std::bitset<8> x(buffer[0]);
				distance=x.to_ulong();
				distance=distance<<8;
				x=buffer[1];
				distance=distance|x.to_ulong();
				distanceFloat = ((float)distance)/1000;
			}
		} else {
			std::cout << "Rangefinder not connected, run initializeI2C()\n";
		}

		if( use_h_filter ) distanceFloat = filter_h( distanceFloat );

		return distanceFloat;
 }

float Rangefinder::getUnfilteredHeight()
{
	return unfiltered_h;
}


/************************************************
*
*	Filters the measurement to check if it lies 
*	within the expected height
*
*************************************************/
float Rangefinder::filter_h( float measurement )
{
	unfiltered_h = measurement;
	float difference = measurement - expected_h;
	if( difference < -1 ) return 0; //-0.4
	else if( difference > 1 ) return 0; // 3
	else return measurement;

	// if( difference > 0.5)
	// {
		// return 0;
	// } else {
		// return measurement;
	// }	
}


/************************************************
*
*	Sets the filter height and starts the filtering
*
*************************************************/
void Rangefinder::set_filter_h( float h )
{
	expected_h = h;
	use_h_filter = true;
}


/************************************************
*
*	Disable the filter
*
*************************************************/
void Rangefinder::unset_filter()
{
	use_h_filter = false;
}
 

/************************************************
*
*	Returns the height but compensates for the roll and pitch of the drone
*	so the height becomes the straight line down (gravity direction)
*
*************************************************/
float Rangefinder::getHeightRP( float roll, float pitch ){
    if( !in_use) return 0;
	
    int length=3; //<<< Number of bytes to read
	unsigned char buffer[10] = {0};
	unsigned long distance;
	float distanceFloat = 0;
	if( enabled ){
		//read() returns the number of bytes actually read, if it doesn't
		//match then an error occurred (e.g. no response from the device)
		if( addr == 0x31 ){
			unsigned char cmd1[1];
			cmd1[0] = 0x00;
			if( write(file_i2c, cmd1, 1) != 1) std::cout << "FAIL\n";
			usleep(500);
		}

        int resp = read(file_i2c, buffer, length);
        if( resp != length )
// 			if (read(file_i2c, buffer, length) != length)		
		{
            std::cout<<"Lost connection to Terraranger. Using Barometer instead : " << resp << std::endl;
// 				enabled = 0;
		}
		else
		{
			std::bitset<8> x(buffer[0]);
			distance=x.to_ulong();
			distance=distance<<8;
			x=buffer[1];
			distance=distance|x.to_ulong();
			distanceFloat = ((float)distance)/1000;
		}
	} else {
		std::cout << "Rangefinder not connected, run initializeI2C()\n";
	}
	
	// Calculate the correct height
	distanceFloat *= cos(roll)*cos(pitch) - x_offset*sin(pitch) - y_offset*sin(roll);
	
	// Filter the value
	if( use_h_filter ) distanceFloat = filter_h( distanceFloat );

	return distanceFloat;
}
