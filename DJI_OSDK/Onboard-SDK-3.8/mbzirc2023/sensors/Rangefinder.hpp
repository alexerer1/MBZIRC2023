#ifndef RANGEFINDER_HPP
#define RANGEFINDER_HPP

#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>				//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port

class Rangefinder {

private:
    bool enabled;
    bool in_use = true;
    int file_i2c;
    int addr = 0x30;
    
    float expected_h = 0;

    float unfiltered_h = 0;

    float x_offset = 0;
    float y_offset = 0;

public:
    bool use_h_filter = 0;

    int get_address();

    bool is_enabled();
    // bool use_h_filter = 0;
    void set_in_use(bool val);
    
    void set_offset(float x, float y);

    float filter_h( float measurement );

    void set_filter_h( float h );
    void unset_filter();

    /*! Initializing the Terraranger which is comunicating using I2C protocal
    !*/
    bool initializeI2C(int id);

    /*! If connected to the Terraranger then getHeight will return the Terraranger height reading
        If not then it will return the height measurement from the integrated Matrice sensors
    !*/

    void who_am_I();
    float getHeight();
    float getHeightRP( float roll, float pitch );
    float getUnfilteredHeight();
    float overrideInUseHeight( );

};

#endif
