#ifndef PICKUPARM_HPP
#define PICKUPARM_HPP

class PickupArm {

public:
    void init_pins();
    
    bool switch_pressed();
    
    void magnet_on();
    
    void magnet_off();
    
    int get_magnet_status();
    
private:
   	int switch_pin = 0;
	int magnet_pin = 2;
    
    int magnet_status = 0;
};

#endif
