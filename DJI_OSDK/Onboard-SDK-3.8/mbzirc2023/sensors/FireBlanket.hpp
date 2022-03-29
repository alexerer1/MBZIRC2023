#ifndef FIRE_BLANKET_HPP
#define FIRE_BLANKET_HPP

class FireBlanket {

public:
    void init_pins();
    
    void release_blanket();

    void drop_blanket();

    void turn_on_magnets();
    
private:
   	int blanket_pin = 4;
	int release_pin = 5;
    
};

#endif
