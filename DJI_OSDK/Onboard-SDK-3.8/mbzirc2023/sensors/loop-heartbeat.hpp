/*
 * 
 *  Credits: https://raspberry-projects.com/pi/programming-in-c/timing/clock_gettime-for-acurate-timing
 * 
 */

#ifndef LOOP_HEARTBEAT_HPP
#define LOOP_HEARTBEAT_HPP

#include <pthread.h>
#include "InternalThread.hpp"


class HeartbeatTimer : public InternalThread{
    
    public:
        //HeartbeatTimer() {};
        //virtual ~HeartbeatTimer() {};
        
        char start_hb(int freq);
        void stop_hb();
        void DelayMicrosecondsNoSleep (int delay_us);
        void heartbeat_timer();
        char get_hb_flag();
    
        void InternalThreadEntry();
        
        void set_freq( int freq );
        
    private:
        char heartbeat_running = 0;
        char heartbeat_flag = 0;
        int beat_length = 10;
        pthread_mutex_t flag_mutex = PTHREAD_MUTEX_INITIALIZER;

};



#endif
