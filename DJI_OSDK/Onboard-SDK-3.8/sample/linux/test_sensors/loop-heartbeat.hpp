/*
 * 
 *  Credits: https://raspberry-projects.com/pi/programming-in-c/timing/clock_gettime-for-acurate-timing
 * 
 */

#ifndef LOOP_HEARTBEAT_HPP
#define LOOP_HEARTBEAT_HPP

#include <pthread.h>
#include "InternalThread.hpp"

/*
 * 
 * https://stackoverflow.com/questions/1151582/pthread-function-from-a-class
 * 
 */
class MyThreadClass
{
    public:
        MyThreadClass() {/* empty */}
        virtual ~MyThreadClass() {/* empty */}

        /** Returns true if the thread was successfully started, false if there was an error starting the thread */
        bool StartInternalThread()
        {
            return (pthread_create(&_thread, NULL, InternalThreadEntryFunc, this) == 0);
        }

        /** Will not return until the internal thread has exited. */
        void WaitForInternalThreadToExit()
        {
            (void) pthread_join(_thread, NULL);
        }

    protected:
        /** Implement this method in your subclass with the code you want your thread to run. */
        virtual void InternalThreadEntry() = 0;

    private:
        static void * InternalThreadEntryFunc(void * This) {((MyThreadClass *)This)->InternalThreadEntry(); return NULL;}

        pthread_t _thread;
};

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
        
    private:
        char heartbeat_running = 0;
        char heartbeat_flag = 0;
        int beat_length = 10;
        pthread_mutex_t flag_mutex = PTHREAD_MUTEX_INITIALIZER;

};



#endif
