#include "loop-heartbeat.hpp"
#include <time.h>
#include <iostream>

char HeartbeatTimer::start_hb(int freq)
{
    beat_length = (int) (1.0/( (float) freq) * 10000.0);
    std::cout << (int) (1.0/( (float) freq) * 10000.0) << std::endl;
    std::cout << (1.0/( (float) freq) * 1000.0) << std::endl;
    heartbeat_running = 1;
    if( StartInternalThread() ){
        std::cout << "Starting thread\n";
        return 1;
    }
    return 0;
}

void HeartbeatTimer::InternalThreadEntry()
{
    std::cout << "InternalThreadEntry\n";
    heartbeat_timer();
}


char HeartbeatTimer::get_hb_flag()
{
    pthread_mutex_lock(&flag_mutex);
    char flag = heartbeat_flag;
    heartbeat_flag = 0;
    //std::cout << "flag read" << (int) flag << std::endl;
    pthread_mutex_unlock(&flag_mutex);
    return flag;
}

void HeartbeatTimer::stop_hb()
{
    std::cout << "Shutting down thread\n";
    heartbeat_running = 0;
    WaitForInternalThreadToExit();
    std::cout << "Thread shut down\n";
}


//*****************************************************
//*****************************************************
//********** DELAY FOR # uS WITHOUT SLEEPING **********
//*****************************************************
//*****************************************************
//Using delayMicroseconds lets the linux scheduler decide to jump to another process.  Using this function avoids letting the
//scheduler know we are pausing and provides much faster operation if you are needing to use lots of delays.
void HeartbeatTimer::DelayMicrosecondsNoSleep (int delay_us)
{
    long int start_time;
    long int time_difference;
    struct timespec gettime_now;
    
    clock_gettime(CLOCK_REALTIME, &gettime_now);
    start_time = gettime_now.tv_nsec;       //Get nS value
    
    while (1)
    {
        clock_gettime(CLOCK_REALTIME, &gettime_now);
        time_difference = gettime_now.tv_nsec - start_time;
        
        if (time_difference < 0)
            time_difference += 1000000000;              //(Rolls over every 1 second)
        if (time_difference > (delay_us * 1000))    //Delay for # nS
            break;
    }
}

void HeartbeatTimer::heartbeat_timer()
{
    long int last_heartbeat;
    long int heartbeat_difference;
    struct timespec gettime_now; 
    int hb_timer = 0;


	//SETUP HEARTBEAT TIMER
	clock_gettime(CLOCK_REALTIME, &gettime_now);
	last_heartbeat = gettime_now.tv_nsec;

	//---------------------------
	//---------------------------
	//----- HEARTBEAT TIMER -----
	//---------------------------
	//---------------------------
    while(heartbeat_running){
        clock_gettime(CLOCK_REALTIME, &gettime_now);
        heartbeat_difference = gettime_now.tv_nsec - last_heartbeat;		//Get nS value
        if (heartbeat_difference < 0)
            heartbeat_difference += 1000000000;				//(Rolls over every 1 second)
        if (heartbeat_difference > 100000)					//<<< Heartbeat every 0.1mS
        {
            last_heartbeat += 100000;						//<<< Heartbeat every 0.1mS
            if (last_heartbeat > 1000000000)				//(Rolls over every 1 second)
                last_heartbeat -= 1000000000;
            
            //---------------------------
            //-----  HEARTBEAT -----
            //---------------------------
            hb_timer++;
            if (hb_timer == beat_length)
            {
                hb_timer = 0;
                pthread_mutex_lock(&flag_mutex);
                heartbeat_flag = 1;
                pthread_mutex_unlock(&flag_mutex);
//                 std::cout << " Flag Set \n";
            }
        }
	}
}
