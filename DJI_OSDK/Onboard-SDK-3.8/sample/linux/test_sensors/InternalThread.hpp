#ifndef INTERNAL_THREAD_HPP
#define INTERNAL_THREAD_HPP

#include <pthread.h>

/*
 * 
 * https://stackoverflow.com/questions/1151582/pthread-function-from-a-class
 * 
 */
class InternalThread
{
    public:
        InternalThread() {/* empty */}
        virtual ~InternalThread() {/* empty */}

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
        static void * InternalThreadEntryFunc(void * This) {((InternalThread *)This)->InternalThreadEntry(); return NULL;}

        pthread_t _thread;
};

#endif
