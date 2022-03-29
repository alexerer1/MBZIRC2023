#ifndef LASERSCANNER_HPP
#define LASERSCANNER_HPP

// #include "componentserver.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
// #include "xmlio.h"
#include "type_defines.hpp"

class LaserScanner {

private:
    double laserpar[10];
    char buf[256];
    int len;
    // struct xml_in *xmllaser;
    // componentservertype lmssrv;

    // void serverconnect(componentservertype *s);
    void getData();
    // void xml_proca(struct xml_in *x);
    
    int num_pose = 0;

public:
    bool init_laser();
    void test_zoneobst();
    double* get_laserdata();
    pose_t getPosition(float pitch);
};

#endif
