//TestFlight

/*! @file flight_control_sample.cpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  Flight Control API usage in a Linux environment.
 *  Provides a number of helpful additions to core API calls,
 *  especially for position control, attitude control, takeoff,
 *  landing.
 *
 *  @Copyright (c) 2016-2017 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

// OpenCV libs
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/aruco.hpp"

// Raspicam
#include <raspicam/raspicam_cv.h> 


 #include "GeneralCommands.hpp"

/*! Global constants for ULMS-server
!*/
double laserpar[10];
char buf[256];
int len;
struct xml_in *xmllaser;
componentservertype lmssrv;
/*! For keeping track of the global desired Yaw
!*/
double desiredGlobalYaw=0;

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;
/*! Constanst for the Terraranger 
!*/
	int file_i2c;
	float32_t lostConnectionToTerraRanger=0;

/*! Monitored Takeoff (Blocking API call). Return status as well as ack.
    This version of takeoff makes sure your aircraft actually took off
    and only returns when takeoff is complete.
    Use unless you want to do other stuff during takeoff - this will block
    the main thread.
!*/


// Helper Functions

/*! Very simple calculation of local NED offset between two pairs of GPS
/coordinates.
    Accurate when distances are small.
!*/
void
localOffsetFromGpsOffset(Vehicle* vehicle, Telemetry::Vector3f& deltaNed,
                         void* target, void* origin)
{
  Telemetry::GlobalPosition* broadcastTarget;
  Telemetry::GlobalPosition* broadcastOrigin;
  double                     deltaLon;
  double                     deltaLat;

 
    broadcastTarget = (Telemetry::GlobalPosition*)target;
    broadcastOrigin = (Telemetry::GlobalPosition*)origin;
    deltaLon        = broadcastTarget->longitude - broadcastOrigin->longitude;
    deltaLat        = broadcastTarget->latitude - broadcastOrigin->latitude;
    deltaNed.x      = deltaLat * C_EARTH;
    deltaNed.y      = deltaLon * C_EARTH * cos(broadcastTarget->latitude);
    deltaNed.z      = broadcastTarget->altitude - broadcastOrigin->altitude;
}

Telemetry::Vector3f
toEulerAngle(void* quaternionData)
{
  Telemetry::Vector3f    ans;
  Telemetry::Quaternion* quaternion = (Telemetry::Quaternion*)quaternionData;

  double q2sqr = quaternion->q2 * quaternion->q2;
  double t0    = -2.0 * (q2sqr + quaternion->q3 * quaternion->q3) + 1.0;
  double t1 =
    +2.0 * (quaternion->q1 * quaternion->q2 + quaternion->q0 * quaternion->q3);
  double t2 =
    -2.0 * (quaternion->q1 * quaternion->q3 - quaternion->q0 * quaternion->q2);
  double t3 =
    +2.0 * (quaternion->q2 * quaternion->q3 + quaternion->q0 * quaternion->q1);
  double t4 = -2.0 * (quaternion->q1 * quaternion->q1 + q2sqr) + 1.0;

  t2 = (t2 > 1.0) ? 1.0 : t2;
  t2 = (t2 < -1.0) ? -1.0 : t2;

  ans.x = asin(t2);
  ans.y = atan2(t3, t4);
  ans.z = atan2(t1, t0);

  return ans;
}


std::vector< std::vector<double> >  fromPolarToRect(std::vector< std::vector<double> > &lidarData)
{
	int size=lidarData.size();
	double distance, angle;
	std::vector< std::vector<double> > rectData;
	rectData.resize(size);
	for(int i=0; i<size;i++){
		distance=lidarData[i][0];
		angle=lidarData[i][1];
		
		rectData[i][0]=distance*cos(angle);
		rectData[i][1]=distance*sin(angle);
	}
	return rectData;
}

Telemetry::Vector3f transformCoordinates(DJI::OSDK::Vehicle *vehicle, double angleInRad, float x, float y)
{
	Telemetry::Vector3f newCoordinates;
	double angleInDeg;

	angleInDeg 	= angleInRad/DEG2RAD;

	newCoordinates.x=cos(angleInRad)*x + sin(angleInRad)*y;
	newCoordinates.y=-sin(angleInRad)*x + cos(angleInRad)*y;
	//newCoordinates.z=getHeight(vehicle);
	
	//std::cout<< "i :x " << x << ", i :y " << y << ", V " << angleInDeg << ", outputX= " << newCoordinates.x << ", outputY= " << newCoordinates.y <<std::endl;
	
  return newCoordinates;
}

/*! Initializing the I2C module
!*/
void initializeI2C(char * enabled){
		int addr = 0x30;          //<<<<<The I2C address of the Terraranger
		
		//----- OPEN THE I2C BUS -----
		char *filename = (char*)"/dev/i2c-1";
		if ((file_i2c = open(filename, O_RDWR)) < 0)
		{
			std::cout << "Failed to open the i2c bus" << std::endl;
			*enabled = 0;
		} 
		else if (ioctl(file_i2c, I2C_SLAVE, addr) < 0)
		{
			std::cout << "Failed to acquire bus access and/or talk to slave.\n" << std::endl;
			*enabled = 0;
		}
		else {
			std::cout << "I2C device linked" << std::endl;
			*enabled = 1;
		}
}


/*! Checks if there is still connection with the Terraranger. If not then the function returns height data measured by the Matrice
!*/
 float32_t getHeight( char * enabled ){
		int length=3; //<<< Number of bytes to read
		unsigned char buffer[10] = {0};
		unsigned long distance;
		float distanceFloat;
		if( &enabled ){
			//read() returns the number of bytes actually read, if it doesn't
			//match then an error occurred (e.g. no response from the device)
			if (read(file_i2c, buffer, length) != length)		
			{
//				std::cout<<"Lost connection to Terraranger. Using Barometer instead" << std::endl;
				if(lostConnectionToTerraRanger>=20){
					*enabled = 0;
				}
			} else
			{
				lostConnectionToTerraRanger=0;
				std::bitset<8> x(buffer[0]);
				distance=x.to_ulong();
				distance=distance<<8;
				x=buffer[1];
				distance=distance|x.to_ulong();
				distanceFloat = ((float)distance)/1000;
			}
		} else {
			distanceFloat=0;
		}
		return distanceFloat;
 }
 
Telemetry::Vector3f flownDistance(DJI::OSDK::Vehicle* vehicle, DJI::OSDK::Telemetry::GlobalPosition position, DJI::OSDK::Telemetry::GlobalPosition origin){

	Telemetry::Vector3f distance;
	
	//! Get Euler angle
	// Quaternion retrieved via broadcast
	Telemetry::Quaternion broadcastQ;
	
	double yawInRad;
	
	broadcastQ 	= vehicle->broadcast->getQuaternion();
	yawInRad   	= toEulerAngle((static_cast<void*>(&broadcastQ))).z;
	
	localOffsetFromGpsOffset(vehicle, distance ,static_cast<void*>(&position), static_cast<void*>(&origin));
	distance = transformCoordinates(vehicle, yawInRad, distance.x, distance.y);

	return distance;
 }

void getLidarData(){

	laserpar[0]=0;
	while(laserpar[0]==0)
	{
		while ( (xml_in_fd(xmllaser,lmssrv.sockfd) >0))
		//printf("xmlprica");
		xml_proca(xmllaser);
		usleep(100000);
	}
}


void xml_proca(struct xml_in *x){

 while(1){
 	switch (xml_in_nibble(x)) {
    	case XML_IN_NONE:
      	return;
    case XML_IN_TAG_START:
    	//printf("Start tage\n");
	    #if (0)
	    {int i;
	    double a;
	      printf("start tag: %s, %d attributes\n", x->a, x->n);
	      for(i=0;i<x->n;i++){
	        printf("  %s    %s  \n",x->attr[i].name,x->attr[i].value);
	      }
	    }
	    #endif
	     if (strcmp("laser",x->a)==0){
	        int i,ix;
	        for (i=0;i< x->n;i++){
	            ix=atoi(x->attr[i].name+1);
		 	  	if (ix >-1 && ix < 10)
		      		laserpar[ix]=atof(x->attr[i].value);
	        }   
	    }
	   
	    break;
    case XML_IN_TAG_END:
      //printf("end tag: %s\n", x->a);
      break;
    case XML_IN_TEXT:
      //printf("text: %d bytes\n  \"", x->n);
      //fwrite(x->a, 1, x->n, stdout);
      //printf("\"\n");
      break;
    }
  } 
}  


 void createLogFile(){
	 char str[80];
	 char buf[16]; // need a buffer for that
	 int i=1;
	 while (true){
		 sprintf(buf,"%d",i);
		const char* p = buf;
		 strcpy (str,"output");
		 strcat (str,p);
		 strcat (str,".txt");
		 std::ifstream fileExist(str);
		if(!fileExist)
		{
			break;
		}
		i++;
	 }
	 std::freopen( str, "w", stderr );
	 std::cerr << "%Columns [1;4] = position: Lon, Lat, z, yawInDeg & Colums 5,  6, 7 and 8= obstacle location, turn angle, line Y-intersept, number of elemetns & 9-14 are start+end coordinated af object " << std::endl;

 }
 
 void logData( DJI::OSDK::Vehicle *vehicle){
	 std::cerr.precision(17);
	 char str[80];
	 char buf[16]; // need a buffer for that
	 int i=100;
	 while (i>0){
		 sprintf(buf,"%d",i);
		const char* p = buf;
		 strcpy (str,"output");
		 strcat (str,p);
		 strcat (str,".txt");
		 std::ifstream fileExist(str);
		if(fileExist)
		{
			break;
		}
		i=i-1;
	 }
	 
	  // Log position
	 Telemetry::GlobalPosition globalPosition = vehicle->broadcast->getGlobalPosition();
	//globalPosition.height = getHeight(vehicle);
	 // Log Angle
	 Telemetry::Quaternion broadcastQ;
	 double yawInDeg;
	broadcastQ 	= vehicle->broadcast->getQuaternion();
	yawInDeg   	= toEulerAngle((static_cast<void*>(&broadcastQ))).z/DEG2RAD;
	std::cerr << globalPosition.longitude <<", " << globalPosition.latitude << ", " <<   globalPosition.altitude<<", " <<   yawInDeg<<", " <<laserpar[1]<<", " << laserpar[2]<<", " << laserpar[3]<<", " << laserpar[4] <<", " << laserpar[5] <<", " << laserpar[6]<< ", " << laserpar[7] <<", " << laserpar[8]<<std::endl;
 }
 
//#################################################################################################
//#################################################################################################
//#################################################################################################
//#################################################################################################

/***************************************
*
* Open raspicam camera and set resolution
*
****************************************/
int init_cam(raspicam::RaspiCam_Cv *cap)
{
	cap->set(CV_CAP_PROP_FRAME_WIDTH, 640/2); // 320
	cap->set(CV_CAP_PROP_FRAME_HEIGHT, 480/2); // 240
	printf("Opening camera ...");
	//printf("\033[8;0HOpening camera ...");
	if(!cap->open()) { printf("cam not opened!\n"); return 0; }
	//namedWindow("mywin", CV_WINDOW_AUTOSIZE);
	printf("cam is opened!\n");
	return 1;
}