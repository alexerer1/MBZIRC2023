//
//  camera_localizer.cpp
//  
//
//  Created by Ananda Nielsen on 29/04/2019
//  Copyright © 2019 Ananda Nielsen. All rights reserved.
//

#include "camera_localizer.hpp"

// Standard libraries
#include <pthread.h>
#include <semaphore.h>
#include <math.h>
#include <stdio.h> 
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <ctime>
#include <vector>

#include <chrono>

// OpenCV libs
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/aruco.hpp"

// Raspicam
#include <raspicam/raspicam_cv.h> 

// Custom AUTNAV libraries
#include "defines.h" 
#include "rotations.h"
#include "aruco_pose.h"
#include "aruco_map.h"

using namespace std;
using namespace cv;

//##############################################################################
//##############################################################################
//##############################################################################
//##############################################################################

// External global variables (autnav.c)
extern int px4fd4;

extern int camera_on;
extern int marker_visible;

extern double pitch_for_aruco;
extern double roll_for_aruco;
extern double yaw_for_aruco;
extern double height;
extern struct pose_t aruco_pos;

extern sem_t aruco_mutex;

// Mission externs (aruco_missions.cpp)
extern char start_video_file;
extern int ff_loiter;
extern char show_image;

//

extern float simple_dx;
extern float simple_dy;
extern float hom_dx;
extern float hom_dy;
extern float hom_dz;
extern float sigma_1;
extern float sigma_2;
extern float sigma_3;

extern float true_aruco_x;
extern float true_aruco_y;

extern int cam_time;
extern int grab_time;
extern int aruco_time;
extern int flow_time;

extern int n_good_features;
//


// Locally global variables
struct pose_t position_raw;
ar_map global_map;

// Flow variables
cv::Mat old_frame;
cv::Mat new_frame;
cv::Mat border_mask;
cv::Mat show_frame;

std::vector<Point2f> tracked_features[2];
char needInit = 1;

float old_roll = 0;
float old_pitch = 0;
float old_yaw = 0;

float new_roll = 0;
float new_pitch = 0;
float new_yaw = 0;

float h_raw = 0;

const int z_cam = 257;

char aruco_lost = 0;
int ramp_counter = 0;

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
	cap->set(CV_CAP_PROP_FRAME_WIDTH, 320);
	cap->set(CV_CAP_PROP_FRAME_HEIGHT, 240);
	printf("Opening camera ...");
	//printf("\033[8;0HOpening camera ...");
	if(!cap->open()) { printf("cam not opened!\n"); return 0; }
	//namedWindow("mywin", CV_WINDOW_AUTOSIZE);
	printf("cam is opened!\n");
	return 1;
}

//#################################################################################################
//#################################################################################################
//#################################################################################################
//#################################################################################################

/*********************************
*
* Fucntion to be called from the 
* camera thread to run the camera
* and ArUco / flow pose estimation. Runs
* until camera_on is changed
*
**********************************/
extern "C" int start_camera_localizer(const char * map_file){
  raspicam::RaspiCam_Cv cap;

  cv::aruco::Dictionary dictionary=aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
  cv::Mat cameraMatrix=(Mat_<double>(3,3)<<257,0,160.0,0,257.0,120,0,0,1);
  cv::Mat distCoeffs=(Mat_<double>(1,5)<<0.15490157,0.00111697,-0.00344782,0.00492199,-0.80341838);

  std::vector<std::vector<cv::Point2f> > corners;
  std::vector<int> markerIds;

  cv::VideoWriter video("outcpp.avi",CV_FOURCC('M','J','P','G'),16, cv::Size(320,240));

  char calcFlow = 0;

  if( !init_cam(&cap) ) return 0;
  else{
    printf("Camera on\n");
  }

  if(global_map.decode_xml(map_file)) printf("Map: %s decoded\n\n",map_file);
  else return 0;

  border_mask = cv::Mat::zeros(cv::Size(320,240),CV_8UC1);
  for( int i = 19; i < 301; i++ ){
  	for( int j = 19; j < 221; j++ ){
  		border_mask.at<char>(j,i) = 1;
  	}
  }

  while(camera_on){

    auto start = std::chrono::high_resolution_clock::now();

    cap.grab();
    cap.retrieve(new_frame);
    new_roll = roll_for_aruco;
    new_pitch = pitch_for_aruco;
    new_yaw = yaw_for_aruco;
    h_raw = height;
    if( show_image || start_video_file ){
      new_frame.copyTo(show_frame);
    }

    auto stop = std::chrono::high_resolution_clock::now();
    auto dura = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

    grab_time = (int) dura.count();

    auto start2 = std::chrono::high_resolution_clock::now();

    position_raw = find_pos( &new_frame, roll_for_aruco, pitch_for_aruco );

    stop = std::chrono::high_resolution_clock::now();
    dura = std::chrono::duration_cast<std::chrono::microseconds>(stop - start2);

    aruco_time = (int) dura.count();

    if(position_raw.valid && (ff_loiter==0)){

      if( aruco_lost ){

          sem_wait(&aruco_mutex);
          
          aruco_pos.num_pose++;
          
          aruco_pos.x += (position_raw.x - aruco_pos.x)/(float)ramp_counter;;
          aruco_pos.y += (position_raw.y - aruco_pos.y)/(float)ramp_counter;;

          aruco_pos.z = position_raw.z;
          aruco_pos.th = position_raw.th;

          aruco_pos.valid = 3;

          sem_post(&aruco_mutex);  

          ramp_counter--;

          if( ramp_counter == 0 ) {
            aruco_lost = 0;
          }
      }
      else{
        sem_wait(&aruco_mutex);
        
        aruco_pos.num_pose++;
        aruco_pos.x = position_raw.x;
        aruco_pos.y = position_raw.y;
        aruco_pos.z = position_raw.z;
        aruco_pos.th = position_raw.th;

        aruco_pos.valid = 1;

        sem_post(&aruco_mutex);            
        
        aruco_lost = 0;
      }

      calcFlow = 0;
    }
    else{
      calcFlow = 1;
      aruco_pos.valid = 0;
      aruco_lost = 1;
      ramp_counter = 10;
    }

    true_aruco_x = position_raw.x;
    true_aruco_y = position_raw.y;

    start2 = std::chrono::high_resolution_clock::now();

    cv::cvtColor(new_frame,new_frame,cv::COLOR_BGR2GRAY);

    flow_estimation( calcFlow );

    new_frame.copyTo(old_frame);

    stop = std::chrono::high_resolution_clock::now();
    dura = std::chrono::duration_cast<std::chrono::microseconds>(stop - start2);

    flow_time = (int) dura.count();  

    stop = std::chrono::high_resolution_clock::now();
    dura = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

    cam_time = (int) dura.count();  

    if( start_video_file ) {
      tcflush(px4fd4,TCIOFLUSH); 
      video.write(show_frame);
      aruco_pos.valid = 1;
    }

    if(show_image && show_frame.total() != 0 ){

      // Draw all the detected markers (known and unknown)
      aruco::detectMarkers(show_frame, dictionary, corners, markerIds);
      aruco::drawDetectedMarkers(show_frame, corners, markerIds);

      //Draw red dot in image center and marker borders
      cv::circle(show_frame,Point(160,120),1,Scalar(0,0,255),5,8,0);  

      imshow("Drone camera view",show_frame);
      waitKey(1);
      //usleep(100000);
    }

  }

  video.release();
  return 0;
}



//#################################################################################################
//#################################################################################################
//#################################################################################################
//#################################################################################################

int flow_estimation(char calcFlow) {

  // Optical flow estimation parameters
  cv::TermCriteria  termcrit(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,15,0.05);
  cv::Size subPixWinSize(5,5), winsize(10,10);
  const int MAX_FEATURES = 100;
  std::vector<uchar> status;
  std::vector<float> err;

  if( new_frame.empty() ) return 0;
  //cv::cvtColor(new_frame,new_frame,cv::COLOR_BGR2GRAY);

  if( needInit ) {
    printf("Initializing features\n");
    cv::goodFeaturesToTrack( new_frame, tracked_features[0], MAX_FEATURES, 0.01, 10, border_mask, 3, 0, 0.04 );
    cv::cornerSubPix( new_frame, tracked_features[0],subPixWinSize, Size(-1,-1), termcrit );
    std::cout << tracked_features[0].size() << std::endl;
    needInit = 0;
    return 1;
  }

  if( old_frame.empty() ) return 0;

  if( calcFlow ) {

    if(0*show_image && show_frame.total() != 0){
  	 for( int k = 0; k < (int)tracked_features[0].size(); k++ ){
  	   	cv::circle( show_frame, tracked_features[0].at(k), 4.0 ,Scalar(255,50,1),3,8,0 );
  	  }
    }

    cv::calcOpticalFlowPyrLK(old_frame, new_frame, tracked_features[0], tracked_features[1],
                            status, err, winsize, 4, termcrit, 0, 0.0001);
    //cv::cornerSubPix( new_frame, tracked_features[1],subPixWinSize, Size(-1,-1), termcrit );

    float sum_dx = 0;
    float sum_dy = 0;
    float n = 0;

    for( int k = 0; k < (int)status.size(); k++ ){
    	if( err.at(k) > 5 ) status.at(k) = 0;

    	if( status.at(k) ) {
        if(0*show_image && show_frame.total() != 0){
    		  cv::line(show_frame, tracked_features[0].at(k), tracked_features[1].at(k),Scalar(100,0,255), 0.3, 8, 0);
    		  cv::circle(show_frame, tracked_features[1].at(k), 1 ,Scalar(100,0,255),3,8,0 );
        }

    		sum_dx += ( tracked_features[1].at(k).x - tracked_features[0].at(k).x );
    		sum_dy += ( tracked_features[1].at(k).y - tracked_features[0].at(k).y );
    		n += 1.0;
    	}
    }

    Mat n1 = (Mat_<double>(3,1) << 0,0,1);
    Vec3f rot = Vec3f( new_roll, new_pitch, 0 );
    Mat R33 = eulerAnglesToRotationMatrix( rot );
    Mat n2 = R33*n1;
    float ang = atan2( norm(n1.cross(n2)) , n1.dot(n2) );

    sum_dx = ( sum_dx/n )/257.0 * h_raw*cos(ang);
    sum_dy = ( sum_dy/n )/257.0 * h_raw*cos(ang);

    rot = Vec3f( (new_roll-old_roll), (new_pitch-old_pitch), (new_yaw-old_yaw) );

    //cout << "####" << endl << rot << endl;

    R33 = eulerAnglesToRotationMatrix( rot );

    Mat t_cam = (Mat_<double>(3,1) <<
             -0.19,       0,              0.04
             );

    Mat dT = Mat(3,1,CV_64FC1);

    dT.at<double>(0) = (double) -sum_dy;
    dT.at<double>(1) = (double) -sum_dx;
    dT.at<double>(2) = (double) 0;

    Mat t_mark = (R33*t_cam  + dT) - t_cam;

    rot = Vec3f( 0, 0, new_yaw);

    R33 = eulerAnglesToRotationMatrix( rot );

    t_mark = R33*t_mark;

    simple_dx = t_mark.at<double>(0);
    simple_dy = t_mark.at<double>(1);

    cv::Mat H;

    //auto start = std::chrono::high_resolution_clock::now();

    struct H_decomped d_motion;

    int stat = planar_homography_estimation( &H, &status );

    if( stat == 1 ){
      decompose_homography( &H, &d_motion );

      sigma_1 = d_motion.Sigma.at<float>(0);
      sigma_2 = d_motion.Sigma.at<float>(1);
      sigma_3 = d_motion.Sigma.at<float>(2);

      if( d_motion.Sigma.at<float>(0) < 5 ){

        rot = Vec3f( (new_roll-old_roll), (new_pitch-old_pitch), (new_yaw-old_yaw) );

        R33 = eulerAnglesToRotationMatrix( rot );

        t_cam = (Mat_<double>(3,1) <<
                 -0.19,       0,              0.04
                 );

        dT.at<double>(0) = (double) -d_motion.T.at<float>(1);
        dT.at<double>(1) = (double) -d_motion.T.at<float>(0);
        dT.at<double>(2) = (double) d_motion.T.at<float>(2);

        t_mark = (R33*t_cam  + dT) - t_cam;

        rot = Vec3f( 0, 0, new_yaw);

        R33 = eulerAnglesToRotationMatrix( rot );

        t_mark = R33*t_mark;

        float alpha = 0.2;

        //hom_dx = d_motion.T.at<float>(0)*height*(1-alpha) + alpha*hom_dx;
        //hom_dy = d_motion.T.at<float>(1)*height*(1-alpha) + alpha*hom_dy;
        //hom_dz = d_motion.T.at<float>(2)*height*(1-alpha) + alpha*hom_dz;

        hom_dx = t_mark.at<double>(0)*h_raw*cos(ang)*(1-alpha) + alpha*hom_dx;
        hom_dy = t_mark.at<double>(1)*h_raw*cos(ang)*(1-alpha) + alpha*hom_dy;
        hom_dz = t_mark.at<double>(2)*h_raw*cos(ang)*(1-alpha) + alpha*hom_dz;
      }
    }
  }


  if( calcFlow ) {


    sem_wait(&aruco_mutex);
    
    aruco_pos.num_pose++;
    aruco_pos.x += hom_dx;
    aruco_pos.y += hom_dy;
    aruco_pos.z += hom_dz;
    aruco_pos.th = yaw_for_aruco;

    aruco_pos.valid = 2;

    sem_post(&aruco_mutex);

    //std::cout << d_motion.T*height << std::endl;

  }

  old_roll = new_roll;
  old_pitch = new_pitch;
  old_yaw = new_yaw;

  cv::goodFeaturesToTrack( new_frame, tracked_features[0], MAX_FEATURES, 0.01, 10, border_mask, 3, 0, 0.04 );
  //cv::cornerSubPix( new_frame, tracked_features[0],subPixWinSize, Size(-1,-1), termcrit );

  return 1;
}


//#################################################################################################
//#################################################################################################
//#################################################################################################
//#################################################################################################


int planar_homography_estimation(cv::Mat * H_dst, std::vector<uchar> * status) {

  int n = 0;

  int size = status->size();

  std::vector<int> indecies;
  cv::Point2f point1;
  cv::Point2f point2;

  cv::Mat sigma, u, vt, H;

  std::srand(std::time(0));

  // Check if enough good points for estimation
  int status_sum = 0;
  for( int i = 0; i < size; i++){
    status_sum += status->at(i);
  }
  if(status_sum < 4) return 0;

  n_good_features = status_sum;

  /*
  status_sum = 5;
  int xx[5] = {200,150,70,43,160};
  int yy[5] = {53, 12 ,216, 235, 100};
  float chi_tmp[status_sum*3][9];
  for( int j = 0; j < 5; j++ ){  

    point1.x = xx[j];
    point1.y = yy[j];      
    
    point2.x = xx[j]+9;
    point2.y = yy[j]+6;      
    
    construct_chi( chi_tmp, point1, point2, n );
    n++;
    

  }
  */
  // Construct chi from all the tracked features
  
  float chi_tmp[status_sum*3][9];
  for( int j = 0; j < size; j++ ){  

  	if( status->at(j) == 1 ) {
    	point1 = tracked_features[0].at(j);
    	point2 = tracked_features[1].at(j);
    	
      construct_chi( chi_tmp, point1, point2, n );
      n++;
    }

  }
  
  cv::Mat chi = cv::Mat(status_sum*3,9,CV_32FC1, chi_tmp);

  cv::SVD::compute(chi,sigma,u,vt);

  float H_L[3][3];

  float H_L2[9][1];

  for( int j = 0; j < 3; j++ ) {
  	for( int k = 0; k < 3; k++ ) {
  		H_L[j][k] = vt.at<float>( 8,(j*3+k) );
  		H_L2[(j*3+k)][0] = vt.at<float>( 8,(j*3+k) );
  	}
  }

  cv::Mat H_L2_v = cv::Mat(9,1,CV_32FC1,H_L2);

  H = cv::Mat( 3,3,CV_32FC1,H_L );

  cv::SVD::compute(H,sigma);
  
  H = H / sigma.at<float>(1);

  float x1[3][1] = { {point1.x}, {point1.y}, {z_cam} };
  float x2[1][3] = { point2.x, point2.y, z_cam};

  cv::Mat x1_v = cv::Mat(3,1,CV_32FC1,x1);
  cv::Mat x2_v = cv::Mat(1,3,CV_32FC1,x2);

  cv::Mat tmp = x2_v*H*x1_v;

  if( tmp.at<float>(0) < 0 ) {
  	H = -H;
  }
  
  H.copyTo(*H_dst);

  return 1;
}

//#################################################################################################
//#################################################################################################
//#################################################################################################
//#################################################################################################


int construct_chi( float chi[12][9], cv::Point2f point1, cv::Point2f point2, int position ) {

  float x1[3] = {point1.x-160, point1.y-120, z_cam}; 
  float x2[3][3] = { {0, -z_cam, point2.y-120}, {z_cam, 0, -(point2.x-160)}, {-(point2.y-120), (point2.x-160), 0} }; 

  for( int i = 0; i < 3; i++ ) { // Rows of x1
    for( int j = 0; j < 3; j++ ) { // Rows of x2
      for( int k = 0; k < 3; k++ ) { // Cols of x2
        chi[3*position + j][i*3+k] = x1[i] * x2[j][k];
        //std::cout << (3*position+j) << " , " << (i*3+k) << std::endl;
      }
    }
  }

  return 1;
}

//#################################################################################################
//#################################################################################################
//#################################################################################################
//#################################################################################################


int skew_matrix( cv::Mat * skew_dst, cv::Mat x ) {

	cv::Mat skew = cv::Mat::zeros(3,3,CV_32FC1);
	skew.at<float>(0,1) = -x.at<float>(2);
	skew.at<float>(0,2) = x.at<float>(1);
	skew.at<float>(1,0) = x.at<float>(2);
	skew.at<float>(1,2) = -x.at<float>(0);
	skew.at<float>(2,0) = -x.at<float>(1);
	skew.at<float>(2,1) = x.at<float>(0);

	skew.copyTo(*skew_dst);

	return 1;

}

//#################################################################################################
//#################################################################################################
//#################################################################################################
//#################################################################################################


int decompose_homography(cv::Mat * H, struct H_decomped * motion){

  cv::Mat V, Sigma, Vt;

  cv::Mat HLL = (*H).t() * (*H);

  cv::SVD::compute( HLL,Sigma,V,Vt );

  if( cv::determinant(V) < 0){
  	V = -V;
  }

  cv::Mat u1 = ( sqrt(1-Sigma.at<float>(2) )*V.col(0) + sqrt(Sigma.at<float>(0)-1)*V.col(2) ) / ( sqrt(Sigma.at<float>(0) - Sigma.at<float>(2)) );
  cv::Mat u2 = ( sqrt(1-Sigma.at<float>(2) )*V.col(0) - sqrt(Sigma.at<float>(0)-1)*V.col(2) ) / ( sqrt(Sigma.at<float>(0) - Sigma.at<float>(2)) );

  cv::Mat U1 = cv::Mat::zeros(3,3,CV_32FC1);
  cv::Mat U2 = cv::Mat::zeros(3,3,CV_32FC1);
  cv::Mat W1 = cv::Mat::zeros(3,3,CV_32FC1);
  cv::Mat W2 = cv::Mat::zeros(3,3,CV_32FC1);

  cv::Mat tmp;
  cv::Mat skew_tmp;

  // U1
  V.col(1).copyTo(U1.col(0));
  u1.copyTo(U1.col(1));
  skew_matrix(&skew_tmp, V.col(1));
  tmp = skew_tmp*u1;
  tmp.copyTo(U1.col(2));

	// U2
  V.col(1).copyTo(U2.col(0));
  u2.copyTo(U2.col(1));
  skew_matrix(&skew_tmp, V.col(1));
  tmp = skew_tmp*u2;
  tmp.copyTo(U2.col(2));

  // W1
  tmp = *H * V.col(1);
  tmp.copyTo(W1.col(0));
  tmp = *H * u1;
  tmp.copyTo(W1.col(1));
  skew_matrix(&skew_tmp, W1.col(0));
  tmp = skew_tmp*(*H)*u1;
  tmp.copyTo(W1.col(2));

  // W2
  tmp = *H * V.col(1);
  tmp.copyTo(W2.col(0));
  tmp = *H * u2;
  tmp.copyTo(W2.col(1));
  skew_matrix(&skew_tmp, W2.col(0));
  tmp = skew_tmp*(*H)*u2;
  tmp.copyTo(W2.col(2));

  // Compute first set of solutions
  cv::Mat R1 = W1*U1.t();
  skew_matrix(&skew_tmp, V.col(1));
  cv::Mat N1 = skew_tmp*u1;
  cv::Mat T1 = (*H-R1)*N1;

  if( N1.at<float>(2) < 0){
  	N1 = -N1;
  	T1 = -T1;
  }

  // Compute second set of solutions
  cv::Mat R2 = W2*U2.t();
  skew_matrix(&skew_tmp, V.col(1));
  cv::Mat N2 = skew_tmp*u2;
  cv::Mat T2 = (*H-R2)*N2;

  if( N2.at<float>(2) < 0){
  	N2 = -N2;
  	T2 = -T2;
  }

  if( N1.at<float>(2) > N2.at<float>(2) ){
  	R1.copyTo(motion->R);
  	N1.copyTo(motion->N);
  	T1.copyTo(motion->T);
	}
	else{
		R2.copyTo(motion->R);
  	N2.copyTo(motion->N);
  	T2.copyTo(motion->T);	
	}
	Sigma.copyTo(motion->Sigma);

  //cout << "R = \n" << motion->R << endl;
  //cout << "N = \n" << motion->N << endl;
  //cout << "T = \n" << motion->T << endl;

	return 1;
}

//#################################################################################################
//#################################################################################################
//#################################################################################################
//#################################################################################################


int update_aruco_pos( struct H_decomped * motion ){

	cv::Mat H = cv::Mat::eye(4,4,CV_32FC1);

	//cv::Vect3f angles = cv::Vect3f( roll_for_aruco, pitch_for_aruco,  )
	//cv::R33 = eulerAnglesToRotationMatrix( angles );



	return 1;
}







//#################################################################################################
//#################################################################################################
//#################################################################################################
//#################################################################################################
//#################################################################################################
//#################################################################################################
//#################################################################################################
//#################################################################################################









int ransac_homography_estimation(std::vector<uchar> * status) {
  int N = 10;
  int n;

  int size = status->size();
  char index_ok = 0;

  std::vector<int> indecies;
  cv::Point2f point1;
  cv::Point2f point2;

  cv::Mat sigma, u, vt, H;

  std::srand(std::time(0));

  float chi_tmp[12][9];

  // Check if enough good points for ransac
  int status_sum = 0;
  for( int i = 0; i < size; i++){
    status_sum += status->at(i);
  }
  if(status_sum < 4) return 0;

  // Do N attempts  at estimating the homography
  for(int i = 0; i < N; i++ ) {
    
    // Get 4 point indecies that are valid for use in the ransac
    indecies.clear();
    while( indecies.size() < 4 ){
      n = std::rand()%size;

      index_ok = 1;
      for(unsigned int j = 0; j < indecies.size(); j++ ){
        if( indecies.at(j) == n ){
          index_ok = 0;
          break;
        }
      }
      if( index_ok && status->at(n) ){
        indecies.push_back(n);
      }
    }

    for( int j = 0; j < 4; j++ ){  
      n = indecies.at(j);

      point1 = tracked_features[0].at(n);
      point2 = tracked_features[1].at(n);

      construct_chi( chi_tmp, point1, point2, j );
    }

    cv::Mat chi = cv::Mat(12,9,CV_32FC1, chi_tmp);

    //std::cout << chi << std::endl;

    cv::SVD::compute(chi,sigma,u,vt);

    //std::cout << sigma << "  sigma " << sigma.at<float>(8) << std::endl;

    float H_L[3][3];

    float H_L2[9][1];

    for( int j = 0; j < 3; j++ ) {
    	for( int k = 0; k < 3; k++ ) {
    		H_L[j][k] = vt.at<float>( 8,(j*3+k) );
    		H_L2[(j*3+k)][0] = vt.at<float>( 8,(j*3+k) );
    	}
    }

    cv::Mat H_L2_v = cv::Mat(9,1,CV_32FC1,H_L2);

    H = cv::Mat( 3,3,CV_32FC1,H_L );

    cv::SVD::compute(H,sigma);

    H = H / sigma.at<float>(1);
/*
    cv::Mat x1_v;
    cv::Mat x2_v;

	float x1[3][1] = {{point1.x-160}, {point1.y-120}, {z_cam}}; 
	float x2[1][3] = {{point2.x-160, point2.y-120, z_cam}}; 
	x1_v = cv::Mat(3,1,CV_32FC1,x1);
	x2_v = cv::Mat(1,3,CV_32FC1,x2);

	cv::Mat check = (x1_v*H*x2_v);

	std::cout << check << std::endl;

	if( check.at<float>(0) < 0 ) {
		H = -H;
	}

    //std::cout << H << std::endl;

    for( int j = 0; j < size; j++ ) {
    	point1 = tracked_features[0].at(j);
    	point2 = tracked_features[1].at(j);

    	float x1[3] = {point1.x-160, point1.y-120, z_cam}; 
  		//float x2[3][3] = { {0, -z_cam, point2.y-120}, {z_cam, 0, -(point2.x-160)}, {-(point2.y-120), point2.x, 0} };
		float x2[3] = {point2.x-160, point2.y-120, z_cam}; 
  		x1_v = cv::Mat(3,1,CV_32FC1,x1);
  		x2_v = cv::Mat(1,3,CV_32FC1,x2);

  		std::cout << H*x1_v << std::endl;
  		std::cout << x2_v << std::endl << std::endl;

    }
*/

  }

  return 1;
}
