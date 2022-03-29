//
//  arudo_pose.cpp
//  
//
//  Created by Ananda Nielsen on 07/12/2018
//  Copyright © 2018 Ananda Nielsen. All rights reserved.
//

# include "aruco_pose.h"
# include "aruco_map.h"
# include <stdio.h> 

// openCV libs
# include "opencv2/highgui/highgui.hpp"
# include "opencv2/imgproc/imgproc.hpp"
# include "opencv2/opencv.hpp"
# include "opencv2/core/core.hpp"
# include <opencv2/aruco.hpp> //added

# include <sys/time.h>

#include <math.h>
#include "rotations.h"


//#include "defines.h"

using namespace cv;
using namespace std;

//#################################################################################################
//#################################################################################################
//#################################################################################################
//#################################################################################################


/***************************************
*
* Open raspicam camera and set resolution
*
****************************************/
/*
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
*/
//#################################################################################################
//#################################################################################################
//#################################################################################################
//#################################################################################################


/***************************************
*
* Transform a camera position to the
* position of the drone body
*
****************************************/
void transform_camera_to_body( pose_t * cam_pose, double roll, double pitch, double yaw){

	Vec3f rot1 = Vec3f(roll, pitch, yaw);

	Mat R33 = eulerAnglesToRotationMatrix(rot1);

	// Translation from camera to center of mass
	Mat pose = (Mat_<double>(3,1) <<
               -0.09,       0.03,              0.00
               );

	pose = R33*pose;

	cam_pose->x += pose.at<double>(0);
	cam_pose->y += pose.at<double>(1);
	cam_pose->z += pose.at<double>(2);

}

//#################################################################################################
//#################################################################################################
//#################################################################################################
//#################################################################################################

/***************************************
*
* Tranform from marker coordinate syste
* to the global frame of reference
*
****************************************/
void transform_to_global_map(pose_t * local_pose, CameraLocalizer * camera, int marker_id){

	ar_marker marker = camera->global_map.get_marker_from_id(marker_id);

	Vec3f rot = Vec3f(0, 0, marker.th);
	Mat R33 = eulerAnglesToRotationMatrix(rot);
	
	Mat pose = (Mat_<double>(3,1) << 
		local_pose->x, local_pose->y, local_pose->z
		);

	pose = R33*pose;

	local_pose->x = pose.at<double>(0) + marker.x;
	local_pose->y = pose.at<double>(1) + marker.y;
	local_pose->z = pose.at<double>(2) + marker.z;
	local_pose->th += marker.th;
	if( local_pose->th < -M_PI) local_pose->th += 2*M_PI;
	else if ( local_pose->th > M_PI) local_pose->th -= 2*M_PI;
}

//#################################################################################################
//#################################################################################################
//#################################################################################################
//#################################################################################################

/***************************************
*
* Calculate the position of the drone
* in a markers coordinate system
*
****************************************/
pose_t estimate_pos(double roll, double pitch, double yaw, double x, double y, double z) {
	pose_t pose_est;

	double h = z - x*tan(roll) - y*tan(pitch);

	x = h*sin(roll) + x/cos(roll);
	y = h*sin(pitch) + y/cos(pitch);

//     cout << "#############" << endl;
//     cout << "CAM X  = " << x << endl;
//     cout << "CAM Y  = " << y << endl;
//     cout << "CAM TH = " << (yaw)*180.0/M_PI << endl << endl;
    
    Vec3f rot1 = Vec3f(0,0,(yaw));
    Mat R33 = eulerAnglesToRotationMatrix(rot1);
    
    Mat hom = (Mat_<float>(4,4) <<
        R33.at<double>(0,0), R33.at<double>(0,1), R33.at<double>(0,2), x, 
        R33.at<double>(1,0), R33.at<double>(1,1), R33.at<double>(1,2), y,
        R33.at<double>(2,0), R33.at<double>(2,1), R33.at<double>(2,2), h,
        0,0,0,1);

    yaw += M_PI/2.0; 
    if(yaw > M_PI) yaw -= 2*M_PI;
    
    /*
	//Vec3f rot1 = Vec3f(0,0,M_PI/2-yaw_2);
    rot1 = Vec3f(0,0,(yaw - M_PI/2));
    R33 = eulerAnglesToRotationMatrix(rot1);

	Mat pose = (Mat_<double>(3,1) <<
      x,  y,  h
    );
    
	pose = R33*pose;

    */
    
    cv::Mat homInv = hom.inv();
    
//     cout << homInv << endl;
    
	pose_est.x = homInv.at<float>(0,3);
	pose_est.y = -homInv.at<float>(1,3); // Flip because of up/down orientation on z
	pose_est.z = homInv.at<float>(2,3);

	pose_est.th = yaw;

	transform_camera_to_body(&pose_est,roll,pitch,pose_est.th);

//     cout << "Dro X  = " << pose_est.x << endl;
//     cout << "Dro Y  = " << pose_est.y << endl;
//     cout << "Dro TH = " << pose_est.th*180/M_PI << endl;
//     cout << "#############" << endl;
        
	return pose_est;
}


//#################################################################################################
//#################################################################################################
//#################################################################################################
//#################################################################################################

/***************************************
*
* Captures a picture and estimates the
* position of the drone in the global
* coordinate system
*
****************************************/
//struct pose_t find_pos(raspicam::RaspiCam_Cv *cap, double roll, double pitch, bool drone_view)
//pose_t find_pos(cv::Mat *image, ar_map * global_map, double roll, double pitch)
pose_t find_pos(cv::Mat * image, CameraLocalizer * camera, double roll, double pitch)
{

	//Mat src;
	Mat R33 = Mat::eye(3,3,CV_64FC1);

	//image->copyTo(src);

	float markerSize;

	vector<int> markerIds;
	vector<vector<Point2f> > corners, rejectedCandidates;
	vector<Vec3d> rvecs, tvecs;

	aruco::Dictionary dictionary=aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
//	Mat cameraMatrix=(Mat_<double>(3,3)<<257,0,160.0,0,257.0,120,0,0,1);
//	Mat distCoeffs=(Mat_<double>(1,5)<<0.15490157,0.00111697,-0.00344782,0.00492199,-0.80341838);

   	Mat cameraMatrix=(Mat_<double>(3,3)<<311.76734368,0,162.55732796,0,311.28539847,113.87483741,0,0,1);
	Mat distCoeffs=(Mat_<double>(1,5)<<-0.0376471738,  1.21868272, -0.00380002901 , 0.00322747751, -5.65860321);

	pose_t cam_pos;
	cam_pos.valid = 0;
	cam_pos.x = 0;
	cam_pos.y = 0;
	cam_pos.z = 0;
	cam_pos.th = 0;

	float x_avg = 0;
	float y_avg = 0;
	float z_avg = 0;
	float yaw_avg = 0;

	//cap->grab();
	//cap->retrieve(src);

	//if( !src.data ) {
	if( image->empty() ) {
		cout << "ERROR NO PICTURE!!!\n";
		cam_pos.valid = 0;
		return cam_pos;
	}

	//aruco::detectMarkers(src, dictionary, corners, markerIds); 
	aruco::detectMarkers(*image, dictionary, corners, markerIds); 

	if (markerIds.size() > 0){

		int known_markers = 0;
		for(unsigned int i = 0; i < markerIds.size(); i++)
		{
			// Check if marker is known to the map
			if( camera->global_map.get_index_from_id(markerIds[i]) != -1 ){
				known_markers++;
				markerSize = camera->global_map.get_marker_size_from_id(markerIds[i]);
				cv::aruco::estimatePoseSingleMarkers(corners, markerSize, cameraMatrix, distCoeffs, rvecs, tvecs); 
			
				Rodrigues(rvecs[i], R33);
				Vec3f angles = rotationMatrixToEulerAngles(R33);

				cam_pos = estimate_pos(roll, pitch ,angles[2],tvecs[i][0],tvecs[i][1],tvecs[i][2]);
				
				transform_to_global_map(&cam_pos, camera, markerIds[i]);

				x_avg += cam_pos.x;
				y_avg += cam_pos.y;
				z_avg += cam_pos.z;
				yaw_avg += cam_pos.th;
			}
		}

		// Check if any known markers were found
		if( known_markers > 0 ){
			cam_pos.x = x_avg / (float) known_markers;
			cam_pos.y = y_avg / (float) known_markers;
			cam_pos.z = z_avg / (float) known_markers;
			cam_pos.th = yaw_avg / (float) known_markers;

			cam_pos.valid = 1;
		}
	}

	return cam_pos;
	
}
