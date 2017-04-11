#include "ros/ros.h" 
#include "ros/package.h" 
#include "std_msgs/String.h" 
#include "std_msgs/Int16MultiArray.h" 
#include "aruco_detection/ArMarkers.h" 
#include <sstream> 
#include <math.h>
#include <time.h> 

using namespace std; 

//ROS elems
ros::Publisher pub;
vector<double> r, t;
vector<int> ids;
int markerNo;

//PID feedback
float d=0.0;
float angle=0.0; //variables that I must control and that I can measure 
float motorZeroPower=0.0;
float motorOnePower=0.0; //input variables of the system

float previusAngle=0.0;
float integralAngle=0.0;

float estimatedMotorZeroPower = 0;
float estimatedMotorOnePower = 0;
  
const float DISTANCETARGET=0.3; //target distance in cm
const float ANGLETARGET=0; //target angle in cm. NOTE: since the target is 0.0, angle itself is the angle error signal 
const float DISTANCEPIDPROPORTIONALCONSTANT=1000;
const float ANGLEPIDPROPORTIONALCONSTANT=20;
const float ANGLEPIDDERIVATIVECONSTANT=30;
const float ANGLEPIDINTEGRALCONSTANT=10;
const float FILTERCONSTANT = 0.4;

void markerCallback(aruco_detection::ArMarkers msg) {   
	markerNo=msg.markerNo;   
	r=msg.rVecs;   
	t=msg.tVecs;   
	ids=msg.markersIds;   
	  
	std_msgs::Int16MultiArray motorSpeed; 
	   
	if(markerNo > 0) {
		
		d = t.at(2); // z coord
		angle = atan((t.at(0)+0.02)/t.at(2)); //fix upper left corner ref
		
		//integralAngle += angle; //perform integral
		
		
		//FIRST PID FEEDBACK, DISTANCE CONTROL
		
		//PROPORTIONAL
		motorZeroPower=DISTANCEPIDPROPORTIONALCONSTANT*(d-DISTANCETARGET);
		motorOnePower=DISTANCEPIDPROPORTIONALCONSTANT*(d-DISTANCETARGET);
		
		
		if(motorZeroPower>255)
			motorZeroPower=255;
	    if(motorZeroPower<-255)
			motorZeroPower=-255;
		if(motorOnePower>255)
			motorOnePower=255;
		if(motorOnePower<-255)
			motorOnePower=-255;


		//SECOND PID FEEDBACK, ANGLE CONTROL

		//PROPORTIONAL
		motorOnePower+=(angle-ANGLETARGET)*ANGLEPIDPROPORTIONALCONSTANT;
		motorZeroPower-=(angle-ANGLETARGET)*ANGLEPIDPROPORTIONALCONSTANT;
		
		//DERIVATIVE
		motorOnePower+=(angle-previusAngle)*ANGLEPIDDERIVATIVECONSTANT;
		motorZeroPower-=(angle-previusAngle)*ANGLEPIDDERIVATIVECONSTANT;
		
		//INTEGRAL
		/*motorOnePower+=(-(integralAngle)*ANGLEPIDINTEGRALCONSTANT);
		motorZeroPower+=integralAngle*ANGLEPIDINTEGRALCONSTANT;*/


		previusAngle=angle;
		
	}	
	else{  
		if(previusAngle>0){
			motorOnePower=70;
			motorZeroPower=0;
		}else if(previusAngle<0){
			motorOnePower=0;
			motorZeroPower=70;
		}
			
		
		previusAngle=0;
	}
	
	//kalman filter for data
	estimatedMotorOnePower = estimatedMotorOnePower*FILTERCONSTANT + motorOnePower*(1-FILTERCONSTANT);
	estimatedMotorZeroPower = estimatedMotorZeroPower*FILTERCONSTANT + motorZeroPower*(1-FILTERCONSTANT);
	
	motorSpeed.data.push_back(estimatedMotorOnePower);
	motorSpeed.data.push_back(estimatedMotorZeroPower);
	      
	pub.publish(motorSpeed); 
} 
/* 
 *  *
 *   
 * */ 
int main(int argc, char **argv) {   
	ros::init(argc, argv, "process_markers");   
	ros::NodeHandle n;   
	ros::Rate loop_rate(10);   
	
	// SUBSCRIBER   
	ros::Subscriber sub = n.subscribe("markers_stream", 10, markerCallback);   
	
	// PUBLISHER   
	pub = n.advertise<std_msgs::Int16MultiArray>("cmd", 50);   
	while (ros::ok()){     
		ros::spinOnce();     
		loop_rate.sleep();   
	}   
	return 0; 
}





/*
		if((angle-ANGLETARGET)>=0){
			motorOnePower=motorOnePower-(angle-ANGLETARGET)*ANGLEPIDPROPORTIONALCONSTANT;
			motorZeroPower=motorZeroPower+(angle-ANGLETARGET)*ANGLEPIDPROPORTIONALCONSTANT;
			ROS_INFO("angolo maggiore di 0");
		}
		else{
			ROS_INFO("angolo minore di 0");
			motorZeroPower=motorZeroPower-(angle-ANGLETARGET)*ANGLEPIDPROPORTIONALCONSTANT;
			motorOnePower=motorOnePower+(angle-ANGLETARGET)*ANGLEPIDPROPORTIONALCONSTANT;
		}
*/

/*
#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16MultiArray.h"
#include "aruco_detection/ArMarkers.h"
#include <sstream>
#include <math.h>

// OPENCV
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>
// ARUCO
//#include <opencv2/aruco.hpp>

using namespace std;
//using namespace cv;

// ros basic elements
ros::Publisher pub;
vector<double> r, t;
vector<int> ids; 
int markerNo;
float d;
float angle;


void markerCallback(aruco_detection::ArMarkers msg) {
	markerNo=msg.markerNo;   
	r=msg.rVecs;   
	t=msg.tVecs;   
	ids=msg.markersIds;
  
  ros::Rate loop_rate(10);

  for(int i=0; i<t.size(); i+=3){
    d = sqrt( pow(t.at(i),2) + pow(t.at(i+1),2) + pow(t.at(i+2),2));
    angle = atan(t.at(i)/t.at(i+2));
    
    ROS_INFO("distance of %i : %f", ids.at((int)i/3), d);
    ROS_INFO("angle of %i : %f", ids.at((int)i/3), angle);

  }

  //da perfezionare chiaramente. diciamo che lo voglio mantenere a 30 cm in questa prima istanza
  if(markerNo==1){
	  std_msgs::Int16MultiArray motorSpeed;
	  if(d>0.30){
	  	motorSpeed.data.push_back(200);
		motorSpeed.data.push_back(180);
	  }
	  else{
	        motorSpeed.data.push_back(0);
		motorSpeed.data.push_back(0);
	  }
	  pub.publish(motorSpeed);
  }
  else{
	  std_msgs::Int16MultiArray motorSpeed;
	  motorSpeed.data.push_back(0);
	  motorSpeed.data.push_back(0);
	  
	  pub.publish(motorSpeed);
  }
  ros::spinOnce();

  loop_rate.sleep();
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "marker_follower");
  ros::NodeHandle n;
  ros::Rate loop_rate(1);

  //namedWindow( "Output window", WINDOW_AUTOSIZE );

  // SUBSCRIBER
  ros::Subscriber sub = n.subscribe("markers_stream", 5000, markerCallback);

  // PUBLISHER
  pub = n.advertise<std_msgs::Int16MultiArray>("cmd", 1000);

  while (ros::ok())
  {
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
*/
