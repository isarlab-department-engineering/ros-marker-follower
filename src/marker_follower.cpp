#include "ros/ros.h" 
#include "ros/package.h" 
#include "std_msgs/String.h" 
#include "std_msgs/Int16MultiArray.h" 
#include "aruco_detection/ArMarkers.h" 
#include <sstream> 
#include <math.h>
#include <time.h>
#include <string>


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

int startTime=0;

float estimatedMotorZeroPower = 0;
float estimatedMotorOnePower = 0;
  
float distanceTarget=0.5; //target distance in cm
const float ANGLETARGET=0; //target angle in cm. NOTE: since the target is 0.0, angle itself is the angle error signal 
const float DISTANCEPIDPROPORTIONALCONSTANT=1000;
float anglePIDProportionalConstant;
const float ANGLEPIDDERIVATIVECONSTANT=30;
const float ANGLEPIDINTEGRALCONSTANT=10;
const float FILTERCONSTANT = 0.5;

int idToFollow=-1;

void markerCallback(aruco_detection::ArMarkers msg) {
	/*if(startTime==0){
		struct timeval tp;
		gettimeofday(&tp, NULL);
		startTime = tp.tv_sec * 1000 + tp.tv_usec / 1000;
	}*/
	markerNo=msg.markerNo;   
	r=msg.rVecs;   
	t=msg.tVecs;   
	ids=msg.markersIds;     
	std_msgs::Int16MultiArray motorSpeed;
	int indexToFollow=-1;
	
	//set the marker to follow among them that can be seen. must be done in this certain conditions (over the camera view there must be just a marker)
	if(idToFollow==-1 && markerNo==1)
		idToFollow=ids[0];
	
	//find the index to get values
	if(idToFollow!=-1)
		for(int i=0;i<markerNo;i++)
			if(ids[i]==idToFollow)
				indexToFollow=i;
	
	
	if(indexToFollow!=-1 && idToFollow!=-1){
		d = t.at(3*indexToFollow+2); // z coord
		angle = atan((t.at(3*indexToFollow))/t.at(3*indexToFollow+2)); //fix upper left corner ref
		
		//printf("%f",angle);
		
				
		/*//utile per fare i grafici di come reagisce, dopo togliere
		struct timeval tp;
		gettimeofday(&tp, NULL);
		long int currentTime = tp.tv_sec * 1000 + tp.tv_usec / 1000;
		printf("%f \t %f \t %i \n", d, angle, currentTime-startTime);*/
		
					
		//integralAngle += angle; //perform integral
		
		
		//FIRST PID FEEDBACK, DISTANCE CONTROL
		
		//PROPORTIONAL
		motorZeroPower=DISTANCEPIDPROPORTIONALCONSTANT*(d-distanceTarget);
		motorOnePower=DISTANCEPIDPROPORTIONALCONSTANT*(d-distanceTarget);
		
		
		if(motorZeroPower>255)
			motorZeroPower=255;
	    if(motorZeroPower<-255)
			motorZeroPower=-255;
		if(motorOnePower>255)
			motorOnePower=255;
		if(motorOnePower<-255)
			motorOnePower=-255;


		//SECOND PID FEEDBACK, ANGLE CONTROL
		
		anglePIDProportionalConstant=400*abs(angle);
		printf("%f \n",anglePIDProportionalConstant);

		//PROPORTIONAL
		
		motorOnePower+=(angle-ANGLETARGET)*anglePIDProportionalConstant;
		motorZeroPower-=(angle-ANGLETARGET)*anglePIDProportionalConstant;
		
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
	
	/*struct timeval tp;
	gettimeofday(&tp, NULL);
	long int currentTime = tp.tv_sec * 1000 + tp.tv_usec / 1000;
	printf("%i \n", currentTime-startTime);*/
} 
/* 
 *  *
 *   
 * */ 
int main(int argc, char **argv) {
	ros::init(argc, argv, "process_markers");  
	ros::NodeHandle n;   
	ros::Rate loop_rate(10);   
	
	//if(argv[1]!=NULL)
	//	distanceTarget=strtof((argv[1]),0);
	
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
