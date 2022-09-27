#include "ros/ros.h"
#include <iostream>
using namespace std;
#include <stdint.h>
#include <string>
#include "std_msgs/Bool.h"
#include <stdlib.h>
#include "string.h"
#include <math.h>
#include "stdio.h"
#include "std_msgs/String.h"
#include "MPU6050.h"
#include "classifier.h"
//#include "trajectory.c"
#include <math.h>
#include "trajectory.h"
//#include "matrix.h"
#include "matrix.c"
#include "classifier.c"



#define IMU_ADDRESS                (MPU6050_ADDRESS_AD0_LOW<<1)  // address pin low (GND), Accelerometer, Gyroscope, Thermometer
#define BAROMETER_ADDRESS          (0x77<<1)                     // address of BMP-180 Pressure Sensor
#define MAG_ADDRESS                (MPU6050_RA_MAG_ADDRESS<<1)   // address of magnetometer onboard MPU-9150


//C to CPP
std_msgs::String str_var; //global
float A0;
float A1;
float A2;
float G0;
float G1;
float G2;
float A[3] = { };
float G[3] = { };
//----- integration related variables -----//
long int* acount = 0;
double Q[4];
double V[3];
double P[3];
struct kalman* K;
struct Multi_classifier_linearSVM classifiers[5];
//char class = 0;
double walkingSpeed[1];



void sensor(const std_msgs::String::ConstPtr& msg){ //gets data from sensor node
	std::string sensor_info = msg->data.c_str();
	//cout<<sensor_info;
	//cout<<"\n";
	int i =0;
	int b =0;
	int lastspace =0;
	std::string a0;
	std::string a1;
	std::string a2;
	std::string g0;
	std::string g1;
	std::string g2;
	while (sensor_info[i] !='\0'){
		if (sensor_info[i] == ' '){
			if (b==1){//acc in X
				a0 = sensor_info.substr(lastspace, lastspace+1);
				A[0] = stof(a0);
			}
			if (b==4){//accel in Y
				a1 = sensor_info.substr(lastspace, lastspace+1);
				A[1] = stof(a1);
			}
			if (b==7){//accel in Z
				a2 = sensor_info.substr(lastspace, lastspace+1);
				A[2] = stof(a2);
			}
			if (b==10){//geo in X
				g0 = sensor_info.substr(lastspace, lastspace+1);
				G[0] = stof(g0);
			}
			if (b==13){//geo in X
				g1 = sensor_info.substr(lastspace, lastspace+1);
				G[1] = stof(g1);
			}
			if (b==16){//geo in X
				g2 = sensor_info.substr(lastspace, lastspace+1);
				G[2] = stof(g2);
			}
			lastspace =i;
			b++;
		}
		i++;
	}
	
	imu_integrate(G,A, Q, V, P, K, acount, classifiers, walkingSpeed);
	
	cout<<"geo: "<<G<<"\n";
	cout<<"acc: "<<A<<"\n";
	cout<<"velc: "<<V<<"\n";
	cout<<"location: "<<P<<"\n";
	//A[0] =  //Acc in X
	//A[1] =  //Acc in Y
	//A[2] =  //Acc in Z
	
	
}
void kill(const std_msgs::Bool Reached){
	if (Reached.data){
		ROS_INFO("Heard kill");
		exit(EXIT_SUCCESS);
	}
	
}



// Constants
double grav = 9.8027;
const double dt = 0.005;

// Thresholds
double gyro_threshold_velocity = 0.8;
double gyro_threshold_orientation = 0.55;
double acce_threshold = 0.8;
int low_motion_count_threshold = 10;
int nonlow_motion_count_threshold = 5;

// Accumulations
float grav_accu = 0;
int low_motion_count = 0;
int nonlow_motion_count = 0;
double swing_time = 0;
double swing_time_saved = 0;
double stance_time = 0;
double average_swing_time = 0.75;
double last_five_swing_time[5] = {0.75, 0.75, 0.75, 0.75, 0.75};
char feature_number = 1;

 // stance of swing, 0 for stance, 1 for swing
 char walkStatus = 0;

// IMU calibration
// modify this part for your own IMU
double Gbias[3] = {0,0,0};
double Acal[9] = {1,0,0,
		          0,1,0,
				  0,0,1};
double Abias[3] = {0,0,0};

// speed related
double P_speed[3] = {0};
double velocityThreshold = 0.2;
char speedUpdated = 0;
double strideTime = 0;

// Classification related
double P_start[3];
double features[25];
char conditions = 0;

// Orientation, velocity and position estimation
char imu_integrate(float* G, float* A, double* Q, double* V, double* P, struct kalman* K, long* acount,
		           struct Multi_classifier_linearSVM* classifiers, double* walkingSpeed)
{
	////////////////////////////////////////
	//// Calibration and Initialization ////
	////////////////////////////////////////

	// Calibrate IMU
	double Atemp1[3] = {};
	double Atemp2[3] = {};
	for (int i = 0; i<3; i++)
		Atemp1[i] = A[i];
	MatrixMVector(Acal,Atemp1,Atemp2,3);
	for (int i = 0; i<3; i++)
	{
		A[i] = Atemp2[i]+Abias[i];
		G[i] = G[i]-Gbias[i];
	}

	// Initialize kalman filter and quaternion
	if ((*acount) <= 9)
	{
		if ((*acount) == 0)
			init(A,Q,K);

		// Use measured gravity to compensate for accelerometer bias
		grav_accu += normV(A,3);
		if ((*acount) == 9)
			grav = grav_accu/10;
		(*acount)++;
	}

	/////////////////////////////
	//// Gait Reconstruction ////
	/////////////////////////////

	// Compute magnitude of acce and gyro
	double Av = normV(A,3);
	double Gyro = normV(G,3);

	// judge static periods
	char isLowMotion = (Gyro<gyro_threshold_velocity) && (fabs(Av-grav)<acce_threshold);
	
	if (isLowMotion)
	{
		low_motion_count++;
		nonlow_motion_count = 0;
	}
	else
	{
		nonlow_motion_count++;
		low_motion_count = 0;
	}

	// determine transition between stance and swing
	if (!walkStatus)      // stance to swing
	{
		if (nonlow_motion_count >= nonlow_motion_count_threshold)
			walkStatus = 1;
	}
	else                  // swing to stance
	{
		if (low_motion_count >= low_motion_count_threshold)
			walkStatus = 0;
	}

	// update swing_time and stance_time
	if (walkStatus)
	{
		swing_time += dt;
		stance_time = 0;
	}
	else
	{
		if (stance_time == 0 && (*acount) != 1)
			swing_time_saved = swing_time;
		stance_time += dt;
		swing_time = 0;
	}

	// update quaternion
	char apply_correction = (Gyro<gyro_threshold_orientation) && (fabs(Av-grav)<acce_threshold);
	quatUpdate(G, A, Q, apply_correction, K);
	
	// Compute accleration in world frame
	double RM[9];
	QuatToRM(Q,RM);
	
	double acce[3];
	for (int i = 0; i<3; i++)
		acce[i] = (double)A[i];
	double Aworld[3];
	MatrixMVector(RM,acce,Aworld,3);
	
	// Compute velocity
	V[0] = V[0]+Aworld[0]*dt;
	V[1] = V[1]-Aworld[1]*dt;
	V[2] = V[2]-(Aworld[2]+grav)*dt;
	
	// Compute position
	P[0] = P[0]+V[0]*dt+0.5*Aworld[0]*dt*dt;
	P[1] = P[1]+V[1]*dt-0.5*Aworld[1]*dt*dt;
	P[2] = P[2]+V[2]*dt-0.5*(Aworld[2]+grav)*dt*dt;

	if (!walkStatus)
	{
		// Linear velocity correction
		if (fabs(stance_time-dt)<0.001f && (*acount)!=1)
		{
			double acce_error[3];
			for (int i = 0; i<3; i++)
			{
				acce_error[i] = V[i]/swing_time_saved;
				P[i] = P[i]-0.5f*acce_error[i]*swing_time_saved*swing_time_saved;
			}
		}
		
		// assume velocity is zero in stance
		for (int i = 0; i<3; i++)
			V[i] = 0;
	}
	
	//////////////////////////////////
	//// Walking Speed Estimation ////
	//////////////////////////////////

	if (walkStatus)
	{
		// Update walking speed as soon as simultaneous speed exceed threshold
		// and only when the previous swing phase lasts more than 0.3s
		if (sqrt(V[0]*V[0]+V[1]*V[1]+V[2]*V[2])>velocityThreshold && !speedUpdated && swing_time_saved > 0.3)
		{
			double P_stride[3];
			for (int i = 0; i<3; i++)
				P_stride[i] = P[i]-P_speed[i];
			*walkingSpeed = sqrt(P_stride[0]*P_stride[0]+P_stride[1]*P_stride[1]+P_stride[2]*P_stride[2])/strideTime;

			// reset variables
			strideTime = 0;
			for (int i = 0; i<3; i++)
				P_speed[i] = P[i];
			speedUpdated = 1;
		}
	}
	else
		speedUpdated = 0;

	// Update strideTime, whenever in stance or swing
	strideTime += dt;

	////////////////////////
	//// Classification ////
	////////////////////////

	// Update average swing time to adapt different velocity
	if (walkStatus)
	{
		if (fabs(swing_time-dt) < 0.001)
		{
			for (int i = 0; i<4; i++)
				last_five_swing_time[i] = last_five_swing_time[i+1];

			if (swing_time_saved > 0.3 && swing_time_saved < 1.2)
				last_five_swing_time[4] = swing_time_saved;
			else
				last_five_swing_time[4] = 0.75;

			double accumulated_swing_time = 0;
			for (int i = 0; i<5; i++)
				accumulated_swing_time += last_five_swing_time[i];
			average_swing_time = accumulated_swing_time/5;
		}
	}
	else
		for (int i = 0; i<3; i++)
			P_start[i] = P[i];

	// Extract features and predict
	if (fabs(swing_time-average_swing_time*0.1*(feature_number+2)) <= dt/2+0.000001)
	{
		double euler[3];
		QuatToEul(Q,euler);

		if (feature_number == 1)
		{
			features[0] = P[2]-P_start[2];
			features[1] = sqrt((P[0]-P_start[0])*(P[0]-P_start[0])+(P[1]-P_start[1])*(P[1]-P_start[1]));
			features[2] = V[2];
			features[3] = sqrt(V[0]*V[0]+V[1]*V[1]);
			features[4] = euler[0];
			features[5] = G[0];
			conditions = predict(classifiers[0], features);
		}
		else if (feature_number == 2)
		{
			features[6] = P[2]-P_start[2];
			features[7] = sqrt((P[0]-P_start[0])*(P[0]-P_start[0])+(P[1]-P_start[1])*(P[1]-P_start[1]));
			features[8] = V[2];
			features[9] = sqrt(V[0]*V[0]+V[1]*V[1]);
			features[10] = euler[0];
			features[11] = G[0];
			conditions = predict(classifiers[1], features);
		}
		else if (feature_number == 3)
		{
			features[12] = P[2]-P_start[2];
			features[13] = sqrt((P[0]-P_start[0])*(P[0]-P_start[0])+(P[1]-P_start[1])*(P[1]-P_start[1]));
			features[14] = V[2];
			features[15] = sqrt(V[0]*V[0]+V[1]*V[1]);
			features[16] = euler[0];
			features[17] = G[0];
			conditions = predict(classifiers[2], features);
		}
		else if (feature_number == 4)
		{
			features[18] = P[2]-P_start[2];
			features[19] = sqrt((P[0]-P_start[0])*(P[0]-P_start[0])+(P[1]-P_start[1])*(P[1]-P_start[1]));
			features[20] = V[2];
			features[21] = euler[0];
			features[22] = G[0];
			conditions = predict(classifiers[3], features);
		}
		else if (feature_number == 5)
		{
			features[23] = P[2]-P_start[2];
			features[24] = sqrt((P[0]-P_start[0])*(P[0]-P_start[0])+(P[1]-P_start[1])*(P[1]-P_start[1]));
			conditions = predict(classifiers[4], features);
		}

		feature_number += 1;
		if (feature_number == 6)
			feature_number = 1;
	}

	return conditions;
}

void quatUpdate(float*G, float*A, double*Q, char isLowMotion, struct kalman* K)
{
	// Updata kalman filter
	(*K).PKF = (*K).PKF+(*K).QKF;
	
	// quaternion integration
	double STM[16];
	computeSTM(G, STM);

	double newQ[4];
	MatrixMVector(STM, Q, newQ, 4);

	// If is in static period, apply correction
	if (isLowMotion)
	{
		(*K).K = (*K).PKF/((*K).PKF+(*K).RKF);
		getCorrection(newQ,A,*K);
		(*K).PKF = (1-(*K).K)*(*K).PKF*(1-(*K).K)+(*K).K*(*K).RKF*(*K).K;
	}
	
	// updata quaternion
	for (int i = 0; i<4; i++)
		Q[i] = newQ[i];
}

void getCorrection(double*Q, float*A, struct kalman K)
{
	double g = 9.80297286843;

	double theta = A[0]/g;
	if (theta>=-1 && theta<=1)
		theta = asin(theta);
	else if (theta<-1)
		theta = -3.14159/2;
	else
		theta = 3.14159/2;
	
	double phi = A[1]/cos(theta)/g;
	if (phi>=-1 && phi<=1)
		phi = -asin(phi);
	else if (phi<-1)
		phi = 3.14159/2;
	else
		phi = -3.14159/2;

	double E[3];
	QuatToEul(Q,E);

	double newE[3];
	newE[0] = E[0]-(E[0]-phi)*K.K;
	newE[1] = E[1]-(E[1]-theta)*K.K;
	newE[2] = E[2];

	EulToQuat(newE,Q);
}

double computeSTM(float* G, double* STM)
{
	double TIARM = normV(G,3)*dt;
	
	double OMEGA[16] = {0, -G[0], -G[1], -G[2],
	                  G[0], 0, G[2], -G[1],
	                  G[1], -G[2], 0, G[0],
	                  G[2], G[1], -G[0], 0};
	for (int i = 0; i<16; i++)
		OMEGA[i] = OMEGA[i]*dt/2;
										
	double k1 = cos(TIARM/2);
	double k2 = 2/TIARM*sin(TIARM/2);
	for (int i = 0; i<4; i++)
	{
		for (int j = 0; j<4; j++)
		{
			if (i==j)
				STM[4*i+j] = k1;
			else
				STM[4*i+j] = k2*OMEGA[4*i+j];
		}
	}

	return k1;
}

void QuatToRM(double* Q, double* RM)
{
	RM[0] = Q[0]*Q[0]+Q[1]*Q[1]-Q[2]*Q[2]-Q[3]*Q[3];
	RM[1] = 2*Q[1]*Q[2]-2*Q[0]*Q[3];
	RM[2] = 2*Q[1]*Q[3]+2*Q[0]*Q[2];
	RM[3] = 2*Q[1]*Q[2]+2*Q[0]*Q[3];
	RM[4] = Q[0]*Q[0]-Q[1]*Q[1]+Q[2]*Q[2]-Q[3]*Q[3];
	RM[5] = 2*Q[2]*Q[3]-2*Q[0]*Q[1];
	RM[6] = 2*Q[1]*Q[3]-2*Q[0]*Q[2];
	RM[7] = 2*Q[2]*Q[3]+2*Q[0]*Q[1];
	RM[8] = Q[0]*Q[0]-Q[1]*Q[1]-Q[2]*Q[2]+Q[3]*Q[3];
}

void init(float* A, double* Q, struct kalman* K)
{
	(*K).K = 0;
	(*K).PKF = 0;
	(*K).QKF = 1.5e-5f*dt;
	(*K).RKF = 1.5e-1f*dt;
	
	getInitQuat(A,Q);
}

void getInitQuat(float* A, double* Q)
{
	double gM = normV(A,3);
	double z[3];
	z[0] = -A[0]/gM;
	z[1] = -A[1]/gM;
	z[2] = -A[2]/gM;
	
	double x[3];
	if (z[0] != 0)
	{
		x[0] = 0;
		x[1] = sqrt(1/(1+(z[1]*z[1])/(z[2]*z[2])));
		x[2] = -x[1]*z[1]/z[2];
	}
	else if (z[1] != 0)
	{
		x[1] = 0;
		x[0] = sqrt(1/(1+(z[0]*z[0])/(z[2]*z[2])));
		x[2] = -x[0]*z[0]/z[2];
	}
	else
	{
		x[2] = 0;
		x[0] = sqrt(1/(1+(z[0]*z[0])/(z[1]*z[1])));
		x[1] = -x[0]*z[0]/z[1];
	}
	
	double y[3];
	cross(z,x,y);
	
	double t = x[0]+y[1]+z[2];
	if (t>=0)
	{
		double r = sqrt(1+t);
		double s = 0.5f/r;
		Q[0] = 0.5f*r;
		Q[1] = (z[1]-y[2])*s;
		Q[2] = (x[2]-z[0])*s;
		Q[3] = (y[0]-x[1])*s;
	}
	else if (x[0]>=y[1] && x[0]>=z[2])
	{
		double r = sqrt(1+x[0]-y[1]-z[2]);
		double s = 0.5f/r;
		Q[0] = (z[1]-y[2])*s;
		Q[1] = 0.5f*r;
		Q[2] = (x[1]+y[0])*s;
		Q[3] = (z[0]+x[2])*s;
	}
	else if (y[1]>x[0] && y[1]>=z[2])
	{
		double r = sqrt(1+y[1]-x[0]-z[2]);
		double s = 0.5f/r;
		Q[0] = (x[2]-z[0])*s;
		Q[1] = (x[1]+y[0])*s;
		Q[2] = 0.5f*r;
		Q[3] = (z[1]+y[2])*s;
	}
	else if(z[2]>x[0] && z[2]>y[1])
	{
		double r = sqrt(1+z[2]-x[0]-y[1]);
		double s = 0.5f/r;
		Q[0] = (y[0]-x[1])*s;
		Q[1] = (x[2]+z[0])*s;
		Q[2] = (y[2]+z[1])*s;
		Q[3] = 0.5f*r;
	}
}

void QuatToEul(double* Q, double* E)
{
	E[0] = atan2(2*(Q[0]*Q[1]+Q[2]*Q[3]),Q[0]*Q[0]-Q[1]*Q[1]-Q[2]*Q[2]+Q[3]*Q[3]);
	E[1] = asin(2*(Q[0]*Q[2]-Q[1]*Q[3]));
	E[2] = atan2(2*(Q[0]*Q[3]+Q[1]*Q[2]),Q[0]*Q[0]+Q[1]*Q[1]-Q[2]*Q[2]-Q[3]*Q[3]);
}

void EulToQuat(double* E, double* Q)
{
	double cosEul[3];
	double sinEul[3];

	for (int i = 0; i < 3; i++)
	{
		cosEul[i] = cos(E[i]/2.0f);
		sinEul[i] = sin(E[i]/2.0f);
	}

	Q[0] = cosEul[0]*cosEul[1]*cosEul[2]+sinEul[0]*sinEul[1]*sinEul[2];
	Q[1] = sinEul[0]*cosEul[1]*cosEul[2]-cosEul[0]*sinEul[1]*sinEul[2];
	Q[2] = cosEul[0]*sinEul[1]*cosEul[2]+sinEul[0]*cosEul[1]*sinEul[2];
	Q[3] = cosEul[0]*cosEul[1]*sinEul[2]-sinEul[0]*sinEul[1]*cosEul[2];
}

int main(int argc, char **argv){
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	ros::Subscriber sub1 = n.subscribe("sensing_topic",1000,sensor);
	ros::Subscriber sub2 = n.subscribe("kill_confirmation_topic",1000,kill);
	ros::spin();
	return 0;
}

