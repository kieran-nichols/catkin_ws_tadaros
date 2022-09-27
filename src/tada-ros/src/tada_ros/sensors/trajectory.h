

struct kalman
{
	double PKF;
	double QKF;
	double RKF;
	double K;
};

char imu_integrate(float* G, float* A, double* Q, double* V, double* P, struct kalman* K, long int* acount,
		struct Multi_classifier_linearSVM* classifiers, double* walkingSpeed);
void quatUpdate(float*G, float*A, double*Q, char isLowMotion, struct kalman* K);
void getCorrection(double*Q, float*A, struct kalman K);
double computeSTM(float* G, double* STM);
void QuatToRM(double* G, double* RM);
void init(float* A, double* Q, struct kalman* K);
void getInitQuat(float* A, double* Q);
void QuatToEul(double* Q, double* E);
void EulToQuat(double* E, double* Q);
