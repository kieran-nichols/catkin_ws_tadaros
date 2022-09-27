#include <math.h>
#include "matrix.h"

double normV(float* V, char dimension)
{
	double result = 0;
	for (int i = 0; i<dimension; i++)
		result += V[i]*V[i];
	result = sqrt(result);
	return result;
}

void MatrixMVector(double* M, double* V, double* Vout, char dimension)
{
	for (int i=0; i<dimension; i++)
	{
		Vout[i] = 0;
		for (int j=0; j<dimension; j++)
			Vout[i] += M[dimension*i+j]*V[j];
	}
}

void scaleMatrix(double* M, double s, double* Mout, char dimension)
{
	for (int i = 0; i<dimension*dimension; i++)
		M[i] = Mout[i]*s;
}

void cross(double* V1, double* V2, double* Vout)
{
	Vout[0] = V1[1]*V2[2]-V1[2]*V2[1];
	Vout[1] = V1[2]*V2[0]-V1[0]*V2[2];
	Vout[2] = V1[0]*V2[1]-V1[1]*V2[0];
}

double dotVector(double* V1, double* V2, char dimension)
{
	double result = 0;
	for (int i = 0; i<dimension; i++)
		result += V1[i]*V2[i];

	return result;
}
