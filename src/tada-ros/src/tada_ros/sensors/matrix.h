double normV(float* V, char dimension);
void MatrixMVector(double* M, double* V, double* Vout, char dimension);
void MatrixMMatrix(double* M1, double* M2, double* Mout, char dimension);
void scaleMatrix(double* M, double s, double* Mout, char dimension);
void cross(double* V1, double* V2, double* Vout);
double dotVector(double* V1, double* V2, char dimension);
