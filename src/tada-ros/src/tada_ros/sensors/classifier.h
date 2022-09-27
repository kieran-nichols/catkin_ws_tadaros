struct Binary_classifier_linearSVM
{
	double beta[25];
	double bias;
	double kernel_scale;
	double mu[25];
	double sigma[25];
};

struct Multi_classifier_linearSVM
{
	signed char coding_matrix[9];
	struct Binary_classifier_linearSVM binary_classifier[3];
	char n;         // number of features
};

char predict(struct Multi_classifier_linearSVM classifier, double* features);
void initialize_classifier(struct Multi_classifier_linearSVM* classifiers);
