//#include "classifier.h"
#include "matrix.h"
#include <math.h>

double score[3];
double predictor[3];

char predict(struct Multi_classifier_linearSVM classifier, double* features)
{
	// Calculate score for each classifier
	char n = classifier.n;           // number of features
	for (int i = 0; i<3; i++)
	{
		struct Binary_classifier_linearSVM binary_classifier = classifier.binary_classifier[i];
		double scaled_features[n];
		for (int j = 0; j<n; j++)
			scaled_features[j] = (features[j]-binary_classifier.mu[j])/binary_classifier.sigma[j]/binary_classifier.kernel_scale;

		score[i] =  dotVector(scaled_features,binary_classifier.beta,n)+binary_classifier.bias;
	}

	// Calculate multi-class prediction
	for (int i = 0; i<3; i++)
	{
		predictor[i] = 0;
		double g[3];
		for (int j = 0; j<3; j++)
		{
			double temp = 1-classifier.coding_matrix[i*3+j]*score[j];
			if (temp > 0)
				g[j] = temp/2;
			else
				g[j] = 0;
			predictor[i] += fabs(classifier.coding_matrix[i*3+j])*g[j];
		}
	}

	char min_index;
	if (predictor[0]<=predictor[1] && predictor[0]<=predictor[2])
		min_index = 0;
	else if (predictor[1]<=predictor[0] && predictor[1]<=predictor[2])
		min_index = 1;
	else
		min_index = 2;

	return min_index;
}

void initialize_classifier(struct Multi_classifier_linearSVM* classifiers)
{
	for (int i = 0; i<5; i++)
	{
		classifiers[i].coding_matrix[0] = 1;
		classifiers[i].coding_matrix[1] = 1;
		classifiers[i].coding_matrix[2] = 0;
		classifiers[i].coding_matrix[3] = -1;
		classifiers[i].coding_matrix[4] = 0;
		classifiers[i].coding_matrix[5] = 1;
		classifiers[i].coding_matrix[6] = 0;
		classifiers[i].coding_matrix[7] = -1;
		classifiers[i].coding_matrix[8] = -1;
	}

	// Classifier at 30% of swing time
	classifiers[0].n = 6;
	// binary Classifier1
	classifiers[0].binary_classifier[0].beta[0] = -1.2988;
	classifiers[0].binary_classifier[0].beta[1] = -0.3664;
	classifiers[0].binary_classifier[0].beta[2] = -2.0206;
	classifiers[0].binary_classifier[0].beta[3] = -0.2988;
	classifiers[0].binary_classifier[0].beta[4] = 0.3909;
	classifiers[0].binary_classifier[0].beta[5] = 0.0411;

	classifiers[0].binary_classifier[0].mu[0] = 0.0477;
	classifiers[0].binary_classifier[0].mu[1] = 0.1937;
	classifiers[0].binary_classifier[0].mu[2] = 0.1538;
	classifiers[0].binary_classifier[0].mu[3] = 2.4130;
	classifiers[0].binary_classifier[0].mu[4] = 1.7386;
	classifiers[0].binary_classifier[0].mu[5] = -2.0924;

	classifiers[0].binary_classifier[0].sigma[0] = 0.0406;
	classifiers[0].binary_classifier[0].sigma[1] = 0.0509;
	classifiers[0].binary_classifier[0].sigma[2] = 0.5456;
	classifiers[0].binary_classifier[0].sigma[3] = 0.5286;
	classifiers[0].binary_classifier[0].sigma[4] = 0.2179;
	classifiers[0].binary_classifier[0].sigma[5] = 1.8104;

	classifiers[0].binary_classifier[0].kernel_scale = 1.9330;
	classifiers[0].binary_classifier[0].bias = -1.3318;

	// binary classifier2
	classifiers[0].binary_classifier[1].beta[0] = -2.2968;
	classifiers[0].binary_classifier[1].beta[1] = -0.2880;
	classifiers[0].binary_classifier[1].beta[2] = -5.4076;
	classifiers[0].binary_classifier[1].beta[3] = -0.2597;
	classifiers[0].binary_classifier[1].beta[4] = -0.4273;
	classifiers[0].binary_classifier[1].beta[5] = -0.6946;

	classifiers[0].binary_classifier[1].mu[0] = 0.0559;
	classifiers[0].binary_classifier[1].mu[1] = 0.1293;
	classifiers[0].binary_classifier[1].mu[2] = 0.3903;
	classifiers[0].binary_classifier[1].mu[3] = 1.3557;
	classifiers[0].binary_classifier[1].mu[4] = 1.4302;
	classifiers[0].binary_classifier[1].mu[5] = -1.0119;

	classifiers[0].binary_classifier[1].sigma[0] = 0.0950;
	classifiers[0].binary_classifier[1].sigma[1] = 0.0833;
	classifiers[0].binary_classifier[1].sigma[2] = 1.2736;
	classifiers[0].binary_classifier[1].sigma[3] = 0.7257;
	classifiers[0].binary_classifier[1].sigma[4] = 0.1900;
	classifiers[0].binary_classifier[1].sigma[5] = 2.3175;

	classifiers[0].binary_classifier[1].kernel_scale = 1.9955;
	classifiers[0].binary_classifier[1].bias = 0.9058;

	// binary classifier3
	classifiers[0].binary_classifier[2].beta[0] = -0.9853;
	classifiers[0].binary_classifier[2].beta[1] = -1.7830;
	classifiers[0].binary_classifier[2].beta[2] = -2.5318;
	classifiers[0].binary_classifier[2].beta[3] = 1.2715;
	classifiers[0].binary_classifier[2].beta[4] = 0.0120;
	classifiers[0].binary_classifier[2].beta[5] = -0.3725;

	classifiers[0].binary_classifier[2].mu[0] = 0.0773;
	classifiers[0].binary_classifier[2].mu[1] = 0.1750;
	classifiers[0].binary_classifier[2].mu[2] = 0.5769;
	classifiers[0].binary_classifier[2].mu[3] = 2.2621;
	classifiers[0].binary_classifier[2].mu[4] = 1.7273;
	classifiers[0].binary_classifier[2].mu[5] = -1.7356;

	classifiers[0].binary_classifier[2].sigma[0] = 0.0365;
	classifiers[0].binary_classifier[2].sigma[1] = 0.0688;
	classifiers[0].binary_classifier[2].sigma[2] = 0.5579;
	classifiers[0].binary_classifier[2].sigma[3] = 0.7977;
	classifiers[0].binary_classifier[2].sigma[4] = 0.2294;
	classifiers[0].binary_classifier[2].sigma[5] = 1.7555;

	classifiers[0].binary_classifier[2].kernel_scale = 0.8441;
	classifiers[0].binary_classifier[2].bias = 1.4104;

	// Classifier at 40% of swing time
	classifiers[1].n = 12;
	// binary Classifier1
	classifiers[1].binary_classifier[0].beta[0] = -1.7244;
	classifiers[1].binary_classifier[0].beta[1] = -0.6511;
	classifiers[1].binary_classifier[0].beta[2] = 0.0799;
	classifiers[1].binary_classifier[0].beta[3] = 0.5423;
	classifiers[1].binary_classifier[0].beta[4] = -0.0333;
	classifiers[1].binary_classifier[0].beta[5] = 0.2169;
	classifiers[1].binary_classifier[0].beta[6] = -1.0227;
	classifiers[1].binary_classifier[0].beta[7] = -0.1586;
	classifiers[1].binary_classifier[0].beta[8] = -2.9221;
	classifiers[1].binary_classifier[0].beta[9] = -1.0669;
	classifiers[1].binary_classifier[0].beta[10] = -0.0986;
	classifiers[1].binary_classifier[0].beta[11] = 0.0320;

	classifiers[1].binary_classifier[0].mu[0] = 0.0477;
	classifiers[1].binary_classifier[0].mu[1] = 0.1937;
	classifiers[1].binary_classifier[0].mu[2] = 0.1538;
	classifiers[1].binary_classifier[0].mu[3] = 2.4130;
	classifiers[1].binary_classifier[0].mu[4] = 1.7386;
	classifiers[1].binary_classifier[0].mu[5] = -2.0924;
	classifiers[1].binary_classifier[0].mu[6] = 0.0307;
	classifiers[1].binary_classifier[0].mu[7] = 0.4137;
	classifiers[1].binary_classifier[0].mu[8] = -0.4606;
	classifiers[1].binary_classifier[0].mu[9] = 3.0616;
	classifiers[1].binary_classifier[0].mu[10] = 1.4739;
	classifiers[1].binary_classifier[0].mu[11] = -4.4493;

	classifiers[1].binary_classifier[0].sigma[0] = 0.0406;
	classifiers[1].binary_classifier[0].sigma[1] = 0.0509;
	classifiers[1].binary_classifier[0].sigma[2] = 0.5456;
	classifiers[1].binary_classifier[0].sigma[3] = 0.5286;
	classifiers[1].binary_classifier[0].sigma[4] = 0.2179;
	classifiers[1].binary_classifier[0].sigma[5] = 1.8104;
	classifiers[1].binary_classifier[0].sigma[6] = 0.0714;
	classifiers[1].binary_classifier[0].sigma[7] = 0.0917;
	classifiers[1].binary_classifier[0].sigma[8] = 0.3809;
	classifiers[1].binary_classifier[0].sigma[9] = 0.6625;
	classifiers[1].binary_classifier[0].sigma[10] = 0.2012;
	classifiers[1].binary_classifier[0].sigma[11] = 1.4499;

	classifiers[1].binary_classifier[0].kernel_scale = 4.0906;
	classifiers[1].binary_classifier[0].bias = -1.1555;

	// binary Classifier2
	classifiers[1].binary_classifier[1].beta[0] = -1.5730;
	classifiers[1].binary_classifier[1].beta[1] = -0.6239;
	classifiers[1].binary_classifier[1].beta[2] = -1.9785;
	classifiers[1].binary_classifier[1].beta[3] = -0.0483;
	classifiers[1].binary_classifier[1].beta[4] = -0.0786;
	classifiers[1].binary_classifier[1].beta[5] = -1.1199;
	classifiers[1].binary_classifier[1].beta[6] = -2.4165;
	classifiers[1].binary_classifier[1].beta[7] = -0.3214;
	classifiers[1].binary_classifier[1].beta[8] = -4.3538;
	classifiers[1].binary_classifier[1].beta[9] = -0.2960;
	classifiers[1].binary_classifier[1].beta[10] = -0.2418;
	classifiers[1].binary_classifier[1].beta[11] = 0.6016;

	classifiers[1].binary_classifier[1].mu[0] = 0.0559;
	classifiers[1].binary_classifier[1].mu[1] = 0.1293;
	classifiers[1].binary_classifier[1].mu[2] = 0.3903;
	classifiers[1].binary_classifier[1].mu[3] = 1.3557;
	classifiers[1].binary_classifier[1].mu[4] = 1.4302;
	classifiers[1].binary_classifier[1].mu[5] = -1.0119;
	classifiers[1].binary_classifier[1].mu[6] = 0.0730;
	classifiers[1].binary_classifier[1].mu[7] = 0.2477;
	classifiers[1].binary_classifier[1].mu[8] = 0.0910;
	classifiers[1].binary_classifier[1].mu[9] = 1.8036;
	classifiers[1].binary_classifier[1].mu[10] = 1.3034;
	classifiers[1].binary_classifier[1].mu[11] = -2.1533;

	classifiers[1].binary_classifier[1].sigma[0] = 0.0950;
	classifiers[1].binary_classifier[1].sigma[1] = 0.0833;
	classifiers[1].binary_classifier[1].sigma[2] = 1.2736;
	classifiers[1].binary_classifier[1].sigma[3] = 0.7257;
	classifiers[1].binary_classifier[1].sigma[4] = 0.1900;
	classifiers[1].binary_classifier[1].sigma[5] = 2.3175;
	classifiers[1].binary_classifier[1].sigma[6] = 0.1901;
	classifiers[1].binary_classifier[1].sigma[7] = 0.1289;
	classifiers[1].binary_classifier[1].sigma[8] = 1.2865;
	classifiers[1].binary_classifier[1].sigma[9] = 0.5758;
	classifiers[1].binary_classifier[1].sigma[10] = 0.1956;
	classifiers[1].binary_classifier[1].sigma[11] = 1.5707;

	classifiers[1].binary_classifier[1].kernel_scale = 3.3005;
	classifiers[1].binary_classifier[1].bias = 0.8540;

	// binary Classifier3
	classifiers[1].binary_classifier[2].beta[0] = -0.8114;
	classifiers[1].binary_classifier[2].beta[1] = -1.7557;
	classifiers[1].binary_classifier[2].beta[2] = -0.2193;
	classifiers[1].binary_classifier[2].beta[3] = -0.6856;
	classifiers[1].binary_classifier[2].beta[4] = 0.3855;
	classifiers[1].binary_classifier[2].beta[5] = -0.1773;
	classifiers[1].binary_classifier[2].beta[6] = -1.2607;
	classifiers[1].binary_classifier[2].beta[7] = 0.8608;
	classifiers[1].binary_classifier[2].beta[8] = -2.9061;
	classifiers[1].binary_classifier[2].beta[9] = 0.4443;
	classifiers[1].binary_classifier[2].beta[10] = -0.9538;
	classifiers[1].binary_classifier[2].beta[11] = 0.4038;

	classifiers[1].binary_classifier[2].mu[0] = 0.0773;
	classifiers[1].binary_classifier[2].mu[1] = 0.1750;
	classifiers[1].binary_classifier[2].mu[2] = 0.5769;
	classifiers[1].binary_classifier[2].mu[3] = 2.2621;
	classifiers[1].binary_classifier[2].mu[4] = 1.7273;
	classifiers[1].binary_classifier[2].mu[5] = -1.7356;
	classifiers[1].binary_classifier[2].mu[6] = 0.0935;
	classifiers[1].binary_classifier[2].mu[7] = 0.3863;
	classifiers[1].binary_classifier[2].mu[8] = -0.0307;
	classifiers[1].binary_classifier[2].mu[9] = 2.9884;
	classifiers[1].binary_classifier[2].mu[10] = 1.4857;
	classifiers[1].binary_classifier[2].mu[11] = -4.2138;

	classifiers[1].binary_classifier[2].sigma[0] = 0.0365;
	classifiers[1].binary_classifier[2].sigma[1] = 0.0688;
	classifiers[1].binary_classifier[2].sigma[2] = 0.5579;
	classifiers[1].binary_classifier[2].sigma[3] = 0.7977;
	classifiers[1].binary_classifier[2].sigma[4] = 0.2294;
	classifiers[1].binary_classifier[2].sigma[5] = 1.7555;
	classifiers[1].binary_classifier[2].sigma[6] = 0.0801;
	classifiers[1].binary_classifier[2].sigma[7] = 0.1344;
	classifiers[1].binary_classifier[2].sigma[8] = 0.6729;
	classifiers[1].binary_classifier[2].sigma[9] = 0.8113;
	classifiers[1].binary_classifier[2].sigma[10] = 0.1827;
	classifiers[1].binary_classifier[2].sigma[11] = 1.7699;

	classifiers[1].binary_classifier[2].kernel_scale = 1.8139;
	classifiers[1].binary_classifier[2].bias = 1.3897;

	// Classifier at 50% of swing time
	classifiers[2].n = 18;
	// binary Classifier1
	classifiers[2].binary_classifier[0].beta[0] = -1.8204;
	classifiers[2].binary_classifier[0].beta[1] = -1.0318;
	classifiers[2].binary_classifier[0].beta[2] = -0.7231;
	classifiers[2].binary_classifier[0].beta[3] = 1.0312;
	classifiers[2].binary_classifier[0].beta[4] = 0.3209;
	classifiers[2].binary_classifier[0].beta[5] = -0.7308;
	classifiers[2].binary_classifier[0].beta[6] = -1.1080;
	classifiers[2].binary_classifier[0].beta[7] = -0.0820;
	classifiers[2].binary_classifier[0].beta[8] = -0.5743;
	classifiers[2].binary_classifier[0].beta[9] = 0.9679;
	classifiers[2].binary_classifier[0].beta[10] = -0.3732;
	classifiers[2].binary_classifier[0].beta[11] = -0.7739;
	classifiers[2].binary_classifier[0].beta[12] = -1.2591;
	classifiers[2].binary_classifier[0].beta[13] = -0.1824;
	classifiers[2].binary_classifier[0].beta[14] = -2.9888;
	classifiers[2].binary_classifier[0].beta[15] = -1.3087;
	classifiers[2].binary_classifier[0].beta[16] = 0.5678;
	classifiers[2].binary_classifier[0].beta[17] = 3.9487;

	for (int i = 0; i<12; i++)
	{
		classifiers[2].binary_classifier[0].mu[i] = classifiers[1].binary_classifier[0].mu[i];
		classifiers[2].binary_classifier[0].sigma[i] = classifiers[1].binary_classifier[0].sigma[i];
	}

	classifiers[2].binary_classifier[0].mu[12] = -0.0079;
	classifiers[2].binary_classifier[0].mu[13] = 0.6659;
	classifiers[2].binary_classifier[0].mu[14] = -0.4547;
	classifiers[2].binary_classifier[0].mu[15] = 3.2885;
	classifiers[2].binary_classifier[0].mu[16] = 1.0708;
	classifiers[2].binary_classifier[0].mu[17] = -5.3574;

	classifiers[2].binary_classifier[0].sigma[12] = 0.0902;
	classifiers[2].binary_classifier[0].sigma[13] = 0.1507;
	classifiers[2].binary_classifier[0].sigma[14] = 0.3019;
	classifiers[2].binary_classifier[0].sigma[15] = 0.9185;
	classifiers[2].binary_classifier[0].sigma[16] = 0.1838;
	classifiers[2].binary_classifier[0].sigma[17] = 2.0129;

	classifiers[2].binary_classifier[0].kernel_scale = 5.8100;
	classifiers[2].binary_classifier[0].bias = -0.9607;

	// binary Classifier2
	classifiers[2].binary_classifier[1].beta[0] = -1.5262;
	classifiers[2].binary_classifier[1].beta[1] = -0.7552;
	classifiers[2].binary_classifier[1].beta[2] = -1.3348;
	classifiers[2].binary_classifier[1].beta[3] = 0.3727;
	classifiers[2].binary_classifier[1].beta[4] = 0.0424;
	classifiers[2].binary_classifier[1].beta[5] = -1.3652;
	classifiers[2].binary_classifier[1].beta[6] = -1.7616;
	classifiers[2].binary_classifier[1].beta[7] = -0.3877;
	classifiers[2].binary_classifier[1].beta[8] = -2.3471;
	classifiers[2].binary_classifier[1].beta[9] = 0.1885;
	classifiers[2].binary_classifier[1].beta[10] = -0.1399;
	classifiers[2].binary_classifier[1].beta[11] = 0.5521;
	classifiers[2].binary_classifier[1].beta[12] = -2.1166;
	classifiers[2].binary_classifier[1].beta[13] = -0.1135;
	classifiers[2].binary_classifier[1].beta[14] = -3.2085;
	classifiers[2].binary_classifier[1].beta[15] = -0.3087;
	classifiers[2].binary_classifier[1].beta[16] = 0.4202;
	classifiers[2].binary_classifier[1].beta[17] = 0.7642;

	for (int i = 0; i<12; i++)
	{
		classifiers[2].binary_classifier[1].mu[i] = classifiers[1].binary_classifier[1].mu[i];
		classifiers[2].binary_classifier[1].sigma[i] = classifiers[1].binary_classifier[1].sigma[i];
	}

	classifiers[2].binary_classifier[1].mu[12] = 0.0686;
	classifiers[2].binary_classifier[1].mu[13] = 0.3822;
	classifiers[2].binary_classifier[1].mu[14] = -0.1590;
	classifiers[2].binary_classifier[1].mu[15] = 1.8222;
	classifiers[2].binary_classifier[1].mu[16] = 1.1298;
	classifiers[2].binary_classifier[1].mu[17] = -2.3862;

	classifiers[2].binary_classifier[1].sigma[12] = 0.2619;
	classifiers[2].binary_classifier[1].sigma[13] = 0.1525;
	classifiers[2].binary_classifier[1].sigma[14] = 0.8607;
	classifiers[2].binary_classifier[1].sigma[15] = 0.6480;
	classifiers[2].binary_classifier[1].sigma[16] = 0.1922;
	classifiers[2].binary_classifier[1].sigma[17] = 1.5044;

	classifiers[2].binary_classifier[1].kernel_scale = 4.2291;
	classifiers[2].binary_classifier[1].bias = 0.6706;

	// binary Classifier3
	classifiers[2].binary_classifier[2].beta[0] = -1.4981;
	classifiers[2].binary_classifier[2].beta[1] = -1.5265;
	classifiers[2].binary_classifier[2].beta[2] = -0.1699;
	classifiers[2].binary_classifier[2].beta[3] = -0.5338;
	classifiers[2].binary_classifier[2].beta[4] = 0.3653;
	classifiers[2].binary_classifier[2].beta[5] = -0.2014;
	classifiers[2].binary_classifier[2].beta[6] = -1.2028;
	classifiers[2].binary_classifier[2].beta[7] = 0.0670;
	classifiers[2].binary_classifier[2].beta[8] = -1.3652;
	classifiers[2].binary_classifier[2].beta[9] = 0.1455;
	classifiers[2].binary_classifier[2].beta[10] = -0.7164;
	classifiers[2].binary_classifier[2].beta[11] = 0.4965;
	classifiers[2].binary_classifier[2].beta[12] = -1.5194;
	classifiers[2].binary_classifier[2].beta[13] = 0.7728;
	classifiers[2].binary_classifier[2].beta[14] = -1.9827;
	classifiers[2].binary_classifier[2].beta[15] = -0.0422;
	classifiers[2].binary_classifier[2].beta[16] = -1.0547;
	classifiers[2].binary_classifier[2].beta[17] = 0.3429;

	for (int i = 0; i<12; i++)
	{
		classifiers[2].binary_classifier[2].mu[i] = classifiers[1].binary_classifier[2].mu[i];
		classifiers[2].binary_classifier[2].sigma[i] = classifiers[1].binary_classifier[2].sigma[i];
	}

	classifiers[2].binary_classifier[2].mu[12] = 0.0804;
	classifiers[2].binary_classifier[2].mu[13] = 0.6381;
	classifiers[2].binary_classifier[2].mu[14] = -0.1811;
	classifiers[2].binary_classifier[2].mu[15] = 3.3420;
	classifiers[2].binary_classifier[2].mu[16] = 1.0844;
	classifiers[2].binary_classifier[2].mu[17] = -5.4998;

	classifiers[2].binary_classifier[2].sigma[12] = 0.1175;
	classifiers[2].binary_classifier[2].sigma[13] = 0.2022;
	classifiers[2].binary_classifier[2].sigma[14] = 0.4358;
	classifiers[2].binary_classifier[2].sigma[15] = 0.7997;
	classifiers[2].binary_classifier[2].sigma[16] = 0.1905;
	classifiers[2].binary_classifier[2].sigma[17] = 1.7016;

	classifiers[2].binary_classifier[2].kernel_scale = 2.5173;
	classifiers[2].binary_classifier[2].bias = 1.3632;

	// Classifier at 60% of swing time
	classifiers[3].n = 23;
	// binary Classifier1
	classifiers[3].binary_classifier[0].beta[0] = -1.9930;
	classifiers[3].binary_classifier[0].beta[1] = -0.7051;
	classifiers[3].binary_classifier[0].beta[2] = -0.9598;
	classifiers[3].binary_classifier[0].beta[3] = 1.0355;
	classifiers[3].binary_classifier[0].beta[4] = 0.3565;
	classifiers[3].binary_classifier[0].beta[5] = -1.1540;
	classifiers[3].binary_classifier[0].beta[6] = -1.3899;
	classifiers[3].binary_classifier[0].beta[7] = 0.1382;
	classifiers[3].binary_classifier[0].beta[8] = -0.8352;
	classifiers[3].binary_classifier[0].beta[9] = 1.0677;
	classifiers[3].binary_classifier[0].beta[10] = -0.4021;
	classifiers[3].binary_classifier[0].beta[11] = -0.6393;
	classifiers[3].binary_classifier[0].beta[12] = -1.3101;
	classifiers[3].binary_classifier[0].beta[13] = 0.1535;
	classifiers[3].binary_classifier[0].beta[14] = -1.2807;
	classifiers[3].binary_classifier[0].beta[15] = -0.0414;
	classifiers[3].binary_classifier[0].beta[16] = 0.2368;
	classifiers[3].binary_classifier[0].beta[17] = 2.4596;
	classifiers[3].binary_classifier[0].beta[18] = -1.6988;
	classifiers[3].binary_classifier[0].beta[19] = -0.2508;
	classifiers[3].binary_classifier[0].beta[20] = -3.5469;
	classifiers[3].binary_classifier[0].beta[21] = 1.5422;
	classifiers[3].binary_classifier[0].beta[22] = 2.2309;

	for (int i = 0; i<18; i++)
	{
		classifiers[3].binary_classifier[0].mu[i] = classifiers[2].binary_classifier[0].mu[i];
		classifiers[3].binary_classifier[0].sigma[i] = classifiers[2].binary_classifier[0].sigma[i];
	}

	classifiers[3].binary_classifier[0].mu[18] = -0.0206;
	classifiers[3].binary_classifier[0].mu[19] = 0.9108;
	classifiers[3].binary_classifier[0].mu[20] = 0.1262;
	classifiers[3].binary_classifier[0].mu[21] = 0.6652;
	classifiers[3].binary_classifier[0].mu[22] = -4.4933;

	classifiers[3].binary_classifier[0].sigma[18] = 0.1117;
	classifiers[3].binary_classifier[0].sigma[19] = 0.2251;
	classifiers[3].binary_classifier[0].sigma[20] = 0.4901;
	classifiers[3].binary_classifier[0].sigma[21] = 0.2428;
	classifiers[3].binary_classifier[0].sigma[22] = 2.3607;

	classifiers[3].binary_classifier[0].kernel_scale = 6.5011;
	classifiers[3].binary_classifier[0].bias = -1.1339;

	// binary Classifier2
	classifiers[3].binary_classifier[1].beta[0] = -1.8223;
	classifiers[3].binary_classifier[1].beta[1] = -0.9339;
	classifiers[3].binary_classifier[1].beta[2] = -1.3246;
	classifiers[3].binary_classifier[1].beta[3] = 0.2843;
	classifiers[3].binary_classifier[1].beta[4] = -0.0549;
	classifiers[3].binary_classifier[1].beta[5] = -1.0424;
	classifiers[3].binary_classifier[1].beta[6] = -1.7369;
	classifiers[3].binary_classifier[1].beta[7] = -0.4612;
	classifiers[3].binary_classifier[1].beta[8] = -1.7816;
	classifiers[3].binary_classifier[1].beta[9] = 0.2649;
	classifiers[3].binary_classifier[1].beta[10] = -0.3290;
	classifiers[3].binary_classifier[1].beta[11] = 0.4166;
	classifiers[3].binary_classifier[1].beta[12] = -1.8257;
	classifiers[3].binary_classifier[1].beta[13] = -0.1504;
	classifiers[3].binary_classifier[1].beta[14] = -1.9216;
	classifiers[3].binary_classifier[1].beta[15] = 0.2657;
	classifiers[3].binary_classifier[1].beta[16] = -0.0156;
	classifiers[3].binary_classifier[1].beta[17] = 0.0233;
	classifiers[3].binary_classifier[1].beta[18] = -1.8906;
	classifiers[3].binary_classifier[1].beta[19] = 0.0356;
	classifiers[3].binary_classifier[1].beta[20] = -1.7482;
	classifiers[3].binary_classifier[1].beta[21] = 0.5698;
	classifiers[3].binary_classifier[1].beta[22] = 0.3702;

	for (int i = 0; i<18; i++)
	{
		classifiers[3].binary_classifier[1].mu[i] = classifiers[2].binary_classifier[1].mu[i];
		classifiers[3].binary_classifier[1].sigma[i] = classifiers[2].binary_classifier[1].sigma[i];
	}

	classifiers[3].binary_classifier[1].mu[18] = 0.0498;
	classifiers[3].binary_classifier[1].mu[19] = 0.4971;
	classifiers[3].binary_classifier[1].mu[20] = -0.3215;
	classifiers[3].binary_classifier[1].mu[21] = 0.9621;
	classifiers[3].binary_classifier[1].mu[22] = -1.8726;

	classifiers[3].binary_classifier[1].sigma[18] = 0.3011;
	classifiers[3].binary_classifier[1].sigma[19] = 0.1612;
	classifiers[3].binary_classifier[1].sigma[20] = 0.4954;
	classifiers[3].binary_classifier[1].sigma[21] = 0.2026;
	classifiers[3].binary_classifier[1].sigma[22] = 2.3576;

	classifiers[3].binary_classifier[1].kernel_scale = 4.6738;
	classifiers[3].binary_classifier[1].bias = 0.5966;

	// binary Classifier3
	classifiers[3].binary_classifier[2].beta[0] = -1.5119;
	classifiers[3].binary_classifier[2].beta[1] = -1.3700;
	classifiers[3].binary_classifier[2].beta[2] = -0.5737;
	classifiers[3].binary_classifier[2].beta[3] = -0.4661;
	classifiers[3].binary_classifier[2].beta[4] = 0.3414;
	classifiers[3].binary_classifier[2].beta[5] = -0.1743;
	classifiers[3].binary_classifier[2].beta[6] = -1.3157;
	classifiers[3].binary_classifier[2].beta[7] = -0.1518;
	classifiers[3].binary_classifier[2].beta[8] = -1.4355;
	classifiers[3].binary_classifier[2].beta[9] = 0.0082;
	classifiers[3].binary_classifier[2].beta[10] = -0.6835;
	classifiers[3].binary_classifier[2].beta[11] = 0.3053;
	classifiers[3].binary_classifier[2].beta[12] = -1.4875;
	classifiers[3].binary_classifier[2].beta[13] = 0.3290;
	classifiers[3].binary_classifier[2].beta[14] = -1.6165;
	classifiers[3].binary_classifier[2].beta[15] = -0.3607;
	classifiers[3].binary_classifier[2].beta[16] = -0.6986;
	classifiers[3].binary_classifier[2].beta[17] = 0.6499;
	classifiers[3].binary_classifier[2].beta[18] = -1.3832;
	classifiers[3].binary_classifier[2].beta[19] = 0.4071;
	classifiers[3].binary_classifier[2].beta[20] = 0.0413;
	classifiers[3].binary_classifier[2].beta[21] = -0.8463;
	classifiers[3].binary_classifier[2].beta[22] = -0.5260;

	for (int i = 0; i<18; i++)
	{
		classifiers[3].binary_classifier[2].mu[i] = classifiers[2].binary_classifier[2].mu[i];
		classifiers[3].binary_classifier[2].sigma[i] = classifiers[2].binary_classifier[2].sigma[i];
	}

	classifiers[3].binary_classifier[2].mu[18] = 0.0818;
	classifiers[3].binary_classifier[2].mu[19] = 0.8940;
	classifiers[3].binary_classifier[2].mu[20] = 0.2571;
	classifiers[3].binary_classifier[2].mu[21] = 0.6558;
	classifiers[3].binary_classifier[2].mu[22] = -4.8559;

	classifiers[3].binary_classifier[2].sigma[18] = 0.1276;
	classifiers[3].binary_classifier[2].sigma[19] = 0.2621;
	classifiers[3].binary_classifier[2].sigma[20] = 0.3171;
	classifiers[3].binary_classifier[2].sigma[21] = 0.2340;
	classifiers[3].binary_classifier[2].sigma[22] = 1.6009;

	classifiers[3].binary_classifier[2].kernel_scale = 3.2517;
	classifiers[3].binary_classifier[2].bias = 1.3632;

	// Classifier at 70% of swing time
	classifiers[4].n = 25;
	// binary Classifier1
	classifiers[4].binary_classifier[0].beta[0] = -1.8841;
	classifiers[4].binary_classifier[0].beta[1] = -1.0134;
	classifiers[4].binary_classifier[0].beta[2] = -0.6060;
	classifiers[4].binary_classifier[0].beta[3] = 1.2579;
	classifiers[4].binary_classifier[0].beta[4] = -0.1227;
	classifiers[4].binary_classifier[0].beta[5] = -1.1116;
	classifiers[4].binary_classifier[0].beta[6] = -1.1547;
	classifiers[4].binary_classifier[0].beta[7] = -0.0922;
	classifiers[4].binary_classifier[0].beta[8] = -0.4818;
	classifiers[4].binary_classifier[0].beta[9] = 1.1201;
	classifiers[4].binary_classifier[0].beta[10] = -0.6547;
	classifiers[4].binary_classifier[0].beta[11] = -0.4778;
	classifiers[4].binary_classifier[0].beta[12] = -1.0863;
	classifiers[4].binary_classifier[0].beta[13] = 0.0551;
	classifiers[4].binary_classifier[0].beta[14] = -1.5114;
	classifiers[4].binary_classifier[0].beta[15] = 0.3005;
	classifiers[4].binary_classifier[0].beta[16] = -0.0136;
	classifiers[4].binary_classifier[0].beta[17] = 2.2630;
	classifiers[4].binary_classifier[0].beta[18] = -1.4860;
	classifiers[4].binary_classifier[0].beta[19] = -0.1526;
	classifiers[4].binary_classifier[0].beta[20] = -3.2086;
	classifiers[4].binary_classifier[0].beta[21] = 1.1313;
	classifiers[4].binary_classifier[0].beta[22] = 2.0519;
	classifiers[4].binary_classifier[0].beta[23] = -2.7432;
	classifiers[4].binary_classifier[0].beta[24] = -0.5915;

	for (int i = 0; i<23; i++)
	{
		classifiers[4].binary_classifier[0].mu[i] = classifiers[3].binary_classifier[0].mu[i];
		classifiers[4].binary_classifier[0].sigma[i] = classifiers[3].binary_classifier[0].sigma[i];
	}

	classifiers[4].binary_classifier[0].mu[23] = -0.0072;
	classifiers[4].binary_classifier[0].mu[24] = 1.0673;

	classifiers[4].binary_classifier[0].sigma[23] = 0.1368;
	classifiers[4].binary_classifier[0].sigma[24] = 0.2807;

	classifiers[4].binary_classifier[0].kernel_scale = 7.1529;
	classifiers[4].binary_classifier[0].bias = -1.1065;

	// binary Classifier2
	classifiers[4].binary_classifier[1].beta[0] = -1.8419;
	classifiers[4].binary_classifier[1].beta[1] = -0.8250;
	classifiers[4].binary_classifier[1].beta[2] = -1.3952;
	classifiers[4].binary_classifier[1].beta[3] = 0.2250;
	classifiers[4].binary_classifier[1].beta[4] = -0.1549;
	classifiers[4].binary_classifier[1].beta[5] = -1.0388;
	classifiers[4].binary_classifier[1].beta[6] = -1.7723;
	classifiers[4].binary_classifier[1].beta[7] = -0.4676;
	classifiers[4].binary_classifier[1].beta[8] = -1.7883;
	classifiers[4].binary_classifier[1].beta[9] = 0.2777;
	classifiers[4].binary_classifier[1].beta[10] = -0.3026;
	classifiers[4].binary_classifier[1].beta[11] = 0.3596;
	classifiers[4].binary_classifier[1].beta[12] = -1.8464;
	classifiers[4].binary_classifier[1].beta[13] = -0.2489;
	classifiers[4].binary_classifier[1].beta[14] = -1.9178;
	classifiers[4].binary_classifier[1].beta[15] = 0.2869;
	classifiers[4].binary_classifier[1].beta[16] = 0.0456;
	classifiers[4].binary_classifier[1].beta[17] = 0.0341;
	classifiers[4].binary_classifier[1].beta[18] = -1.9095;
	classifiers[4].binary_classifier[1].beta[19] = -0.0624;
	classifiers[4].binary_classifier[1].beta[20] = -1.7619;
	classifiers[4].binary_classifier[1].beta[21] = 0.6191;
	classifiers[4].binary_classifier[1].beta[22] = 0.4280;
	classifiers[4].binary_classifier[1].beta[23] = -2.0998;
	classifiers[4].binary_classifier[1].beta[24] = 0.0951;

	for (int i = 0; i<23; i++)
	{
		classifiers[4].binary_classifier[1].mu[i] = classifiers[3].binary_classifier[1].mu[i];
		classifiers[4].binary_classifier[1].sigma[i] = classifiers[3].binary_classifier[1].sigma[i];
	}

	classifiers[4].binary_classifier[1].mu[23] = 0.0231;
	classifiers[4].binary_classifier[1].mu[24] = 0.5583;

	classifiers[4].binary_classifier[1].sigma[23] = 0.3166;
	classifiers[4].binary_classifier[1].sigma[24] = 0.1735;

	classifiers[4].binary_classifier[1].kernel_scale = 5.5933;
	classifiers[4].binary_classifier[1].bias = 0.6043;

	// binary Classifier3
	classifiers[4].binary_classifier[2].beta[0] = -1.2947;
	classifiers[4].binary_classifier[2].beta[1] = -1.4235;
	classifiers[4].binary_classifier[2].beta[2] = -0.2284;
	classifiers[4].binary_classifier[2].beta[3] = -0.5233;
	classifiers[4].binary_classifier[2].beta[4] = 0.2980;
	classifiers[4].binary_classifier[2].beta[5] = -0.2306;
	classifiers[4].binary_classifier[2].beta[6] = -1.0632;
	classifiers[4].binary_classifier[2].beta[7] = -0.1690;
	classifiers[4].binary_classifier[2].beta[8] = -1.2511;
	classifiers[4].binary_classifier[2].beta[9] = -0.0150;
	classifiers[4].binary_classifier[2].beta[10] = -0.7972;
	classifiers[4].binary_classifier[2].beta[11] = 0.4369;
	classifiers[4].binary_classifier[2].beta[12] = -1.2736;
	classifiers[4].binary_classifier[2].beta[13] = 0.3316;
	classifiers[4].binary_classifier[2].beta[14] = -1.4729;
	classifiers[4].binary_classifier[2].beta[15] = -0.3244;
	classifiers[4].binary_classifier[2].beta[16] = -0.7191;
	classifiers[4].binary_classifier[2].beta[17] = 0.6547;
	classifiers[4].binary_classifier[2].beta[18] = -1.2001;
	classifiers[4].binary_classifier[2].beta[19] = 0.4761;
	classifiers[4].binary_classifier[2].beta[20] = 0.0171;
	classifiers[4].binary_classifier[2].beta[21] = -0.9283;
	classifiers[4].binary_classifier[2].beta[22] = -0.4367;
	classifiers[4].binary_classifier[2].beta[23] = -1.4675;
	classifiers[4].binary_classifier[2].beta[24] = 0.2812;

	for (int i = 0; i<23; i++)
	{
		classifiers[4].binary_classifier[2].mu[i] = classifiers[3].binary_classifier[2].mu[i];
		classifiers[4].binary_classifier[2].sigma[i] = classifiers[3].binary_classifier[2].sigma[i];
	}

	classifiers[4].binary_classifier[2].mu[23] = 0.1011;
	classifiers[4].binary_classifier[2].mu[24] = 1.0664;

	classifiers[4].binary_classifier[2].sigma[23] = 0.1157;
	classifiers[4].binary_classifier[2].sigma[24] = 0.2885;

	classifiers[4].binary_classifier[2].kernel_scale = 2.9272;
	classifiers[4].binary_classifier[2].bias = 1.4208;
}
