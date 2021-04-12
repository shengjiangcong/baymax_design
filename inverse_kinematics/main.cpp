#include <iostream>
#include "robcprobot_arm6DOFik.h"

using namespace std;

int main()
{
	const double PI = 3.141592653;
	float alength[6] = { 0, 289.49, 77.58, 0, 0, 0 };//DH Table -a
	float alpha[6] = { PI / 2, 0, PI / 2, -PI / 2, PI / 2, 0 };//DH Table -alpha
	float dlength[6] = { 267, 0, 0, 342.38, 0, -123.22 };//DH Table -d
	float theta0[6] = { 0 };//DH Table -theta
	float angleOffset[6] = { 0, 0, -PI / 2, 0, -PI / 2, 0};
	float highlim[6] = { 2 * PI, 2 * PI, 2 * PI, 2 * PI, 2 * PI, 2 * PI };
	float lowlim[6] = { -2 * PI, -2 * PI, -2 * PI, -2 * PI, -2 * PI, -2 * PI };
	float JointWeight[6] = { 1, 0.6, 0.6, 0.3, 0.3, 0.3 };

	robc_ARM6DOF_set_geometry(alength, alpha, dlength, theta0);
	robc_ARM6DOF_set_angleoffset(angleOffset);
	robc_ARM6DOF_set_movelim(highlim, lowlim);
	robc_ARM6DOF_set_modle(0);
	robc_ARM6DOF_set_jointweight(JointWeight);
	float dest[6] = { -0.65, 1.56, -1.56, 1.05, 0.34, 1.83};//0  -0.18623  -1.38456  0  -0.9057  0
	dest[1] = -1 * dest[1] - 0.18623;
	dest[2] = -1 * dest[2] - 1.38456;
	dest[4] = -1 * dest[4] - 0.9057;


	//d1 = j1
	//d2 = -j2
	//d3 = -1.58 - j3
	//d4 = j4
	//d5 = 1.57 - j5
	robc_ARM6DOF_Anno_kicalc(dest);
	float dest1[6] = { 283.6, -258.5, -242.7, -3.02, -0.08, 0.315 };

	robc_ARM6DOF_set_jointval(dest);
	robc_ARM6DOF_set_modle(0);

	cout << robc_ARM6DOF_Anno_ikcalc(dest1) << endl;

	return 0;
}