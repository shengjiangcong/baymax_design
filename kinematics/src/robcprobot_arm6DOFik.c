/** ---------------------------------------------------------------------------
 * Copyright (c) 2017, PS-Micro, Co. Ltd.  All rights reserved.
 *
 * @file    <robcprobot_ARM6DOFik.c>    by yaoym
 * @date    2019-05-17
 * @version 0.1.2  2019.05
---------------------------------------------------------------------------- */
/**
 * @file
 *   This file includes ARM6DOF kinematics and inverse kinematics functions.
 */

/**
 * @ingroup
 */

#include "kinematics/robcprobot_arm6DOFik.h"
#include "kinematics/robcprobot_matrix.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <float.h>

#define PI 3.1415926535897932384626433832795
#define PI_2 1.5707963267948966192313216916398
#define PI2 6.283185307179586476925286766559

#define ANG2RAD_EQU(N) (N *= (180.0/3.1415926535898) )
#define ANG2RAD(N) ( (N) * (180.0/3.1415926535898) )
#define RAD2ANG (3.1415926535898/180.0)
#define IS_ZERO(var) if(var < 0.0000000001 && var > -0.0000000001){var = 0;} 
#define JUDGE_ZERO(var) ( (var) < 0.0000000001 && (var) > -0.0000000001 )

#define MATRIX_1 1
#define MATRIX_M 4
#define MATRIX_N 4

#define JOINT_OFFSET_J2 1.5707963267949
#define JOINT_OFFSET_J3 1.5707963267949

 /* ----------------------------------------------------------------------------
 *   Static Parameter
 ---------------------------------------------------------------------------- */

/**
 * Structure for ARM6DOF IK
 *
 */
typedef struct
{
    double DHparam[6][4];                         /* ARM6DOF geometry, { unit: [pulse/ms2], range: (0, 32767) } */
    double DestPos[6];                         /* Destination position , { unit: [pulse/ms2], range: (0, 32767) } */
	double PosVal[6];                         /* Destination position , { unit: [pulse/ms2], range: (0, 32767) } */
    double JointVal[6];                   /* Target start speed, { unit: [pulse/ms], range: [0, 32767) } */
	double JointValLimH[6];                   /* Target start speed, { unit: [pulse/ms], range: [0, 32767) } */
	double JointValLimL[6];                   /* Target start speed, { unit: [pulse/ms], range: [0, 32767) } */
	double JointValDest[6];                   /* Target start speed, { unit: [pulse/ms], range: [0, 32767) } */
	float JointWeight[6];                   /* Target start speed, { unit: [pulse/ms], range: [0, 32767) } */
	float angleOffset[6];
	int PostureModle;
	int IsGoodSolution[8];
} ROBCP_ARM6DOF_PARAM;

static ROBCP_ARM6DOF_PARAM ARM6DOF_para = 
{ { { 0,				PI / 2,		230,		0 },
	 { 262,		0,				0,				0 } ,
	 { 0,				PI / 2,		0,				0 } ,
	 { 0,				-PI / 2,		228.85535,	0 } ,
	 { 0,				PI / 2,		0,				0 } ,
	 { 0,				0,				55,			0 } }, 
	 {100, 100, 100, 0, 0, 0} ,  {0, 0, 0, 0, 0, 0} , { 0, 0, 0, 0, 0, 0},  {PI, PI , PI , PI , PI , PI } , {-PI, -PI , PI , -PI , -PI , -PI },
   {0}, { 1, 1, 1, .3, .3, .3 }, {0},  0, {1} };

/* ----------------------------------------------------------------------------
   *   Static Function
---------------------------------------------------------------------------- */
/**
* \brief       trap parameter check
*
* \param[in]   *axis_param      structure  MOTNP_TRAP_AXIS_PARAM
* \return      Return value, see return value for details
*/
static  void fun_zyz(double matrix_R[MATRIX_N-1][MATRIX_N-1], double *p_r, double *p_p, double *p_y)
{
	double mtmp = sqrt(pow(matrix_R[1][2],2) + pow(matrix_R[0][2],2));

	//  printf("ZYZ  \n--- > -pi  and < 0\n");
	p_r[0] = atan2(matrix_R[1][2], matrix_R[0][2]);
	p_p[0] = atan2(mtmp, matrix_R[2][2]);
	p_y[0] = atan2(matrix_R[2][1], -matrix_R[2][0]);

	//  printf("ZYZ  \n--- > -pi  and < 0\n");
	p_r[1] = atan2(-matrix_R[1][2], -matrix_R[0][2]);
	p_p[1] = atan2(-mtmp, matrix_R[2][2]);
	p_y[1] = atan2(-matrix_R[2][1], matrix_R[2][0]);
}

/**
* \brief       trap parameter check
*
* \param[in]   *axis_param      structure  MOTNP_TRAP_AXIS_PARAM
* \return      Return value, see return value for details
*/
static void calculate_matrix_A(double matrix[MATRIX_N][MATRIX_N], double *p_param)
{//根据关节参数计算矩阵
	double *pmatrix = (double *)matrix;
	double var_c, var_s, angle_c, angle_s;

	var_c = cos(p_param[3]);
	IS_ZERO(var_c);
	var_s = sin(p_param[3]);
	IS_ZERO(var_s);
	angle_c = cos(p_param[1]);
	IS_ZERO(angle_c);
	angle_s = sin(p_param[1]);
	IS_ZERO(angle_s);

	*pmatrix++ = var_c;
	*pmatrix++ = -var_s * angle_c;
	*pmatrix++ = var_s * angle_s;
	*pmatrix++ = p_param[0] * var_c;

	*pmatrix++ = var_s;
	*pmatrix++ = var_c * angle_c;
	*pmatrix++ = -var_c *angle_s;
	*pmatrix++ = p_param[0] * var_s;

	*pmatrix++ = 0;
	*pmatrix++ = angle_s;
	*pmatrix++ = angle_c;
	*pmatrix++ = p_param[2];

	*pmatrix++ = 0;
	*pmatrix++ = 0;
	*pmatrix++ = 0;
	*pmatrix = 1;
}

/**
* \brief       trap parameter check
*
* \param[in]   *axis_param      structure  MOTNP_TRAP_AXIS_PARAM
* \return      Return value, see return value for details
*/
static void calculate_matrix_R(double angle_r, double angle_p, double angle_y, double(*matrix_R)[MATRIX_N])
{/*计算旋转矩阵 */
	double r_c, r_s, p_c, p_s, y_c, y_s;

	r_c = cos(angle_r);
	IS_ZERO(r_c);
	r_s = sin(angle_r);
	IS_ZERO(r_s);
	p_c = cos(angle_p);
	IS_ZERO(p_c);
	p_s = sin(angle_p);
	IS_ZERO(p_s);
	y_c = cos(angle_y);
	IS_ZERO(p_c);
	y_s = sin(angle_y);
	IS_ZERO(y_s);

	matrix_R[0][0] = r_c * p_c;
	matrix_R[0][1] = r_c * p_s * y_s - r_s * y_c;
	matrix_R[0][2] = r_c * p_s * y_c + r_s * y_s;

	matrix_R[1][0] = r_s * p_c;
	matrix_R[1][1] = r_s * p_s * y_s + r_c * y_c;
	matrix_R[1][2] = r_s * p_s * y_c - r_c * y_s;

	matrix_R[2][0] = -p_s;
	matrix_R[2][1] = p_c * y_s;
	matrix_R[2][2] = p_c * y_c;
}

/**
 * \brief       trap parameter check
 *
 * \param[in]   *axis_param      structure  MOTNP_TRAP_AXIS_PARAM
 * \return      Return value, see return value for details
 */
static double angleRange(double iAngle)
{
	double rAngle = 0;
	if (iAngle>PI)
		rAngle = iAngle-PI*2;
	else if (iAngle<-PI)
		rAngle = iAngle+PI*2;
	else
		rAngle = iAngle;
	return rAngle;
}

/**
 * \brief       trap parameter check
 *
 * \param[in]   *axis_param      structure  MOTNP_TRAP_AXIS_PARAM
 * \return      Return value, see return value for details
 */
static int8_t JointValPossible(void)
{
	int8_t IsOK = 1;
	int i = 0;
	for (i = 0; i<6; i++)
	{
		if (ARM6DOF_para.JointValDest[i]>=ARM6DOF_para.JointValLimH[i]|| 
			  ARM6DOF_para.JointValDest[i]<=ARM6DOF_para.JointValLimL[i])
			IsOK = 0;
	}
	return IsOK;
}

/**
* \brief       trap parameter check
*
* \param[in]   *axis_param      structure  MOTNP_TRAP_AXIS_PARAM
* \return      Return value, see return value for details
*/
static int16_t ARM6DOFik_j23calc(double param[4], double *ang1, double *ang2 )
{
	double arm1 = param[0]; 
	double arm2 = param[1];
	double posX = param[2];
	double posY = param[3];

	double TempR2, TempR;
	double TempA, TempB;
	double TempTheta11, TempTheta12;

	TempR2 = posX*posX + posY*posY;
	TempR = sqrt(TempR2);
	TempA = (arm1*arm1 - arm2*arm2 + TempR2) / (2 * arm1 * TempR);
	TempB = TempA*TempA;
	if (TempB <= 1)
	{
		TempB = sqrt(1 - TempB);
		TempTheta12 = atan2(posY, posX);
		TempTheta11 = atan2(TempB, TempA);
		*ang1 = angleRange(TempTheta11 + TempTheta12 - JOINT_OFFSET_J2);
		*ang2 = atan2(-TempR*sin(TempTheta11), (TempR*cos(TempTheta11) - arm1));

		TempTheta11 = -atan2(TempB, TempA);
		*(ang1 + 1) = angleRange(TempTheta11 + TempTheta12 - JOINT_OFFSET_J2);
		*(ang2 + 1) = atan2(-TempR*sin(TempTheta11), (TempR*cos(TempTheta11) - arm1));

		return 0;
	}
	else
		return -1;
}

/**
* \brief       trap parameter check
*
* \param[in]   *axis_param      structure  MOTNP_TRAP_AXIS_PARAM
* \return      Return value, see return value for details
*/
static int16_t ARM6DOFik_j456calc(double  j1, double j2, double j3, double p_matrix_R[MATRIX_N][MATRIX_N], double *p_j4, double *p_j5, double *p_j6)
{
	double matrix_a[MATRIX_N][MATRIX_N], matrix_b[MATRIX_N][MATRIX_N];
	double matrix_tmp[MATRIX_N][MATRIX_N];
	double matrix_am[MATRIX_N-1][MATRIX_N-1], matrix_bm[MATRIX_N-1][MATRIX_N-1];
	double matrix_rm[MATRIX_N - 1][MATRIX_N - 1];
	
	ARM6DOF_para.DHparam[0][3] = j1;
	ARM6DOF_para.DHparam[1][3] = j2 + JOINT_OFFSET_J2;
	ARM6DOF_para.DHparam[2][3] = j3 + JOINT_OFFSET_J3;

	calculate_matrix_A(matrix_a, *(ARM6DOF_para.DHparam + 0));
	calculate_matrix_A(matrix_b, *(ARM6DOF_para.DHparam + 1));
	matrix_mul(*matrix_a, *matrix_b, *matrix_tmp, MATRIX_N, MATRIX_N);

	calculate_matrix_A(matrix_b, *(ARM6DOF_para.DHparam + 2));
	matrix_mul(*matrix_tmp, *matrix_b, *matrix_a, MATRIX_N, MATRIX_N);

	matrix_remap(*matrix_a, *matrix_am, MATRIX_N, MATRIX_N, MATRIX_N - 1, MATRIX_N - 1);
	matrix_remap(*p_matrix_R, *matrix_rm, MATRIX_N, MATRIX_N, MATRIX_N - 1, MATRIX_N - 1);
	matrix_translate(*matrix_am, MATRIX_N-1, MATRIX_N-1);
	matrix_mul(*matrix_am, *matrix_rm, *matrix_bm, MATRIX_N-1, MATRIX_N-1);

	fun_zyz(matrix_bm, p_j4, p_j5, p_j6);

	return 1;
}

/**
* \brief       trap parameter check
*
* \param[in]   *axis_param      structure  MOTNP_TRAP_AXIS_PARAM
* \return      Return value, see return value for details
*/
static double ARM6DOFik_CalcuScore(void)
{
	double score = 0;
	int i;
	for (i = 0; i<6; i++)
		score += fabs(ARM6DOF_para.JointVal[i] - ARM6DOF_para.JointValDest[i])*ARM6DOF_para.JointWeight[i];
	return score;
}


/* ----------------------------------------------------------------------------
   *   External Function
---------------------------------------------------------------------------- */

/**
 * \brief       设置ARM6DOF几何参数
 *
 * \param[in]   float* Arm      指向存放ARM6DOF几何参数的数组[DH参数]
 * \return      程序正常，返回0
 */
int16_t robc_ARM6DOF_set_geometry(float alength[6], float alpha[6], float dlength[6], float theta0[6])
{
	int i=0;
	for (i = 0; i < 6; i++)
	{
		ARM6DOF_para.DHparam[i][0] = alength[i]; 
		ARM6DOF_para.DHparam[i][1] = alpha[i]; 
		ARM6DOF_para.DHparam[i][2] = dlength[i]; 
		ARM6DOF_para.DHparam[i][3] = theta0[i];
	}
	return 0;
}

/**
* \brief       设置ARM6DOF几何参数
*
* \param[in]   float* Arm      指向存放ARM6DOF几何参数的数组[DH参数]
* \return      程序正常，返回0
*/
int16_t robc_ARM6DOF_set_angleoffset(float angleoffset[6])
{
	int i = 0;
	for (i = 0; i < 6; i++)
		ARM6DOF_para.angleOffset[i] = angleoffset[i];
	return 0;
}

/**
 * \brief       设置ARM6DOF各个关节运动范围参数
 *
 * \param[in]   float* highlim      ARM6DOF运动关节运动范围上限[rad1, rad2, rad3, rad4, rad5 rad6]
 *\															默认值:  {PI, PI , PI, PI, PI, PI }
 * \param[in]    float* lowlim      ARM6DOF运动关节运动范围下限[rad1, rad2, rad3, rad4, rad5 rad6]
 *\															默认值: {-PI, -PI ,-PI, -PI, -PI, -PI },
 * \return      程序正常，返回0
 */
int16_t robc_ARM6DOF_set_movelim(float highlim[6], float lowlim[6])
{
	int i=0;
	for (i=0; i<6; i++)
	{
		ARM6DOF_para.JointValLimH[i] = highlim[i];
		ARM6DOF_para.JointValLimL[i] = lowlim[i];
	}
	return 0;
}

/**
* \brief       设置运动学反解模式
*
* \param[in]   int8_t modle      modle=0: 自动选择; modle=1: 左解; modle=0: 右解
* \														 默认值:  0
* \return      程序正常，返回0
*/
int16_t robc_ARM6DOF_set_modle(int8_t modle)
{
	ARM6DOF_para.PostureModle = modle;
	return 0;
}

/**
* \brief       设置各轴控制能量权值
*
* \param[in]   float*      轴1-轴4各轴控制能量权值，范围0-1，权值越大在自动选择反解模式时该轴运动量越小
* \										  默认值: {1, 1 ,1 , 0.3, 0.3, 0.3}
* \return      程序正常，返回0
*/
int16_t robc_ARM6DOF_set_jointweight(float jointweight[6])
{
	int i = 0;
	for (i = 0; i<6; i++)
	{
		ARM6DOF_para.JointWeight[i] = jointweight[i];
		if (ARM6DOF_para.JointWeight[i] < 0)
			ARM6DOF_para.JointWeight[i] = 0;
		else if (ARM6DOF_para.JointWeight[i] > 1)
			ARM6DOF_para.JointWeight[i] = 1;
	}
	return 0;
}

/**
* \brief       设置ARM6DOF各轴关节位置参数
*
* \param[in]   float*      ARM6DOF轴1-轴4关节位置参数
* \										  默认值:  { 0, 0, 0, 0, 0 , 0 }
* \return      程序正常，返回0
*/
int16_t robc_ARM6DOF_set_jointval(float jointval[6])
{
	int i = 0;
	for (i = 0; i<6; i++)
		ARM6DOF_para.JointVal[i] = jointval[i];
	return 0;
}

/**
 * \brief       更新ARM6DOF各轴关节位置至指定数组
 *
 * \param[in]   float*      存放反解出关节位置的数组
 * \										   默认值: 无
 * \return      程序正常，返回0
 */
int16_t robc_ARM6DOF_updata_jointval(float jointval[6])
{
	int i=0;
	for (i=0; i<6; i++)
		jointval[i] = ARM6DOF_para.JointVal[i];
	return 0;
}

/**
* \brief       更新ARM6DOF笛卡尔空间位置至指定数组
*
* \param[in]   float*      存放正解出笛卡尔空间位置的数组
* \										   默认值: 无
* \return      程序正常，返回0
*/
int16_t robc_ARM6DOF_updata_posval(float posval[6])
{
	int i = 0;
	for (i = 0; i<6; i++)
		posval[i] = ARM6DOF_para.PosVal[i];
	return 0;
}

/**
 * \brief       ARM6DOF反解计算程序
 *
 * \param[in]   float*      ARM6DOF笛卡尔空间目标位置数组
 * \										   默认值: { 0, 0, 0, 0 }
 * \return      反解成功返回0，不成功返回-1
 */
int16_t robc_ARM6DOF_Anno_ikcalc(float Dest[6])
{
	double matrix_R[MATRIX_N][MATRIX_N];
	double j1[2] = { 0 };  //元素值 >=360 度或 < -360 度 表示角度无效
	double j2[4] = { 0 };
	double j3[4] = { 0 };
	double j4[8] = { 0 };
	double j5[8] = { 0 };
	double j6[8] = { 0 };

	double worldx, worldy, worldz;
	double worldrr, worldrp, worldry;
	double px, py, pz;

	double a1 = ARM6DOF_para.DHparam[0][0];

	double a2 = ARM6DOF_para.DHparam[1][0];
	double a3 = ARM6DOF_para.DHparam[2][0];
	double d1 = ARM6DOF_para.DHparam[0][2];
	double d4 = ARM6DOF_para.DHparam[3][2];
	double d6 = ARM6DOF_para.DHparam[5][2];

	double paramj23[4] = { 0 };//arm1, arm2, px23, py23;

	double score = 0;

	int i, j;
	for (i = 0; i<6; i++)
		ARM6DOF_para.DestPos[i] = Dest[i];

	worldx = ARM6DOF_para.DestPos[0];
	worldy = ARM6DOF_para.DestPos[1];
	worldz = ARM6DOF_para.DestPos[2];
	worldrr = ARM6DOF_para.DestPos[3];










	worldrp = ARM6DOF_para.DestPos[4];
	worldry = ARM6DOF_para.DestPos[5];

	calculate_matrix_R(worldrr, worldrp, worldry, matrix_R);

	px = worldx - matrix_R[0][2] * d6;
	py = worldy - matrix_R[1][2] * d6;
	pz = worldz - matrix_R[2][2] * d6;

	matrix_R[0][3] = px;
	matrix_R[1][3] = py;
	matrix_R[2][3] = pz;
	matrix_R[3][0] = 0;
	matrix_R[3][1] = 0;
	matrix_R[3][2] = 0;
	matrix_R[3][3] = 1;

	//计算 j1
	j1[0] = atan2(py, px);
	IS_ZERO(j1[0]);
	j1[1] = angleRange(j1[0] + PI);
	JUDGE_ZERO(j1[1] - 2 * PI) ? (j1[1] = 0) : 1;
	
	//计算 j2, j3
	paramj23[0] =  a2;//arm1
	paramj23[1] = sqrt(a3*a3 + d4*d4);//arm2
	paramj23[2] = sqrt(px*px + py*py) - a1;//px23
	paramj23[3] = pz - d1;//py23
	if (ARM6DOFik_j23calc(paramj23, j2, j3) == 0)
	{
		if (a1 != 0)
		{
			paramj23[2] = -sqrt(px*px + py*py) - a1;//px23
			ARM6DOFik_j23calc(paramj23, j2 + 2, j3 + 2);
		}
		else
		{
			j2[2] = -j2[0];
			j3[2] = j3[1];
			j2[3] = -j2[1];
			j3[3] = j3[0];
		}
	}
	else
	{
		for (i = 0; i<8; i++)
			ARM6DOF_para.IsGoodSolution[i] = 0;
		return -1;
	}

	for (i = 0; i < 4; i++)
		j3[i] = angleRange(j3[i] + PI_2 - JOINT_OFFSET_J3 - atan2(a3, d4));

	//计算 j4, j5, j6
	for (i = 0, j = 0; i<4; i++)
	{
		ARM6DOFik_j456calc(j1[i / 2], j2[i], j3[i], matrix_R, &j4[j], &j5[j], &j6[j]);
		j += 2;
	}

	
	for (i = 0; i < 8; i++)
	{
		ARM6DOF_para.JointValDest[0] = j1[i / 4] - ARM6DOF_para.angleOffset[0];
		ARM6DOF_para.JointValDest[1] = j2[i / 2] - ARM6DOF_para.angleOffset[1];
		ARM6DOF_para.JointValDest[2] = j3[i / 2] - ARM6DOF_para.angleOffset[2];
		ARM6DOF_para.JointValDest[3] = j4[i] - ARM6DOF_para.angleOffset[3];
		ARM6DOF_para.JointValDest[4] = j5[i] - ARM6DOF_para.angleOffset[4];
		ARM6DOF_para.JointValDest[5] = j6[i] - ARM6DOF_para.angleOffset[5];
 		if (JointValPossible())
			ARM6DOF_para.IsGoodSolution[i] = 1;
		else
			ARM6DOF_para.IsGoodSolution[i] = 0;
	}

	if (ARM6DOF_para.PostureModle < 1 || ARM6DOF_para.PostureModle > 8)
	{
		j = -1;
		score = -1;
		for (i = 0; i < 8; i++)
		{
			if (ARM6DOF_para.IsGoodSolution[i])
			{
				ARM6DOF_para.JointValDest[0] = j1[i / 4] - ARM6DOF_para.angleOffset[0];
				ARM6DOF_para.JointValDest[1] = j2[i / 2] - ARM6DOF_para.angleOffset[1];
				ARM6DOF_para.JointValDest[2] = j3[i / 2] - ARM6DOF_para.angleOffset[2];
				ARM6DOF_para.JointValDest[3] = j4[i] - ARM6DOF_para.angleOffset[3];
				ARM6DOF_para.JointValDest[4] = j5[i] - ARM6DOF_para.angleOffset[4];
				ARM6DOF_para.JointValDest[5] = j6[i] - ARM6DOF_para.angleOffset[5];
				if (score < 0)
				{
					score = ARM6DOFik_CalcuScore();
					j = i;
				}
				else
				{
					if (score > ARM6DOFik_CalcuScore())
					{
						score = ARM6DOFik_CalcuScore();
						j = i;
					}
				}
			}
		}

		if (j < 0)
			return -1;
		else
		{
			ARM6DOF_para.JointValDest[0] = j1[j / 4] - ARM6DOF_para.angleOffset[0];
			ARM6DOF_para.JointValDest[1] = j2[j / 2] - ARM6DOF_para.angleOffset[1];
			ARM6DOF_para.JointValDest[2] = j3[j / 2] - ARM6DOF_para.angleOffset[2];
			ARM6DOF_para.JointValDest[3] = j4[j] - ARM6DOF_para.angleOffset[3];
			ARM6DOF_para.JointValDest[4] = j5[j] - ARM6DOF_para.angleOffset[4];
			ARM6DOF_para.JointValDest[5] = j6[j] - ARM6DOF_para.angleOffset[5];
			for (i = 0; i < 6; i++)
			{
				ARM6DOF_para.JointVal[i] = ARM6DOF_para.JointValDest[i];
				//printf("%f\n", ARM6DOF_para.JointValDest[i]);
			}
			return 0;
		}
	}
	else
	{
		j = ARM6DOF_para.PostureModle-1;
		if (ARM6DOF_para.IsGoodSolution[j])
		{
			ARM6DOF_para.JointValDest[0] = j1[j / 4] - ARM6DOF_para.angleOffset[0];
			ARM6DOF_para.JointValDest[1] = j2[j / 2] - ARM6DOF_para.angleOffset[1];
			ARM6DOF_para.JointValDest[2] = j3[j / 2] - ARM6DOF_para.angleOffset[2];
			ARM6DOF_para.JointValDest[3] = j4[j] - ARM6DOF_para.angleOffset[3];
			ARM6DOF_para.JointValDest[4] = j5[j] - ARM6DOF_para.angleOffset[4];
			ARM6DOF_para.JointValDest[5] = j6[j] - ARM6DOF_para.angleOffset[5];
			for (i = 0; i < 6; i++)
			{
				ARM6DOF_para.JointVal[i] = ARM6DOF_para.JointValDest[i];
				//printf("%f\n", ARM6DOF_para.JointValDest[i]);
			}
			return 0;
		}
		else
			return -1;
	}
}

/**
* \brief       ARM6DOF正解计算程序
*
* \param[in]   float*      ARM6DOF关节空间目标位置数组
* \										   默认值: { 0, 0, 0, 0 }
* \return      程序正常，返回0
*/
int16_t robc_ARM6DOF_Anno_kicalc(float JointVal[6])
{
	double matrix_T1[MATRIX_N][MATRIX_N];
	double matrix_T2[MATRIX_N][MATRIX_N];
	double matrix_T3[MATRIX_N][MATRIX_N];
	double matrix_T4[MATRIX_N][MATRIX_N];
	double matrix_T5[MATRIX_N][MATRIX_N];
	double matrix_T6[MATRIX_N][MATRIX_N];

	double matrix_A1[MATRIX_N][MATRIX_N];
	double matrix_A2[MATRIX_N][MATRIX_N];
	double matrix_A3[MATRIX_N][MATRIX_N];
	double matrix_A4[MATRIX_N][MATRIX_N];
	double matrix_A5[MATRIX_N][MATRIX_N];
	double matrix_A6[MATRIX_N][MATRIX_N];

	int i;

	for (i = 0; i<6; i++)
		ARM6DOF_para.DHparam[i][3] = JointVal[i] + ARM6DOF_para.angleOffset[i];

	ARM6DOF_para.DHparam[1][3] += JOINT_OFFSET_J2;
	ARM6DOF_para.DHparam[2][3] += JOINT_OFFSET_J3;

	for (i = 0; i<6; i++)
	{
		ARM6DOF_para.JointValDest[i] = angleRange(ARM6DOF_para.DHparam[i][3] + ARM6DOF_para.angleOffset[i]);
	}
	/*if (!JointValPossible())
		return -1;*/

	//计算关节坐标矩阵 matrix A1--A6
	calculate_matrix_A(matrix_A1, *(ARM6DOF_para.DHparam + 0));
	calculate_matrix_A(matrix_A2, *(ARM6DOF_para.DHparam + 1));
	calculate_matrix_A(matrix_A3, *(ARM6DOF_para.DHparam + 2));
	calculate_matrix_A(matrix_A4, *(ARM6DOF_para.DHparam + 3));
	calculate_matrix_A(matrix_A5, *(ARM6DOF_para.DHparam + 4));
	calculate_matrix_A(matrix_A6, *(ARM6DOF_para.DHparam + 5));

	//计算变换矩阵 matrix T1---T6
	matrix_copy(*matrix_A1, *matrix_T1, MATRIX_N, MATRIX_N);
	matrix_mul(*matrix_T1, *matrix_A2, *matrix_T2, MATRIX_N, MATRIX_N);
	matrix_mul(*matrix_T2, *matrix_A3, *matrix_T3, MATRIX_N, MATRIX_N);
	matrix_mul(*matrix_T3, *matrix_A4, *matrix_T4, MATRIX_N, MATRIX_N);
	matrix_mul(*matrix_T4, *matrix_A5, *matrix_T5, MATRIX_N, MATRIX_N);
	matrix_mul(*matrix_T5, *matrix_A6, *matrix_T6, MATRIX_N, MATRIX_N);

	ARM6DOF_para.PosVal[0] = matrix_T6[0][3];
	ARM6DOF_para.PosVal[1] = matrix_T6[1][3];
	ARM6DOF_para.PosVal[2] = matrix_T6[2][3];
	ARM6DOF_para.PosVal[3] = atan2(matrix_T6[1][0], matrix_T6[0][0]);//z
	ARM6DOF_para.PosVal[4] = atan2(-matrix_T6[2][0], sqrt(pow(matrix_T6[2][1],2)+ pow(matrix_T6[2][2],2)));//y
	ARM6DOF_para.PosVal[5] = atan2(matrix_T6[2][1], matrix_T6[2][2]);//x
	/*printf("%f\n", matrix_T1[0][3]);
	printf("%f\n", matrix_T1[1][3]);
	printf("%f\n", matrix_T1[2][3]);

	printf("%f\n", matrix_T2[0][3]);
	printf("%f\n", matrix_T2[1][3]);
	printf("%f\n", matrix_T2[2][3]);

	printf("%f\n", matrix_T3[0][3]);
	printf("%f\n", matrix_T3[1][3]);
	printf("%f\n", matrix_T3[2][3]);*/

	/*printf("%f\n", matrix_T4[0][3]);
	printf("%f\n", matrix_T4[1][3]);
	printf("%f\n", matrix_T4[2][3]);

	printf("%f\n", matrix_T5[0][3]);
	printf("%f\n", matrix_T5[1][3]);
	printf("%f\n", matrix_T5[2][3]);*/

	/*printf("%f\n", ARM6DOF_para.PosVal[0]);	
	printf("%f\n", ARM6DOF_para.PosVal[1]);
	printf("%f\n", ARM6DOF_para.PosVal[2]);
	printf("%f\n", ARM6DOF_para.PosVal[3]);
	printf("%f\n", ARM6DOF_para.PosVal[4]);
	printf("%f\n", ARM6DOF_para.PosVal[5]);*/



	/*
	printf("\n----curent x, y, z-----\n%lf \n %lf\n %lf\n ",
		matrix_T6[0][3], matrix_T6[1][3],
		matrix_T6[2][3]);
	printf("matrix_T2 =  \n");
	printmatrix(matrix_T6, MATRIX_N, MATRIX_N);
	printf("\n----curent r, p, y-----\n%lf \n %lf\n %lf\n ",
		ARM6DOF_para.PosVal[3], ARM6DOF_para.PosVal[4],
		ARM6DOF_para.PosVal[5]);
	printf("\nOVER\n ");
	*/
	return 0;
}
int16_t robc_ARM6DOF_C800_ikcalc(float Dest[6])
{
	double matrix_R[MATRIX_N][MATRIX_N];
	double j1[2] = { 0 };
	double j2[8] = { 0 };
	double j3[8] = { 0 };
	double j4[8] = { 0 };
	double j5[4] = { 0 };
	double j6[4] = { 0 };
	double joint[8][6] = { 0 };
	int solution[8] = { 0 };
	double score = 0;

	double d1 = ARM6DOF_para.DHparam[0][2];
	double d4 = ARM6DOF_para.DHparam[3][2];
	double d5 = ARM6DOF_para.DHparam[4][2];
	double d6 = ARM6DOF_para.DHparam[5][2];
	double a2 = ARM6DOF_para.DHparam[1][0];
	double a3 = ARM6DOF_para.DHparam[2][0];
	double m = 0, n = 0;
	double mm = 0, nn = 0;
	double mmm = 0, nnn = 0;
	double s2 = 0, c2 = 0;

	calculate_matrix_R(Dest[3], Dest[4], Dest[5], matrix_R);

	m = d6*matrix_R[1][2] - Dest[1];
	n = d6*matrix_R[0][2] - Dest[0];
	////J1////
	j1[0] = atan2(m, n) - atan2(d4, -sqrt(m*m + n*n - d4*d4));
	j1[1] = atan2(m, n) - atan2(d4, sqrt(m*m + n*n - d4*d4));
	/////J5/////
	j5[0] = acos(matrix_R[0][2] * sin(j1[0]) - matrix_R[1][2] * cos(j1[0]));
	j5[1] =-acos(matrix_R[0][2] * sin(j1[0]) - matrix_R[1][2] * cos(j1[0]));
	j5[2] = acos(matrix_R[0][2] * sin(j1[1]) - matrix_R[1][2] * cos(j1[1]));
	j5[3] =-acos(matrix_R[0][2] * sin(j1[1]) - matrix_R[1][2] * cos(j1[1]));
	//////J6//////
	mm = matrix_R[0][0] * sin(j1[0]) - matrix_R[1][0] * cos(j1[0]);
	nn = matrix_R[0][1] * sin(j1[0]) - matrix_R[1][1] * cos(j1[0]);
	j6[0] = atan2(mm, nn) - atan2(sin(j5[0]), 0);
	j6[1] = atan2(mm, nn) - atan2(sin(j5[1]), 0);

	mm = matrix_R[0][0] * sin(j1[1]) - matrix_R[1][0] * cos(j1[1]);
	nn = matrix_R[0][1] * sin(j1[1]) - matrix_R[1][1] * cos(j1[1]);
	j6[2] = atan2(mm, nn) - atan2(sin(j5[2]), 0);
	j6[3] = atan2(mm, nn) - atan2(sin(j5[3]), 0);
	////J3,J2////////
	mmm = d5*(sin(j6[0])*(matrix_R[0][0] * cos(j1[0]) + matrix_R[1][0] * sin(j1[0])) + cos(j6[0])*(matrix_R[0][1] * cos(j1[0]) + matrix_R[1][1] * sin(j1[0])))
		- d6*(matrix_R[0][2] * cos(j1[0]) + matrix_R[1][2] * sin(j1[0]))+Dest[0] * cos(j1[0]) + Dest[1] * sin(j1[0]);
	nnn = Dest[2] - d1 - matrix_R[2][2] *d6 + d5*(matrix_R[2][1]*cos(j6[0]) + matrix_R[2][0]*sin(j6[0]));
	j3[0] = acos((mmm*mmm + nnn*nnn - a2*a2 - a3*a3) / (2 * a2*a3));
	j3[1] = -acos((mmm*mmm + nnn*nnn - a2*a2 - a3*a3) / (2 * a2*a3));

	s2 = ((a3*cos(j3[0]) + a2)*nnn - a3*sin(j3[0])*mmm) / (a2*a2 + a3*a3 + 2 * a2*a3*cos(j3[0]));
	c2 = (mmm + a3*sin(j3[0])*s2) / (a3*cos(j3[0]) + a2);
	j2[0] = atan2(s2, c2);
	s2 = ((a3*cos(j3[1]) + a2)*nnn - a3*sin(j3[1])*mmm) / (a2*a2 + a3*a3 + 2 * a2*a3*cos(j3[1]));
	c2 = (mmm + a3*sin(j3[1])*s2) / (a3*cos(j3[1]) + a2);
	j2[1] = atan2(s2, c2);

	mmm = d5*(sin(j6[1])*(matrix_R[0][0] * cos(j1[0]) + matrix_R[1][0] * sin(j1[0])) + cos(j6[1])*(matrix_R[0][1] * cos(j1[0]) + matrix_R[1][1] * sin(j1[0])))
		- d6*(matrix_R[0][2] * cos(j1[0]) + matrix_R[1][2] * sin(j1[0])) + Dest[0] * cos(j1[0]) + Dest[1] * sin(j1[0]);
	nnn = Dest[2] - d1 - matrix_R[2][2] * d6 + d5*(matrix_R[2][1] * cos(j6[1]) + matrix_R[2][0] * sin(j6[1]));
	j3[2] = acos((mmm*mmm + nnn*nnn - a2*a2 - a3*a3) / (2 * a2*a3));
	j3[3] = -acos((mmm*mmm + nnn*nnn - a2*a2 - a3*a3) / (2 * a2*a3));

	s2 = ((a3*cos(j3[2])+a2)*nnn-a3*sin(j3[2])*mmm) / (a2*a2+a3*a3+2*a2*a3*cos(j3[2]));
	c2 = (mmm+a3*sin(j3[2])*s2) / (a3*cos(j3[2])+a2);
	j2[2] = atan2(s2, c2);
	s2 = ((a3*cos(j3[3]) + a2)*nnn - a3*sin(j3[3])*mmm) / (a2*a2 + a3*a3 + 2 * a2*a3*cos(j3[3]));
	c2 = (mmm + a3*sin(j3[3])*s2) / (a3*cos(j3[3]) + a2);
	j2[3] = atan2(s2, c2);

	mmm = d5*(sin(j6[2])*(matrix_R[0][0] * cos(j1[1]) + matrix_R[1][0] * sin(j1[1])) + cos(j6[2])*(matrix_R[0][1] * cos(j1[1]) + matrix_R[1][1] * sin(j1[1])))
		- d6*(matrix_R[0][2] * cos(j1[1]) + matrix_R[1][2] * sin(j1[1])) + Dest[0] * cos(j1[1]) + Dest[1] * sin(j1[1]);
	nnn = Dest[2] - d1 - matrix_R[2][2] * d6 + d5*(matrix_R[2][1] * cos(j6[2]) + matrix_R[2][0] * sin(j6[2]));
	j3[4] = acos((mmm*mmm + nnn*nnn - a2*a2 - a3*a3) / (2 * a2*a3));
	j3[5] = -acos((mmm*mmm + nnn*nnn - a2*a2 - a3*a3) / (2 * a2*a3));

	s2 = ((a3*cos(j3[4]) + a2)*nnn - a3*sin(j3[4])*mmm) / (a2*a2 + a3*a3 + 2 * a2*a3*cos(j3[4]));
	c2 = (mmm + a3*sin(j3[4])*s2) / (a3*cos(j3[4]) + a2);
	j2[4] = atan2(s2, c2);
	s2 = ((a3*cos(j3[5]) + a2)*nnn - a3*sin(j3[5])*mmm) / (a2*a2 + a3*a3 + 2 * a2*a3*cos(j3[5]));
	c2 = (mmm + a3*sin(j3[5])*s2) / (a3*cos(j3[5]) + a2);
	j2[5] = atan2(s2, c2);

	mmm = d5*(sin(j6[3])*(matrix_R[0][0] * cos(j1[1]) + matrix_R[1][0] * sin(j1[1])) + cos(j6[3])*(matrix_R[0][1] * cos(j1[1]) + matrix_R[1][1] * sin(j1[1])))
		- d6*(matrix_R[0][2] * cos(j1[1]) + matrix_R[1][2] * sin(j1[1])) + Dest[0] * cos(j1[1]) + Dest[1] * sin(j1[1]);
	nnn = Dest[2] - d1 - matrix_R[2][2] * d6 + d5*(matrix_R[2][1] * cos(j6[3]) + matrix_R[2][0] * sin(j6[3]));
	j3[6] = acos((mmm*mmm + nnn*nnn - a2*a2 - a3*a3) / (2 * a2*a3));
	j3[7] = -acos((mmm*mmm + nnn*nnn - a2*a2 - a3*a3) / (2 * a2*a3));

	s2 = ((a3*cos(j3[6]) + a2)*nnn - a3*sin(j3[6])*mmm) / (a2*a2 + a3*a3 + 2 * a2*a3*cos(j3[6]));
	c2 = (mmm + a3*sin(j3[6])*s2) / (a3*cos(j3[6]) + a2);
	j2[6] = atan2(s2, c2);
	s2 = ((a3*cos(j3[7]) + a2)*nnn - a3*sin(j3[7])*mmm) / (a2*a2 + a3*a3 + 2 * a2*a3*cos(j3[7]));
	c2 = (mmm + a3*sin(j3[7])*s2) / (a3*cos(j3[7]) + a2);
	j2[7] = atan2(s2, c2);
	////J6///////
	j4[0] = atan2(-sin(j6[0])*(matrix_R[0][0] * cos(j1[0]) + matrix_R[1][0] * sin(j1[0])) - cos(j6[0])*(matrix_R[0][1] * cos(j1[0]) + matrix_R[1][1] * sin(j1[0])), matrix_R[2][1] * cos(j6[0]) + matrix_R[2][0] * sin(j6[0])) - j2[0] - j3[0];
	j4[1] = atan2(-sin(j6[0])*(matrix_R[0][0] * cos(j1[0]) + matrix_R[1][0] * sin(j1[0])) - cos(j6[0])*(matrix_R[0][1] * cos(j1[0]) + matrix_R[1][1] * sin(j1[0])), matrix_R[2][1] * cos(j6[0]) + matrix_R[2][0] * sin(j6[0])) - j2[1] - j3[1];
	j4[2] = atan2(-sin(j6[1])*(matrix_R[0][0] * cos(j1[0]) + matrix_R[1][0] * sin(j1[0])) - cos(j6[1])*(matrix_R[0][1] * cos(j1[0]) + matrix_R[1][1] * sin(j1[0])), matrix_R[2][1] * cos(j6[1]) + matrix_R[2][0] * sin(j6[1])) - j2[2] - j3[2];
	j4[3] = atan2(-sin(j6[1])*(matrix_R[0][0] * cos(j1[0]) + matrix_R[1][0] * sin(j1[0])) - cos(j6[1])*(matrix_R[0][1] * cos(j1[0]) + matrix_R[1][1] * sin(j1[0])), matrix_R[2][1] * cos(j6[1]) + matrix_R[2][0] * sin(j6[1])) - j2[3] - j3[3];
	j4[4] = atan2(-sin(j6[2])*(matrix_R[0][0] * cos(j1[1]) + matrix_R[1][0] * sin(j1[1])) - cos(j6[2])*(matrix_R[0][1] * cos(j1[1]) + matrix_R[1][1] * sin(j1[1])), matrix_R[2][1] * cos(j6[2]) + matrix_R[2][0] * sin(j6[2])) - j2[4] - j3[4];
	j4[5] = atan2(-sin(j6[2])*(matrix_R[0][0] * cos(j1[1]) + matrix_R[1][0] * sin(j1[1])) - cos(j6[2])*(matrix_R[0][1] * cos(j1[1]) + matrix_R[1][1] * sin(j1[1])), matrix_R[2][1] * cos(j6[2]) + matrix_R[2][0] * sin(j6[2])) - j2[5] - j3[5];
	j4[6] = atan2(-sin(j6[3])*(matrix_R[0][0] * cos(j1[1]) + matrix_R[1][0] * sin(j1[1])) - cos(j6[3])*(matrix_R[0][1] * cos(j1[1]) + matrix_R[1][1] * sin(j1[1])), matrix_R[2][1] * cos(j6[3]) + matrix_R[2][0] * sin(j6[3])) - j2[6] - j3[6];
	j4[7] = atan2(-sin(j6[3])*(matrix_R[0][0] * cos(j1[1]) + matrix_R[1][0] * sin(j1[1])) - cos(j6[3])*(matrix_R[0][1] * cos(j1[1]) + matrix_R[1][1] * sin(j1[1])), matrix_R[2][1] * cos(j6[3]) + matrix_R[2][0] * sin(j6[3])) - j2[7] - j3[7];

	/**********************把角度转换到-PI和PI之间************************/
	for (unsigned int i = 0; i < 8; i++)
	{
		j2[i] += PI / 2;
		j4[i] += PI / 2;
		j4[i] = -j4[i];
	}
	for (unsigned int i = 0; i < 2; i++)
	{
		j1[i] = -j1[i];
	}
	for (unsigned int i = 0; i < 4; i++)
	{
		j5[i] = -j5[i];
	}

	for (unsigned int i = 0; i < 2; i++)
	{
		j1[i] = revert_degree_betweenPI(j1[i]);
	}
	for (unsigned int i = 0; i < 4; i++)
	{
		j5[i] = revert_degree_betweenPI(j5[i]);
		j6[i] = revert_degree_betweenPI(j6[i]);
	}
	for (unsigned int i = 0; i < 8; i++)
	{
		j2[i] = revert_degree_betweenPI(j2[i]);
		j3[i] = revert_degree_betweenPI(j3[i]);
		j4[i] = revert_degree_betweenPI(j4[i]);
	}

	joint[0][0] = j1[0];
	joint[1][0] = j1[0];
	joint[2][0] = j1[0];
	joint[3][0] = j1[0];
	joint[4][0] = j1[1];
	joint[5][0] = j1[1];
	joint[6][0] = j1[1];
	joint[7][0] = j1[1];

	joint[0][1] = j2[0];
	joint[1][1] = j2[1];
	joint[2][1] = j2[2];
	joint[3][1] = j2[3];
	joint[4][1] = j2[4];
	joint[5][1] = j2[5];
	joint[6][1] = j2[6];
	joint[7][1] = j2[7];

	joint[0][2] = j3[0];
	joint[1][2] = j3[1];
	joint[2][2] = j3[2];
	joint[3][2] = j3[3];
	joint[4][2] = j3[4];
	joint[5][2] = j3[5];
	joint[6][2] = j3[6];
	joint[7][2] = j3[7];

	joint[0][3] = j4[0];
	joint[1][3] = j4[1];
	joint[2][3] = j4[2];
	joint[3][3] = j4[3];
	joint[4][3] = j4[4];
	joint[5][3] = j4[5];
	joint[6][3] = j4[6];
	joint[7][3] = j4[7];

	joint[0][4] = j5[0];
	joint[1][4] = j5[0];
	joint[2][4] = j5[1];
	joint[3][4] = j5[1];
	joint[4][4] = j5[2];
	joint[5][4] = j5[2];
	joint[6][4] = j5[3];
	joint[7][4] = j5[3];

	joint[0][5] = j6[0];
	joint[1][5] = j6[0];
	joint[2][5] = j6[1];
	joint[3][5] = j6[1];
	joint[4][5] = j6[2];
	joint[5][5] = j6[2];
	joint[6][5] = j6[3];
	joint[7][5] = j6[3];
	/********************判断无效数字*******************/
	for (unsigned int i = 0; i < 8; i++)
	{
		for (unsigned int j = 0; j < 6; j++)
			if (isnan(joint[i][j]) == 1)
				solution[i] = 1;
	}
	/*******判断是否超出joint范围*************************/
	for (unsigned int i = 0; i < 8; i++)
	{
		ARM6DOF_para.JointValDest[0] = joint[i][0];
		ARM6DOF_para.JointValDest[1] = joint[i][1];
		ARM6DOF_para.JointValDest[2] = joint[i][2];
		ARM6DOF_para.JointValDest[3] = joint[i][3];
		ARM6DOF_para.JointValDest[4] = joint[i][4];
		ARM6DOF_para.JointValDest[5] = joint[i][5];
	 	if (!JointValPossible())
	 		solution[i] = 1;
	}

	score = -1;
	int j = -1;

	for (unsigned int i = 0; i < 8; i++)
	{
		if (solution[i] == 0)
		{
			ARM6DOF_para.JointValDest[0] = joint[i][0];
			ARM6DOF_para.JointValDest[1] = joint[i][1];
			ARM6DOF_para.JointValDest[2] = joint[i][2];
			ARM6DOF_para.JointValDest[3] = joint[i][3];
			ARM6DOF_para.JointValDest[4] = joint[i][4];
			ARM6DOF_para.JointValDest[5] = joint[i][5];

			if (score < 0)
			{
				score = ARM6DOFik_CalcuScore();
				j = i;
			}
			else
			{
				if (score > ARM6DOFik_CalcuScore())
				{
					score = ARM6DOFik_CalcuScore();
					j = i;
				}
			}
		}
	}

	ARM6DOF_para.JointValDest[0] = joint[j][0];
	ARM6DOF_para.JointValDest[1] = joint[j][1];
	ARM6DOF_para.JointValDest[2] = joint[j][2];
	ARM6DOF_para.JointValDest[3] = joint[j][3];
	ARM6DOF_para.JointValDest[4] = joint[j][4];
	ARM6DOF_para.JointValDest[5] = joint[j][5];

	for (unsigned int i = 0; i < 6; i++)
		ARM6DOF_para.JointVal[i] = ARM6DOF_para.JointValDest[i];

    if( j == -1)
        return -1;
    else
		return 0;
}
int16_t robc_ARM6DOF_C800_kicalc(float JointVal[6])
{
	double matrix_T1[MATRIX_N][MATRIX_N];
    double matrix_T2[MATRIX_N][MATRIX_N];
	double matrix_T3[MATRIX_N][MATRIX_N];
	double matrix_T4[MATRIX_N][MATRIX_N];
	double matrix_T5[MATRIX_N][MATRIX_N];
	double matrix_T6[MATRIX_N][MATRIX_N];

	double matrix_A1[MATRIX_N][MATRIX_N];
	double matrix_A2[MATRIX_N][MATRIX_N];
	double matrix_A3[MATRIX_N][MATRIX_N];
	double matrix_A4[MATRIX_N][MATRIX_N];
	double matrix_A5[MATRIX_N][MATRIX_N];
	double matrix_A6[MATRIX_N][MATRIX_N];

	int i;

	for (i = 0; i < 6; i++)
		ARM6DOF_para.DHparam[i][3] = JointVal[i];

	ARM6DOF_para.DHparam[0][3] = -ARM6DOF_para.DHparam[0][3];
	ARM6DOF_para.DHparam[3][3] = -ARM6DOF_para.DHparam[3][3];
	ARM6DOF_para.DHparam[4][3] = -ARM6DOF_para.DHparam[4][3];

	for (i = 0; i < 6; i++)
	{
		ARM6DOF_para.DHparam[i][3] += ARM6DOF_para.angleOffset[i];
	}

	calculate_matrix_A(matrix_A1, *(ARM6DOF_para.DHparam + 0));
	calculate_matrix_A(matrix_A2, *(ARM6DOF_para.DHparam + 1));
	calculate_matrix_A(matrix_A3, *(ARM6DOF_para.DHparam + 2));
	calculate_matrix_A(matrix_A4, *(ARM6DOF_para.DHparam + 3));
	calculate_matrix_A(matrix_A5, *(ARM6DOF_para.DHparam + 4));
	calculate_matrix_A(matrix_A6, *(ARM6DOF_para.DHparam + 5));

	matrix_copy(*matrix_A1, *matrix_T1, MATRIX_N, MATRIX_N);
	matrix_mul(*matrix_T1, *matrix_A2, *matrix_T2, MATRIX_N, MATRIX_N);
	matrix_mul(*matrix_T2, *matrix_A3, *matrix_T3, MATRIX_N, MATRIX_N);
	matrix_mul(*matrix_T3, *matrix_A4, *matrix_T4, MATRIX_N, MATRIX_N);
	matrix_mul(*matrix_T4, *matrix_A5, *matrix_T5, MATRIX_N, MATRIX_N);
	matrix_mul(*matrix_T5, *matrix_A6, *matrix_T6, MATRIX_N, MATRIX_N);

	ARM6DOF_para.PosVal[0] = matrix_T6[0][3];
	ARM6DOF_para.PosVal[1] = matrix_T6[1][3];
	ARM6DOF_para.PosVal[2] = matrix_T6[2][3];
	ARM6DOF_para.PosVal[3] = atan2(matrix_T6[1][0], matrix_T6[0][0]);
	ARM6DOF_para.PosVal[4] = atan2(-matrix_T6[2][0], sqrt(pow(matrix_T6[2][1],2)+ pow(matrix_T6[2][2],2)));
	ARM6DOF_para.PosVal[5] = atan2(matrix_T6[2][1], matrix_T6[2][2]);
}
double revert_degree_betweenPI(double joint_degree)
{
	if (joint_degree > PI)
	{
		while (1)
		{
			joint_degree -= 2 * PI;
			if (joint_degree <= PI)
				break;
		}
	}
	else if (joint_degree < -PI)
	{
		while (1)
		{
			joint_degree += 2 * PI;
			if (joint_degree >= -PI)
				break;
		}
	}
	return joint_degree;
}
/** @} */
