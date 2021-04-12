/** ---------------------------------------------------------------------------
 * Copyright (c) 2017, PS-Micro, Co. Ltd.  All rights reserved.
 *
 * @file    <robcprobot_ARM6DOFik.c>    by yaoym
 * @date    2019-03-17
 * @version 0.1.1  2019.03
---------------------------------------------------------------------------- */
/**
 * @file
 *   This file includes ARM6DOF kinematics and inverse kinematics functions.
 */

/**
 * @ingroup
 */

#include "robcprobot_matrix.h"
#include <stdio.h>
#include <stdlib.h>
 /*#include <assert.h>
#include <math.h>*/

 /* ----------------------------------------------------------------------------
 *   Static Parameter
 ---------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------------
   *   Static Function
---------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------------
   *   External Function
---------------------------------------------------------------------------- */
/**
* \brief       trap parameter check
*
* \param[in]   *axis_param      structure  MOTNP_TRAP_AXIS_PARAM
* \return      Return value, see return value for details
*/
void matrix_remap(double *matrix_a, double *matrix_b, int m, int n, int p, int q)
{
	int i, j;
	for (i = 0; i<p; i++)
	{
		for (j = 0; j<q; j++)
			matrix_b[i*q + j] = matrix_a[i*n + j];
	}
}

/**
* \brief       trap parameter check
*
* \param[in]   *axis_param      structure  MOTNP_TRAP_AXIS_PARAM
* \return      Return value, see return value for details
*/
void matrix_copy(double *matrix_a, double *matrix_b, int m, int n)
{
	int i, j;
	for (i = 0; i<m; i++) 
	{
		for (j = 0; j<n; j++) 
			matrix_b[i*n+j] = matrix_a[i*n + j];
	}
}

/**
* \brief       trap parameter check
*
* \param[in]   *axis_param      structure  MOTNP_TRAP_AXIS_PARAM
* \return      Return value, see return value for details
*/
int matrix_mul(double *matrix_a, double *matrix_b, double *matrix_result, int m, int n)
{
	int i, j, k;
	double sum;
	/*嵌套循环计算结果矩阵（m*n）的每个元素*/
	for (i = 0; i<m; i++)
		for (j = 0; j<n; j++)
		{
			/*按照矩阵乘法的规则计算结果矩阵的i*j元素*/
			sum = 0;
			for (k = 0; k<n; k++)
				sum += matrix_a[i*n+k] * matrix_b[k*n+j];
			matrix_result[i*n+j] = sum;
		}
	return 0;
}

/**
* \brief       trap parameter check
*
* \param[in]   *axis_param      structure  MOTNP_TRAP_AXIS_PARAM
* \return      Return value, see return value for details
*/
int matrix_add(double *matrix_a, 	double *matrix_b, 	double *matrix_sum, int m, int n)
{
	int i, j;

	for (i = 0; i<m; i++) {
		for (j = 0; j<n; j++) {
			matrix_sum[i*n+j] = matrix_a[i*n + j] + matrix_b[i*n + j];
		}
	}

	return 0;
}

/**
* \brief       trap parameter check
*
* \param[in]   *matrix demesion, translate demesion
* \return      Return value, see return value for details
*/
void matrix_translate(double *matrix, int m, int n)
{//矩阵转置
	double m_tmp;
	int i, j, k;
	for (i = 0, j = 0; i<m; i++, j++)
	{
		for (k = j; k<n; k++)
		{
			if (i == k) continue;
			m_tmp = matrix[i*n+k];
			matrix[i*n+k] = matrix[k*n+i];
			matrix[k*n+i] = m_tmp;
		}
	}
}

/**
* \brief       trap parameter check
*
* \param[in]   *axis_param      structure  MOTNP_TRAP_AXIS_PARAM
* \return      Return value, see return value for details
*/
void printmatrix(double *matrix, int m, int n)
{
	int i, j;
	for (i = 0; i<m; i++) {
		for (j = 0; j<n; j++) {
			printf(" %lf ", matrix[i*n+j]);
		}
		printf("\n");
	}
	printf("\n");
}
/** @} */
