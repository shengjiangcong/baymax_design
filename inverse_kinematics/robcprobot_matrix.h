/** ---------------------------------------------------------------------------
 * Copyright (c) 2017, PS-Micro, Co. Ltd.  All rights reserved.
 *
 * @file    <robcprobot_ARM6DOFik.h>    by yaoym
 * @date    2019-05-17
 * @version 0.1.1  2019.05
---------------------------------------------------------------------------- */
/**
 * @file
 *   This file defines the ARM6DOF kinematics and inverse kinematics API functions.
 */

/**
 * @ingroup
 */

#ifndef _ROBCPROBOT_MATRIX_H_
#define _ROBCPROBOT_MATRIX_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "stdint.h"

 /**
 * \brief       trap parameter check
 *
 * \param[in]   *axis_param      structure  MOTNP_TRAP_AXIS_PARAM
 * \return      Return value, see return value for details
 */
void matrix_remap(double *matrix_a, double *matrix_b, int m, int n, int p, int q);

 /**
 * \brief       trap parameter check
 *
 * \param[in]   *axis_param      structure  MOTNP_TRAP_AXIS_PARAM
 * \return      Return value, see return value for details
 */
void matrix_copy(double *matrix_a, double *matrix_b, int m, int n);

/**
* \brief       trap parameter check
*
* \param[in]   *axis_param      structure  MOTNP_TRAP_AXIS_PARAM
* \return      Return value, see return value for details
*/
int matrix_mul(double *matrix_a, double *matrix_b, double *matrix_result, int m, int n);

/**
* \brief       trap parameter check
*
* \param[in]   *axis_param      structure  MOTNP_TRAP_AXIS_PARAM
* \return      Return value, see return value for details
*/
int matrix_add(double *matrix_a, double *matrix_b, double *matrix_sum, int m, int n);

/**
* \brief       trap parameter check
*
* \param[in]   *axis_param      structure  MOTNP_TRAP_AXIS_PARAM
* \return      Return value, see return value for details
*/
void matrix_translate(double *matrix, int m, int n);

/**
* \brief       trap parameter check
*
* \param[in]   *axis_param      structure  MOTNP_TRAP_AXIS_PARAM
* \return      Return value, see return value for details
*/
void printmatrix(double *matrix, int m, int n);

/** @} */

#ifdef __cplusplus
};
#endif

#endif /* _ROBCPROBOT_MATRIX_H_ */

