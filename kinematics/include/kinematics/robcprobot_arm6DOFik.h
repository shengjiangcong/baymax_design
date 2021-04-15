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

#ifndef _ROBCPROBOT_ARM6DOFIK_H_
#define _ROBCPROBOT_ARM6DOFIK_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "stdint.h"

 /**
 * \brief       设置ARM6DOF几何参数
 *
 * \param[in]   float* Arm      指向存放ARM6DOF几何参数的数组[臂长1，臂长2，高度]
 *\													 默认值:  {100, 100, 100}
 * \return      程序正常，返回0
 */
int16_t robc_ARM6DOF_set_geometry(float alength[6], float alpha[6], float dlength[6], float theta0[6]);

/**
* \brief       设置ARM6DOF角度偏置值
*
* \param[in]   float* Arm      指向存放ARM6DOF几何参数的数组[offset参数]
* \return      程序正常，返回0
*/
int16_t robc_ARM6DOF_set_angleoffset(float angleoffset[6]);

/**
* \brief       设置ARM6DOF各个关节运动范围参数
*
* \param[in]   float* highlim      ARM6DOF运动关节运动范围上限[rad1, rad2, mm, rad3]
*\															默认值:  {PI, PI , 100, PI }
* \param[in]    float* lowlim      ARM6DOF运动关节运动范围下限[rad1, rad2, mm, rad3]
*\															默认值: {-PI, -PI ,0 , -PI },
* \return      程序正常，返回0
*/
int16_t robc_ARM6DOF_set_movelim(float highlim[6], float lowlim[6]);

/**
* \brief       设置运动学反解模式
*
* \param[in]   int8_t modle      modle=0: 自动选择; modle=1-8: 1-8解
* \														 默认值:  0
* \return      程序正常，返回0
*/
int16_t robc_ARM6DOF_set_modle(int8_t modle);

/**
* \brief       设置各轴控制能量权值
*
* \param[in]   float*      轴1-轴4各轴控制能量权值，范围0-1，权值越大在自动选择反解模式时该轴运动量越小
* \										  默认值: {1, .6 , .6 , .3, .3 ,.3}
* \return      程序正常，返回0
*/
int16_t robc_ARM6DOF_set_jointweight(float jointweight[6]);

/**
* \brief       设置ARM6DOF各轴关节位置参数
*
* \param[in]   float*      ARM6DOF轴1-轴4关节位置参数
* \										  默认值:  { 0, 0, 0, 0, 0, 0 }
* \return      程序正常，返回0
*/
int16_t robc_ARM6DOF_set_jointval(float jointval[6]);

/**
* \brief       更新ARM6DOF各轴关节位置至指定数组
*
* \param[in]   float*      存放反解出关节位置的数组
* \										   默认值: 无
* \return      程序正常，返回0
*/
int16_t robc_ARM6DOF_updata_jointval(float jointval[6]);

/**
* \brief       更新ARM6DOF笛卡尔空间位置至指定数组
*
* \param[in]   float*      存放正解出笛卡尔空间位置的数组
* \										   默认值: 无
* \return      程序正常，返回0
*/
int16_t robc_ARM6DOF_updata_posval(float posval[6]);

/**
* \brief       ARM6DOF反解计算程序
*
* \param[in]   float*      ARM6DOF笛卡尔空间目标位置数组
* \										   默认值: { 0, 0, 0, 0, 0, 0 }
* \return      反解成功返回0，不成功返回-1
*/
int16_t robc_ARM6DOF_Anno_ikcalc(float Dest[6]);

/**
* \brief       ARM6DOF正解计算程序
*
* \param[in]   float*      ARM6DOF关节空间目标位置数组
* \										   默认值: { 0, 0, 0, 0, 0, 0 }
* \return      程序正常，返回0
*/
int16_t robc_ARM6DOF_Anno_kicalc(float JointVal[6]);

/**
* \brief       C800反解计算程序
*
* \param[in]   float*      C800笛卡尔空间目标位置数组
* \										   默认值: { 0, 0, 0, 0, 0, 0 }
* \return      反解成功返回0，不成功返回-1
*/
int16_t robc_ARM6DOF_C800_ikcalc(float Dest[6]);

/**
* \brief       C800正解计算程序
*
* \param[in]   float*      C800关节空间目标位置数组
* \										   默认值: { 0, 0, 0, 0, 0, 0 }
* \return      程序正常，返回0
*/
int16_t robc_ARM6DOF_C800_kicalc(float JointVal[6]);

/**
* \brief       角度值归一到-PI~PI之间
*
* \param[in]   double      需要转换的角度（单位弧度）
* \
* \return      程序正常，返回转换后的角度
*/
double revert_degree_betweenPI(double joint_degree);
/** @} */

#ifdef __cplusplus
};
#endif

#endif /* _ROBCPROBOT_ARM6DOFIK_H_ */
