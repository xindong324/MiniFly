#ifndef __STATE_ESTIMATOR_H
#define __STATE_ESTIMATOR_H
#include "stabilizer_types.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 姿态估测代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/6/22
 * 版本：V1.2
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

typedef struct
{
	float vAccDeadband; /* 加速度死区 */
	float accX;			/* x 方向估测加速度 单位(cm/s/s)*/
	float accY;			/* y 方向估测加速度 单位(cm/s/s)*/
	float accZ;			/* z 方向估测加速度 单位(cm/s/s)*/
	float velocityX;	/* x 方向估测速度 单位(cm/s)*/
	float velocityY;	/* y 方向估测速度 单位(cm/s)*/
	float velocityZ;	/* z 方向估测速度 单位(cm/s)*/
	float positonX; 	/* x 方向估测位移 单位(cm)*/
	float positonY; 	/* x 方向估测位移 单位(cm)*/
	float positonZ; 	/* x 方向估测位移 单位(cm)*/
} estimator_t;

void positionEstimate(sensorData_t* sensorData, state_t* state, float dt);	
float getFusedHeight(void);	/*读取融合高度*/
void estRstHeight(void);	/*复位估测高度*/
void estRstAll(void);		/*复位所有估测*/

#endif /* __STATE_ESTIMATOR_H */


