#include "state_estimator.h"
#include "attitude_pid.h"
#include "position_pid.h"
#include "maths.h"
#include "vl53l0x.h"
#include "stabilizer.h"

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

#define VELOCITY_LIMIT		(130.f)	/*速度限幅 单位cm/s*/

static float errorZ = 0.f;			/*位移误差*/
static float wBaro = 0.35f;			/*气压校正权重*/

static bool isRstHeight = false;	/*复位高度*/
static bool isRstAll = true;		/*复位估测*/

static float fusedHeight;			/*融合高度，起飞点为0*/
static float fusedHeightLpf = 0.f;	/*融合高度，低通*/
static float startBaroAsl = 0.f;	/*起飞点海拔*/


static estimator_t estimator = 
{
	.vAccDeadband = 4.0f,
	.accX = 0.f,
	.accY = 0.f,
	.accZ = 0.f,
	.velocityX = 0.0f,
	.velocityY = 0.0f,
	.velocityZ = 0.0f,
	.positonX = 0.0f,
	.positonY = 0.0f,
	.positonZ = 0.0f,
};


void positionEstimate(sensorData_t* sensorData, state_t* state, float dt) 
{	
	static float rangeLpf = 0.f;
	static float accZLpf = 0.f;			/*Z轴加速度低通*/
	
	float weight = wBaro;
	
	float relateHight = sensorData->baro.asl - startBaroAsl;	/*气压相对高度*/
	
	if(getModuleID() == OPTICAL_FLOW)	/*光流模块可用*/
	{
		vl53l0xReadRange(&sensorData->zrange);	/*读取激光数据*/
	
		rangeLpf += (sensorData->zrange.distance - rangeLpf) * 0.1f;	/*低通 单位cm*/		
			
		float quality = sensorData->zrange.quality;
	
		if(quality < 0.3f)	/*低于这个可行度，激光数据不可用*/
		{
			quality = 0.f;
		}else
		{
			weight = quality;
//			if(quality > 0.8f)
				startBaroAsl = sensorData->baro.asl - rangeLpf;
		}
		fusedHeight = rangeLpf * quality + (1.0f - quality) * relateHight;/*融合高度*/					
	}
	else	/*无光流模块*/
	{
		fusedHeight = relateHight;	/*融合高度*/
	}
	
	fusedHeightLpf += (fusedHeight - fusedHeightLpf) * 0.1f;	/*融合高度 低通*/
	
	if(isRstHeight)
	{	
		isRstHeight = false;
		
		weight = 0.95f;		/*增加权重，快速调整*/	
		
		startBaroAsl = sensorData->baro.asl;
		
		if(getModuleID() == OPTICAL_FLOW)
		{
			if(sensorData->zrange.distance < VL53L0X_MAX_RANGE)
			{
				startBaroAsl -= sensorData->zrange.distance;
				fusedHeight = sensorData->zrange.distance;
			}
		}
		
		estimator.positonZ = fusedHeight;
	}
	else if(isRstAll)
	{
		isRstAll = false;
		
		accZLpf = 0.f;
//		rangeLpf = 0.f;
	
		fusedHeight  = 0.f;
		fusedHeightLpf = 0.f;
		startBaroAsl = sensorData->baro.asl;
		
		if(getModuleID() == OPTICAL_FLOW)
		{
			if(sensorData->zrange.distance < VL53L0X_MAX_RANGE)
			{
				startBaroAsl -= sensorData->zrange.distance;
				fusedHeight = sensorData->zrange.distance;
			}
		}
		
		estimator.velocityZ = 0.f;
		estimator.positonZ = fusedHeight;
	}
	
	bool isKeyFlightLand = ((getCommanderKeyFlight()==true) || (getCommanderKeyland()==true));	/*定高飞或者降落状态*/
	
	float accZRemovalDead = applyDeadbandf(state->acc.z, estimator.vAccDeadband);/*去除死区的Z轴加速度*/
	accZLpf += (accZRemovalDead - accZLpf) * 0.1f;		/*低通*/
	
	if(isKeyFlightLand == true)		/*定高飞或者降落状态*/
		state->acc.z = constrainf(accZLpf, -1000.f, 1000.f);	/*加速度限幅*/
	else
		state->acc.z = accZRemovalDead;
	
	estimator.accZ = accZRemovalDead;
	estimator.accZ -= 0.02f * errorZ * weight * weight * dt;	/*补偿加速度*/		
	
	
	/*位置和速度估测*/
	estimator.positonZ += estimator.velocityZ * dt + estimator.accZ * dt * dt / 2.0f;
	estimator.velocityZ += estimator.accZ * dt;
	
	/*高度误差*/
	errorZ = fusedHeight - estimator.positonZ;		
	
	/*校正位移和速度*/
	float ewdt = errorZ * weight * dt;
	estimator.positonZ += ewdt;
	estimator.velocityZ += weight * ewdt;
	
	
	if(isKeyFlightLand == true)		/*定高飞或者降落状态*/
		estimator.velocityZ = constrainf(estimator.velocityZ, -VELOCITY_LIMIT, VELOCITY_LIMIT);	/*速度限幅 VELOCITY_LIMIT*/
	
	state->position.z = estimator.positonZ;	
	state->velocity.z = estimator.velocityZ;
}

/*读取融合高度 单位m*/	
float getFusedHeight(void)
{
	return (0.01f * fusedHeightLpf);
}

/*复位估测高度*/
void estRstHeight(void)
{
	isRstHeight = true;
}

/*复位所有估测*/
void estRstAll(void)
{
	isRstAll = true;
}


