#include "system.h"
#include "stabilizer.h"
#include "sensors.h"
#include "sensfusion6.h"
#include "commander.h"
#include "anomal_detec.h"
#include "state_control.h"
#include "state_estimator.h"
#include "power_control.h"
#include "position_pid.h"
#include "flip.h"
#include "optical_flow.h"
#include "vl53l0x.h"
#include "maths.h"

/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"
#include "task.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * �������ȿ��ƴ���	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2018/6/22
 * �汾��V1.2
 * ��Ȩ���У�����ؾ���
 * Copyright(C) �������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

static bool isInit;

static setpoint_t 	setpoint;	/*����Ŀ��״̬*/
static sensorData_t sensorData;	/*����������*/
static state_t 		state;		/*������̬*/
static control_t 	control;	/*������Ʋ���*/

bool isAdjustFinished;
static u16 velModeTimes = 0;		/*����ģʽ����*/
static u16 absModeTimes = 0;		/*����ֵģʽ����*/
static float setHeight = 0.f;		/*�趨Ŀ��߶� ��λcm*/
static float coeVel = 0.f;			/*�趨�ٶ�ϵ�� ��λm/s*/


void stabilizerTask(void* param);

void stabilizerInit(void)
{
	if(isInit) return;

	stateControlInit();		/*��̬PID��ʼ��*/
	powerControlInit();		/*�����ʼ��*/

	isInit = true;
}

bool stabilizerTest(void)
{
	bool pass = true;

	pass &= stateControlTest();
	pass &= powerControlTest();

	return pass;
}

/*���ÿ��ٵ�������*/	
void setFastAdjustPosParam(u16 velTimes, u16 absTimes, float coeff, float height)
{
	if(velTimes != 0 && velModeTimes == 0)
	{
		coeVel = coeff;
		velModeTimes = velTimes;
	}
	if(absTimes != 0 && absModeTimes ==0)
	{
		setHeight = height;
		absModeTimes = absTimes;
	}		
}

/*���ٵ����߶�*/
static void fastAdjustPosZ(void)
{	
	if(velModeTimes > 0)
	{
		velModeTimes--;
		estRstHeight();
		setpoint.mode.z = modeVelocity;
		setpoint.velocity.z = coeVel * state.velocity.z;		
	}
	else if(absModeTimes > 0)
	{
		absModeTimes--;
		estRstAll();
		setpoint.mode.z = modeAbs;		
		setpoint.position.z = state.position.z + setHeight;
	}	
}

void stabilizerTask(void* param)
{
	u32 tick = 0;
	u32 lastWakeTime = getSysTickCnt();
	
	ledseqRun(SYS_LED, seq_alive);
	
	while(!sensorsAreCalibrated())
	{
		vTaskDelayUntil(&lastWakeTime, MAIN_LOOP_DT);
	}
	
	while(1) 
	{
		vTaskDelayUntil(&lastWakeTime, MAIN_LOOP_DT);		/*1ms������ʱ*/

		//��ȡ6�����ѹ���ݣ�500Hz��
		if (RATE_DO_EXECUTE(RATE_500_HZ, tick))
		{
			sensorsAcquire(&sensorData, tick);				/*��ȡ6�����ѹ����*/
		}

		//��Ԫ����ŷ���Ǽ��㣨250Hz��
		if (RATE_DO_EXECUTE(ATTITUDE_ESTIMAT_RATE, tick))
		{
			imuUpdate(sensorData.acc, sensorData.gyro, &state, ATTITUDE_ESTIMAT_DT);				
		}

		//λ��Ԥ�����㣨250Hz��
		if (RATE_DO_EXECUTE(POSITION_ESTIMAT_RATE, tick))
		{
			positionEstimate(&sensorData, &state, POSITION_ESTIMAT_DT);
		}
			
		//Ŀ����̬�ͷ���ģʽ�趨��100Hz��	
		if (RATE_DO_EXECUTE(RATE_100_HZ, tick))
		{
			commanderGetSetpoint(&setpoint, &state);	/*Ŀ�����ݺͷ���ģʽ�趨*/	
		}
		
		if (RATE_DO_EXECUTE(RATE_250_HZ, tick))
		{
			fastAdjustPosZ();	/*���ٵ����߶�*/
		}		
		
		/*��ȡ��������(100Hz)*/
		if (RATE_DO_EXECUTE(RATE_100_HZ, tick))
		{
			getFlowData(&state);	
		}
		
		/*�������(500Hz) �Ƕ���ģʽ*/
		if (RATE_DO_EXECUTE(RATE_500_HZ, tick) && (getCommanderCtrlMode() != 0x03))
		{
			flyerFlipCheck(&setpoint, &control, &state);	
		}
		
		/*�쳣���*/
		anomalDetec(&sensorData, &state, &control);			
		
		/*PID����*/	
		
		stateControl(&control, &sensorData, &state, &setpoint, tick);
				
		
		//���Ƶ�������500Hz��
		if (RATE_DO_EXECUTE(RATE_500_HZ, tick))
		{
			powerControl(&control);	
		}
		
		tick++;
	}
}


void getAttitudeData(attitude_t* get)
{
	get->pitch = -state.attitude.pitch;
	get->roll = state.attitude.roll;
	get->yaw = -state.attitude.yaw;
}

float getBaroData(void)
{
	return sensorData.baro.asl;
}

float getPosestimator_accZ(void)
{
	return state.acc.z;
}

float getPositionZ(void)
{
	return sensorData.baro.asl;
}


void getState(state_t* get)
{
	*get = state;
}

void getSensorData(sensorData_t* get)
{
	*get = sensorData;
}



