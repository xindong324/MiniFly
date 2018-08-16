#include "string.h"
#include "stdbool.h"
#include "optical_flow.h"
#include "config_param.h"
#include "commander.h"
#include "delay.h"
#include "maths.h"
#include "vl53l0x.h"
#include "state_estimator.h"

#include "filter.h"
#include "arm_math.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 光流模块驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/5/2
 * 版本：V1.2
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

#define NCS_PIN					PAout(8)
#define OPTICAL_POWER_ENABLE	PBout(0)

#define RESOLUTION				(0.002131946f)	/*1m高度下 1个像素对应的位移，单位m*/
#define OULIER_LIMIT 			100


static bool isInit = false;
static bool isOpFlowOk = false;		/*光流是否正常*/
static bool isOpDataValid = false;	/*光流数据是否可用*/
static u8 outlierCount = 0;			/*数据不可用计数*/
static float xyVelLimit = 1.5f;		/*xy速度限幅 单位m/s*/

TaskHandle_t opFlowTaskHandle = NULL;

#if defined(__CC_ARM) 
	#pragma anon_unions	/*用于支持结构体联合体*/
#endif

typedef __packed struct motionBurst_s 
{
	__packed union 
	{
		uint8_t motion;
		__packed struct 
		{
			uint8_t frameFrom0    : 1;
			uint8_t runMode       : 2;
			uint8_t reserved1     : 1;
			uint8_t rawFrom0      : 1;
			uint8_t reserved2     : 2;
			uint8_t motionOccured : 1;
		};
	};

	uint8_t observation;
	int16_t deltaX;
	int16_t deltaY;

	uint8_t squal;

	uint8_t rawDataSum;
	uint8_t maxRawData;
	uint8_t minRawData;

	uint16_t shutter;
} motionBurst_t;

motionBurst_t currentMotion;

static void InitRegisters(void);

//光流电源控制
void opticalFlowPowerControl(bool state)
{
	if(state == true)
		OPTICAL_POWER_ENABLE = true;
	else
		OPTICAL_POWER_ENABLE = false;
}

static void registerWrite(uint8_t reg, uint8_t value)
{
	// 最高位为1 写寄存器
	reg |= 0x80u;

	spiBeginTransaction();
	
	NCS_PIN = 0;
	
	delay_us(50);
	spiExchange(1, &reg, &reg);
	delay_us(50);
	spiExchange(1, &value, &value);
	delay_us(50);

	NCS_PIN = 1;
	
	spiEndTransaction();
	delay_us(200);
}

static uint8_t registerRead(uint8_t reg)
{
	uint8_t data = 0;

	// 最高位为0 读寄存器
	reg &= ~0x80u;

	spiBeginTransaction();
	
	NCS_PIN = 0;
	
	delay_us(50);	
	spiExchange(1, &reg, &reg);
	delay_us(500);
	spiExchange(1, &data, &data);	
	delay_us(50);
	
	NCS_PIN = 1;
	
	spiEndTransaction();
	delay_us(200);

	return data;
}

static void readMotion(motionBurst_t * motion)
{
	uint8_t address = 0x16;

	spiBeginTransaction();
	
	NCS_PIN = 0;
	
	delay_us(50);	
	spiExchange(1, &address, &address);
	delay_us(50);
	spiExchange(sizeof(motionBurst_t), (uint8_t*)motion, (uint8_t*)motion);	
	delay_us(50);
	
	NCS_PIN = 1;
	
	spiEndTransaction();
	delay_us(50);

	uint16_t realShutter = (motion->shutter >> 8) & 0x0FF;
	realShutter |= (motion->shutter & 0x0ff) << 8;
	motion->shutter = realShutter;
}

static void InitRegisters(void)
{	
	registerWrite(0x7F, 0x00);
	registerWrite(0x61, 0xAD);
	registerWrite(0x7F, 0x03);
	registerWrite(0x40, 0x00);
	registerWrite(0x7F, 0x05);
	registerWrite(0x41, 0xB3);
	registerWrite(0x43, 0xF1);
	registerWrite(0x45, 0x14);
	registerWrite(0x5B, 0x32);
	registerWrite(0x5F, 0x34);
	registerWrite(0x7B, 0x08);
	registerWrite(0x7F, 0x06);
	registerWrite(0x44, 0x1B);
	registerWrite(0x40, 0xBF);
	registerWrite(0x4E, 0x3F);
	registerWrite(0x7F, 0x08);
	registerWrite(0x65, 0x20);
	registerWrite(0x6A, 0x18);
	registerWrite(0x7F, 0x09);
	registerWrite(0x4F, 0xAF);
	registerWrite(0x5F, 0x40);
	registerWrite(0x48, 0x80);
	registerWrite(0x49, 0x80);
	registerWrite(0x57, 0x77);
	registerWrite(0x60, 0x78);
	registerWrite(0x61, 0x78);
	registerWrite(0x62, 0x08);
	registerWrite(0x63, 0x50);
	registerWrite(0x7F, 0x0A);
	registerWrite(0x45, 0x60);
	registerWrite(0x7F, 0x00);
	registerWrite(0x4D, 0x11);
	registerWrite(0x55, 0x80);
	registerWrite(0x74, 0x1F);
	registerWrite(0x75, 0x1F);
	registerWrite(0x4A, 0x78);
	registerWrite(0x4B, 0x78);
	registerWrite(0x44, 0x08);
	registerWrite(0x45, 0x50);
	registerWrite(0x64, 0xFF);
	registerWrite(0x65, 0x1F);
	registerWrite(0x7F, 0x14);
	registerWrite(0x65, 0x67);
	registerWrite(0x66, 0x08);
	registerWrite(0x63, 0x70);
	registerWrite(0x7F, 0x15);
	registerWrite(0x48, 0x48);
	registerWrite(0x7F, 0x07);
	registerWrite(0x41, 0x0D);
	registerWrite(0x43, 0x14);
	registerWrite(0x4B, 0x0E);
	registerWrite(0x45, 0x0F);
	registerWrite(0x44, 0x42);
	registerWrite(0x4C, 0x80);
	registerWrite(0x7F, 0x10);
	registerWrite(0x5B, 0x02);
	registerWrite(0x7F, 0x07);
	registerWrite(0x40, 0x41);
	registerWrite(0x70, 0x00);

	vTaskDelay(10); // delay 10ms

	registerWrite(0x32, 0x44);
	registerWrite(0x7F, 0x07);
	registerWrite(0x40, 0x40);
	registerWrite(0x7F, 0x06);
	registerWrite(0x62, 0xF0);
	registerWrite(0x63, 0x00);
	registerWrite(0x7F, 0x0D);
	registerWrite(0x48, 0xC0);
	registerWrite(0x6F, 0xD5);
	registerWrite(0x7F, 0x00);
	registerWrite(0x5B, 0xA0);
	registerWrite(0x4E, 0xA8);
	registerWrite(0x5A, 0x50);
	registerWrite(0x40, 0x80);
	
//	/*初始化LED_N*/
//	registerWrite(0x7F, 0x0E);
//	registerWrite(0x72, 0x0F);
//	registerWrite(0x7F, 0x00);
}

static float lpfVal = 0.15f;						/*低通系数*/
static float flowPixelSumX, flowPixelSumY;			/*累积像素*/
static float flowPixelLpfX=0.f, flowPixelLpfY=0.f;	/*累积像素低通*/
static float flowPixelCompX=0.f, flowPixelCompY=0.f;/*倾角补偿*/

static float flowDataDx=0.f, flowDataDy=0.f;		/*2帧之间位移变化量，单位m*/
static float flowDataVx=0.f, flowDataVy=0.f;		/*速度，单位m/s*/
static float flowDataOutX=0.f, flowDataOutY=0.f;	/*补偿像素*/
static float lastOutX=0.f, lastOutY=0.f;			/*上一次的补偿像素*/

void opticalFlowTask(void *param)
{	
	static u16 count = 0;	
	u32 lastWakeTime = getSysTickCnt();
		
	isOpFlowOk = true;
	
	while(1) 
	{
		vTaskDelayUntil(&lastWakeTime, 10);		/*100Hz 周期延时*/
		
		readMotion(&currentMotion);

		if(currentMotion.minRawData == 0 && currentMotion.maxRawData == 0)
		{
			if(count++ > 100 && isOpFlowOk == true)
			{
				count = 0;
				isOpFlowOk = false;				/*光流出错*/
				vTaskSuspend(opFlowTaskHandle);	/*挂起光流任务*/
			}		
		}else
		{
			count = 0;
		}
		/*连续2帧之间的像素变化，根据实际安装方向调整 (pitch:x)  (roll:y)*/
		int16_t pixelDx = currentMotion.deltaY;
		int16_t pixelDy = -currentMotion.deltaX;

		if (ABS(pixelDx) < OULIER_LIMIT && ABS(pixelDy) < OULIER_LIMIT) 
		{
			flowPixelSumX += pixelDx;
			flowPixelSumY += pixelDy;
			flowPixelLpfX += (flowPixelSumX - flowPixelLpfX) * lpfVal;
			flowPixelLpfY += (flowPixelSumY - flowPixelLpfY) * lpfVal;		
		} 
		else 
		{
			outlierCount++;
		}
	}
}

/*复位光流数据*/
static void resetOpData(void)
{
	flowPixelSumX = 0.f;
	flowPixelSumY = 0.f;
	flowPixelLpfX = 0.f;
	flowPixelLpfY = 0.f;
	flowPixelCompX=0.f;
	flowPixelCompY=0.f;
	
	flowDataOutX = 0;
	flowDataOutY = 0;
	lastOutX = 0;
	lastOutY = 0;
}

bool getFlowData(state_t *state)
{
	static u8 cnt =  0;
	float height = getFusedHeight();/*读取高度信息 单位m*/
	
	if(isOpFlowOk && height < 4.0f)	/*4m范围内，光流可用*/
	{
		isOpDataValid = true;
		
		float tempCoefficient = RESOLUTION * height;
		float tanRoll = tanf(state->attitude.roll * DEG2RAD);
		float tanPitch = tanf(state->attitude.pitch * DEG2RAD);
		
		flowPixelCompX += (480.f * tanPitch - flowPixelCompX) * lpfVal;	/*倾角补偿，负方向*/
		flowPixelCompY += (480.f * tanRoll - flowPixelCompY) * lpfVal;
		flowDataOutX = (flowPixelLpfX + flowPixelCompX);	/*实际输出像素*/
		flowDataOutY = (flowPixelLpfY + flowPixelCompY);
		
		if(height < 0.08f)	/*光流测量范围大于8cm*/
		{
			tempCoefficient = 0.0f;
		}
		flowDataDx = tempCoefficient * (flowDataOutX - lastOutX);	/*2帧之间位移变化量，单位m*/
		flowDataDy = tempCoefficient * (flowDataOutY - lastOutY);	
		lastOutX = flowDataOutX;	/*上一次实际输出像素*/
		lastOutY = flowDataOutY;
		flowDataVx = 100.f * (flowDataDx);	/*速度 m/s*/
		flowDataVy = 100.f * (flowDataDy);	
		state->velocity.x += (flowDataVx - state->velocity.x) * 0.08f;	/*速度LPF*/
		state->velocity.y += (flowDataVy - state->velocity.y) * 0.08f;	/*速度LPF*/
		state->position.x += flowDataDx;	/*累积位移*/
		state->position.y += flowDataDy;	/*累积位移*/

		state->velocity.x = constrainf(state->velocity.x, -xyVelLimit, xyVelLimit);	/*速度限幅 单位m/s*/
		state->velocity.y = constrainf(state->velocity.y, -xyVelLimit, xyVelLimit);
	}
	else if(isOpDataValid == true)
	{
		if(cnt++ > 100)
		{
			cnt = 0;
			isOpDataValid = false;
		}
	
		resetOpData();
		state->velocity.x = 0.f;	/*清零*/
		state->velocity.y = 0.f;	
		state->position.x = 0.f;	
		state->position.y = 0.f;	
	}else
	{
		cnt= 0;
	}
	
	return isOpFlowOk;	/*返回光流状态*/
}
/*初始化光流模块*/
void opticalFlowInit(void)
{
	if (!isInit) /*第一次初始化通用IO*/
	{
		GPIO_InitTypeDef GPIO_InitStructure;

		//初始化CS引脚	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能时钟
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能时钟
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;	
		GPIO_Init(GPIOA, &GPIO_InitStructure);		

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
		GPIO_Init(GPIOB, &GPIO_InitStructure);		
	}
	else 
	{
		resetOpData();
		isOpFlowOk = true;				
	}
	
	opticalFlowPowerControl(true);	/*打开电源*/
	vTaskDelay(50);
	
	NCS_PIN = 1;
	spi2Init();
	vTaskDelay(40);

	uint8_t chipId = registerRead(0);
	uint8_t invChipId = registerRead(0x5f);
//	printf("Motion chip is: 0x%x\n", chipId);
//	printf("si pihc noitoM: 0x%x\n", invChipId);

	// 上电复位
	registerWrite(0x3a, 0x5a);
	vTaskDelay(5);

	InitRegisters();
	vTaskDelay(5);
	
	if (isInit) 
	{
		vTaskResume(opFlowTaskHandle);	/*恢复光流任务*/
	}
	else if(opFlowTaskHandle == NULL)
	{
		xTaskCreate(opticalFlowTask, "OPTICAL_FLOW", 300, NULL, 3, &opFlowTaskHandle);	/*创建光流模块任务*/
	}		
	
	vl53l0xInit();	/*初始化vl53l0x*/
	
	isInit = true;
}

bool getOpDataState(void)
{
	return isOpDataValid;
}
	
	

