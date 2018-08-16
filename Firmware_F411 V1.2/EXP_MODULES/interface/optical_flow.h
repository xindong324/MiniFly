#ifndef __OPTICAL_FLOW_H
#define __OPTICAL_FLOW_H
#include "sys.h"
#include <stdbool.h>
#include "spi.h"
#include "stabilizer_types.h"
#include "module_mgt.h"

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

void opticalFlowPowerControl(bool state);	//光流电源控制
bool getFlowData(state_t *state);			//读取光流数据
void opticalFlowInit(void);		/*初始化光流模块*/
bool getOpDataState(void);		/*光流数据状态*/

#endif
