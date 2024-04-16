/*
 * Drixit_App_common.h
 *
 *  Created on: 15 abr. 2024
 *      Author: JuanAguerre
 */

#ifndef DRIXIT_APP_COMMON_H_
#define DRIXIT_APP_COMMON_H_


#include "stdio.h"
#include "chip.h"
#include <cr_section_macros.h>
#include "sensor_lis3mdl.h"
#include "flash_w25q80db.h"
#include "leds.h"
#include "sw_button.h"
#include "MCU43xx_m4_UART_0_2_3.h"
#include "MCU43xx_m4_SPIFI.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"


typedef 	uint32_t 	Sensor_Sample_Id;		// 0xFFFFFFFF = last ID


typedef struct{
	Sensor_LIS3MDL_Value_t		values;
	Sensor_Sample_Id			id;
	uint32_t					reserved;	// Keep the structure aligned (usar para CRC ??)
} SensorData_t;




#endif /* DRIXIT_APP_COMMON_H_ */
