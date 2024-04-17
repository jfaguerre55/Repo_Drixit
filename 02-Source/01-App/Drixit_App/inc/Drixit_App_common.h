/*
 * Drixit_App_common.h
 *
 *  Created on: 15 abr. 2024
 *      Author: JuanAguerre
 */

#ifndef DRIXIT_APP_COMMON_H_
#define DRIXIT_APP_COMMON_H_


#include <stdio.h>
#include <assert.h>
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



// Drixit App definitions
typedef 	uint32_t 	Sensor_Sample_Id;
typedef struct{
	Sensor_Sample_Id			id;
	Sensor_LIS3MDL_Value_t		values;
	uint16_t					crc;
	uint8_t						reserved[10];	// Keep the structure aligned. Size = 32 bytes
} SensorData_t;



// FreeRTOS tasks prototypes
void vTaskLedBlink(void * pvLed );							// Test Blinking LED
/* Data handling tasks */
void vTaskSensorAdqData(void * pvParameters);				// Aquisition task
void vTaskMemoryWriter(void * pvParameters);				// Memory write task
void vTaskMemoryReader(void * pvParameters);				// Memory read task
/* Communication tasks */
void vTaskUartReqManager(void * pvParameters);				// UART-rx. Task for handle IDs requests
void vTaskUartRespManager(void * pvParameters);				// UART-tx. Task for handle IDs responses
void vTaskButtonReqManager(void * pvParameters);			// Task for handle IDs requests from Sw button GPIO

bool System_OS_Init(void);


// Hardware devices prototypes
void * swISP_When_Pushed(void * pvParameters);				// Sw button pushed callback
void * swISP_When_Release(void * pvParameters);				// Sw button release callback
void * uart_rx_cb(void * pvParameters);						// UART rx callback
void * uart_tx_cb(void * pvParameters);						// UART tx callback
bool   System_Hardware_Init(void);



#endif /* DRIXIT_APP_COMMON_H_ */
