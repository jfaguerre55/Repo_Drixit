/*
 * Drixit_HW_Devices.c
 *
 *  Created on: 16 abr. 2024
 *      Author: JuanAguerre
 */

#include "Drixit_App_common.h"


// FreeRTOS objects
extern QueueHandle_t 		xSensorDataQueue;						// Sensor data buffer
extern SemaphoreHandle_t	xMemoryAccessMutex;						// Memory mutex
extern QueueHandle_t 		xIdRequestQueue;
extern SemaphoreHandle_t	xButtonRequestBufferSemaphore;
extern SemaphoreHandle_t	xUartRequestSemaphore;
extern SemaphoreHandle_t	xUartTxMutex;



Sensor_LIS3MDL_t				sensorLIS3MDL;
Sensor_LIS3MDL_Config_Init_t	sensorLIS3MDL_config = {
		2, 1,			/*!< Port/Bit DATA READY pin */
		1, 5,			/*!< I2C perif number I2C / I2C ISRs Priority  */
		100000,	0x1C,	/*!< I2C BPS / I2C address */
		2, 3,    		/*!< Port/Bit SDA */
		2, 4,			/*!< Port/Bit SCL */
};


Flash_W25Q80DB_t	flash_W25Q80DB;


LED_t 					led_red;
LED_Config_Init_t		led_red_config = {
		6, 11, LED_RED,		// Port - Pin - Color
		LED_MODE_ONF,		// Modo (on/off o level)
		LED_TURN_ON_1,		// Se prende por 1
		100					// Level inicial
};



Sw_Button_t				swISP;
Sw_Button_Init_t		swISP_config = {
		2 , 7,										// Port - Pin
		SW_BUTTON_NA, SW_R_PULLUP, DEBOUNCE_ON,		// NO/NA - PullUp/PullDown - Debounce ON/OFF
		swISP_When_Pushed, NULL,					// Callback cuando se presiona y sus argumentos
		swISP_When_Release, NULL					// Callback cuando se libera y sus argumentos
};


void * swISP_When_Pushed(void * pvParameters){

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	xSemaphoreGiveFromISR( xButtonRequestBufferSemaphore, &xHigherPriorityTaskWoken );

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

	return NULL;
}


void * swISP_When_Release(void * pvParameters){
	return NULL;
}




MCU_UART_t 					uart;
MCU_UART_Init_Config_t 		uart_config={
	// General
	.mode		= MCU_UART_Mode_None,
	.rate_bps 	= 115200,
	.data_len 	= MCU_UART_8BitsPerChar,
	.partity 	= MCU_UART_Parity_Disabled,
	.stop_bit 	= MCU_UART_OneStopBit,
	// Hardware
	.port_num 	= 0,
	.int_priority= 10,
	.rx_mode 	= MCU_UART_Rx_Mode_Buffer,
	// Pines uC (Tx - Rx - Dir)
	.tx_port 	= 2,
	.tx_pin 	= 0,
	.rx_port 	= 2,
	.rx_pin 	= 1,
	// RS485 support
	.rs485_add	= 0xFF,
	.dir_port 	= 255,
	.dir_pin 	= 255
};


/* UART Rx callback function. This cb will be executed when 4 bytes are received. */
void * uart_rx_cb(void * pvParameters){

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	xSemaphoreGiveFromISR( xUartRequestSemaphore, &xHigherPriorityTaskWoken );

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

	return NULL;
}


void * uart_tx_cb(void * pvParameters){

	return NULL;

}


/* Function to hardware init */
bool System_Hardware_Init(void){

	bool init_err = false;

    LED_Config(&led_red,   	 &led_red_config);

    SW_BUTTON_Config(&swISP, &swISP_config );

    MCU_UART_Config(&uart, &uart_config);
    MCU_UART_Config_Rx_Callback(&uart, uart_rx_cb, NULL, sizeof(Sensor_Sample_Id));
    MCU_UART_Enable_Rx_Interrupt(&uart);
    MCU_UART_Config_Tx_Callback(&uart, uart_tx_cb, NULL);
    MCU_UART_Enable_NVIC(&uart, 6);

    LIS3MDL_Init(&sensorLIS3MDL, &sensorLIS3MDL_config);

    Flash_W25Q80DB_Init(&flash_W25Q80DB);

	return init_err;

}


