/*
===============================================================================
 Name        : Drixit_App.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

#include "stdio.h"
#include "chip.h"
#include <cr_section_macros.h>
#include "sensor_lis3mdl.h"
#include "leds.h"
#include "sw_button.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

Sensor_LIS3MDL_t				sensorLIS3MDL;
Sensor_LIS3MDL_Config_Init_t	sensorLIS3MDL_config = {
		2,				/*!< Port DATA READY pin */
		1,				/*!< Bit DATA READY pin*/
		1,				/*!< I2C perif number I2C */
		5,				/*!< I2C ISRs Priority */
		100000,			/*!< I2C BPS */
		0x1C,			/*!< I2C address */
		2,    			/*!< Port SDA */
		3,				/*!< Bit SDA */
		2,				/*!< Port SCL */
		4				/*!< Bit SCL */
};


void vTaskLedBlink( void * pvLed );


LED_t 				led_red;
LED_Config_Init_t	led_red_config = {
		2, 11, LED_RED,		// Port - Pin - Color
		LED_MODE_ONF,		// Modo (on/off o level)
		LED_TURN_ON_1,		// Se prende por 1
		100					// Level inicial
};


Sw_Button_t				tec1;
void * 					TEC1_When_Pushed(void *);
void *					TEC1_When_Release(void *);
Sw_Button_Init_t		tec1_config = {
		1 , 0,										// Port - Pin
		SW_BUTTON_NA, SW_R_PULLUP, DEBOUNCE_ON,		// NO/NA - PullUp/PullDown - Debounce ON/OFF
		TEC1_When_Pushed, NULL,						// Callback cuando se presiona y sus argumentos
		TEC1_When_Release, NULL						// Callback cuando se libera y sus argumentos
};


int main(void) {

    SystemCoreClockUpdate();

    LED_Config(&led_red,   	 &led_red_config);
    SW_BUTTON_Config(&tec1, &tec1_config );
    LIS3MDL_Init(&sensorLIS3MDL, &sensorLIS3MDL_config);

    xTaskCreate( vTaskLedBlink, "LedBlinkRed", configMINIMAL_STACK_SIZE, &led_red  , 5, NULL );

    vTaskStartScheduler();

    while(1) {

    }


    return 0 ;
}




void vTaskLedBlink( void * pvLed )
{
	 for( ;; )
	 {
		vTaskDelay(pdMS_TO_TICKS(500));
		LED_Toogle((LED_t *)pvLed);
		printf("Led Task\n");

	 }
}



void * TEC1_When_Pushed(void * p){
	return NULL;
}


void * TEC1_When_Release(void * p){


	return NULL;
}
