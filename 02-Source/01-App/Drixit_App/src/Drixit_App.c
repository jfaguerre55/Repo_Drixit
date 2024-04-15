/*
===============================================================================
 Name        : Drixit_App.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

#include "Drixit_App_common.h"



Sensor_LIS3MDL_t				sensorLIS3MDL;
Sensor_LIS3MDL_Config_Init_t	sensorLIS3MDL_config = {
		2, 1,			/*!< Port/Bit DATA READY pin */
		1, 5,			/*!< I2C perif number I2C / I2C ISRs Priority  */
		100000,	0x1C,	/*!< I2C BPS / I2C address */
		2, 3,    		/*!< Port/Bit SDA */
		2, 4,			/*!< Port/Bit SCL */
};




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

MCU_UART_t 				uart;
MCU_UART_Init_Config_t 	uart_config;
void * uart_rx_cb(void * args);
void * uart_tx_cb(void * args);


// FreeRTOS objects
// Task prototypes
void vTaskLedBlink(void * pvLed );
void vTaskSensorAdquisitionData(void * pvParameters);
void vTaskButtonHMI(void * pvParameters);
void vTaskCommunicationManager(void * pvParameters);
// Queues handlers
QueueHandle_t 		xSensorDataQueue;
QueueHandle_t 		xIdRequestQueue;
SemaphoreHandle_t	xButtonRequestSemaphore;
SemaphoreHandle_t	xUartRequestSemaphore;



int main(void) {

    SystemCoreClockUpdate();

    LED_Config(&led_red,   	 &led_red_config);
    SW_BUTTON_Config(&tec1, &tec1_config );

    // General
    uart_config.mode		= MCU_UART_Mode_None;
    uart_config.rate_bps 	= 115200;
    uart_config.data_len 	= MCU_UART_8BitsPerChar;
    uart_config.partity 	= MCU_UART_Parity_Disabled;
    uart_config.stop_bit 	= MCU_UART_OneStopBit;
    // Hardware
    uart_config.port_num 	= 0;
    uart_config.int_priority= 10;
    uart_config.rx_mode 	= MCU_UART_Rx_Mode_Buffer;
    // Pines uC (Tx - Rx - Dir)
    uart_config.tx_port 	= 0;
    uart_config.tx_pin 		= 25;
    uart_config.rx_port 	= 0;
    uart_config.rx_pin 		= 24;
    // RS485 support
    uart_config.rs485_add	= 0xFF;
    uart_config.dir_port 	= 255;
    uart_config.dir_pin 	= 255;
    MCU_UART_Config(&uart, &uart_config);
    MCU_UART_Config_Rx_Callback(&uart, uart_rx_cb, NULL, sizeof(Sensor_Sample_Id));
    MCU_UART_Enable_Rx_Interrupt(&uart);
    MCU_UART_Config_Tx_Callback(&uart, uart_tx_cb, NULL);
    MCU_UART_Enable_NVIC(&uart, 6);

    LIS3MDL_Init(&sensorLIS3MDL, &sensorLIS3MDL_config);

    // Semaphore
    xButtonRequestSemaphore = xSemaphoreCreateBinary();
    if( xButtonRequestSemaphore != NULL ){}

    xUartRequestSemaphore = xSemaphoreCreateBinary();
    if( xUartRequestSemaphore != NULL ){}

    // Queues
    xSensorDataQueue = xQueueCreate( 10, sizeof( SensorData_t ) );
    if( xSensorDataQueue != NULL );

    xIdRequestQueue = xQueueCreate( 100, sizeof( uint32_t ) );
    if( xIdRequestQueue != NULL );

    xTaskCreate( vTaskLedBlink, "LedBlinkRed", configMINIMAL_STACK_SIZE, &led_red  , 1, NULL );
    xTaskCreate( vTaskSensorAdquisitionData, "AdquisitionSensorData", configMINIMAL_STACK_SIZE, NULL , 25, NULL );
    xTaskCreate( vTaskButtonHMI, "ButtonHMI", configMINIMAL_STACK_SIZE, NULL , 17, NULL );
    xTaskCreate( vTaskCommunicationManager, "CommManager", configMINIMAL_STACK_SIZE, NULL , 20, NULL );


    vTaskStartScheduler();

    while(1) {

    }


    return 0 ;
}






void vTaskCommunicationManager(void * pvParameters)
{

	BaseType_t 		xStatus;
	uint8_t rx_uart_array[10];
	Sensor_Sample_Id	id_req = 0;

	for( ;; )
	{
		 xSemaphoreTake( xUartRequestSemaphore, portMAX_DELAY );

		 // Leer el buffer de la UART
		 MCU_UART_Read_Array(&uart, rx_uart_array, 10);


		 // Validar el ID
		 if(rx_uart_array==0)
			 id_req = rx_uart_array[0];

		 // Add ID to the queue
		 xStatus = xQueueSendToBack( xIdRequestQueue, &id_req, 0 );
		 if( xStatus != pdPASS ){}
	}

}



void * uart_rx_cb(void * args){

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	xSemaphoreGiveFromISR( xUartRequestSemaphore, &xHigherPriorityTaskWoken );

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

	return NULL;


}



void * uart_tx_cb(void * args){

	return NULL;

}



void vTaskLedBlink( void * pvLed )
{
	 for( ;; )
	 {
		vTaskDelay(pdMS_TO_TICKS(500));
		LED_Toogle((LED_t *)pvLed);
		printf("Led Tabdbrbrsk\n");
	 }
}



void vTaskSensorAdquisitionData( void * pvParameters )
{
	BaseType_t 		xStatus;
	int16_t 		sensor_reading_buff[4];
	SensorData_t	sensor_data;

	for( ;; )
	{
		vTaskDelay(pdMS_TO_TICKS(1000));

		LIS3MDL_Get_XYZT(&sensorLIS3MDL, &sensor_reading_buff);

		sensor_data.id = 0;
		sensor_data.values.x = 		sensor_reading_buff[0];
		sensor_data.values.y = 		sensor_reading_buff[1];
		sensor_data.values.z = 		sensor_reading_buff[2];
		sensor_data.values.temp = 	sensor_reading_buff[3];

		xStatus = xQueueSendToBack( xSensorDataQueue, &sensor_data, 0 );
		if( xStatus != pdPASS ){}


	}
}





void vTaskButtonHMI( void * pvParameters )
{
	BaseType_t 			xStatus;
	Sensor_Sample_Id	id_req = 0xFFFFFFFF;

	for( ;; )
	{

		 xSemaphoreTake( xButtonRequestSemaphore, portMAX_DELAY );

		 xStatus = xQueueSendToBack( xIdRequestQueue, &id_req, 0 );
		 if( xStatus != pdPASS ){}

	}
}



void * TEC1_When_Pushed(void * p){

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	xSemaphoreGiveFromISR( xButtonRequestSemaphore, &xHigherPriorityTaskWoken );

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

	return NULL;
}


void * TEC1_When_Release(void * p){


	return NULL;
}
