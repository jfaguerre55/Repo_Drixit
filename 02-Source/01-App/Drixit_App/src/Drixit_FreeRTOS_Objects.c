/*
 * Drixit_FreeRTOS_Objects.c
 *
 *  Created on: 16 abr. 2024
 *      Author: JuanAguerre
 */


#include "Drixit_App_common.h"



// Hardware devices
extern Sensor_LIS3MDL_t				sensorLIS3MDL;
extern Sensor_LIS3MDL_Config_Init_t	sensorLIS3MDL_config;

extern LED_t 						led_red;
extern LED_Config_Init_t			led_red_config;

extern Sw_Button_t					swISP;
extern Sw_Button_Init_t				swISP_config;

extern MCU_UART_t 					uart;
extern MCU_UART_Init_Config_t 		uart_config;




QueueHandle_t 		xSensorDataQueue;						// Sensor data buffer
SemaphoreHandle_t	xMemoryAccessMutex;						// Memory mutex
QueueHandle_t 		xIdRequestQueue;
SemaphoreHandle_t	xButtonRequestBufferSemaphore;
SemaphoreHandle_t	xUartRequestSemaphore;
SemaphoreHandle_t	xUartTxMutex;





void vTaskMemoryWriter(void * pvParameters)
{
	BaseType_t 		xStatus;
	SensorData_t	sensor_data;
	uint8_t 		p_byte=66;

	for( ;; )
	{
		xSemaphoreTake( xMemoryAccessMutex, portMAX_DELAY );
		{

		xStatus = xQueueReceive( xSensorDataQueue, &sensor_data, portMAX_DELAY );
		if( xStatus == pdPASS )	{}

		MCU_UART_Send_Byte(&uart, &p_byte);
		printf( "Writing ID=%d x=%d\n", sensor_data.id, sensor_data.values.x );
//		MCU_UART_Send_Array(&uart, const uint8_t * array, uint32_t array_length);
		}
		xSemaphoreGive( xMemoryAccessMutex );
	}

}


void vTaskMemoryReader(void * pvParameters)
{

//	BaseType_t 		xStatus;

	for( ;; )
	{
		xSemaphoreTake( xMemoryAccessMutex, portMAX_DELAY );
		{
		printf( "Reading" );
		}
		xSemaphoreGive( xMemoryAccessMutex );
	}

}



void vTaskUartRxManager(void * pvParameters)
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







void vTaskLedBlink( void * pvLed )
{
	 for( ;; )
	 {
		vTaskDelay(pdMS_TO_TICKS(500));
		LED_Toogle((LED_t *)pvLed);




	 }
}



void vTaskSensorAdquisitionData( void * pvParameters )
{
	BaseType_t 		xStatus;
	int16_t 		sensor_reading_buff[4];
	SensorData_t	sensor_data;
	uint32_t		id_counter=0;

	for( ;; )
	{
		vTaskDelay(pdMS_TO_TICKS(1000));

		LIS3MDL_Get_XYZT(&sensorLIS3MDL, &sensor_reading_buff);

		sensor_data.id = id_counter;
		id_counter++;
		sensor_data.values.x = 		55; //sensor_reading_buff[0];
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

		 xSemaphoreTake( xButtonRequestBufferSemaphore, portMAX_DELAY );

		 xStatus = xQueueSendToBack( xIdRequestQueue, &id_req, 0 );
		 if( xStatus != pdPASS ){}

	}
}





bool System_OS_Init(void){

    // Semaphore
    xButtonRequestBufferSemaphore = xSemaphoreCreateCounting(BUTTON_REQUEST_MAX, 0);
    if( xButtonRequestBufferSemaphore != NULL ){}

    xUartRequestSemaphore = xSemaphoreCreateBinary();
    if( xUartRequestSemaphore != NULL ){}

    // Queues
    xSensorDataQueue = xQueueCreate( 10, sizeof( SensorData_t ) );
    if( xSensorDataQueue != NULL );

    xIdRequestQueue = xQueueCreate( 100, sizeof( uint32_t ) );
    if( xIdRequestQueue != NULL );

    // Mutex
    xMemoryAccessMutex = xSemaphoreCreateMutex();
    if( xMemoryAccessMutex != NULL );

    xUartTxMutex = xSemaphoreCreateMutex();
	if( xUartTxMutex != NULL );

    xTaskCreate( vTaskLedBlink, "LedBlinkRed", configMINIMAL_STACK_SIZE, &led_red  , 1, NULL );
    xTaskCreate( vTaskSensorAdquisitionData, "AdquisitionSensorData", configMINIMAL_STACK_SIZE, NULL , 25, NULL );
    xTaskCreate( vTaskButtonHMI, "ButtonHMI", configMINIMAL_STACK_SIZE, NULL , 17, NULL );
    xTaskCreate( vTaskUartRxManager, "CommManager", configMINIMAL_STACK_SIZE, NULL , 20, NULL );

    xTaskCreate( vTaskMemoryWriter, "MemotyWrite", configMINIMAL_STACK_SIZE, NULL , 5, NULL );
    xTaskCreate( vTaskMemoryReader, "MemotyWrite", configMINIMAL_STACK_SIZE, NULL , 10, NULL );

    return true;
}



