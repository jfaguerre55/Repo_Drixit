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


SensorData_t		last_Sensor_Data;


QueueHandle_t 		xSensorDataQueue;						// Sensor data buffer
SemaphoreHandle_t	xMemoryAccessMutex;						// Memory mutex

QueueHandle_t 		xIdRequestQueue;
QueueHandle_t 		xIdResponseQueue;

SemaphoreHandle_t	xButtonRequestBufferSemaphore;
SemaphoreHandle_t	xUartRequestSemaphore;
SemaphoreHandle_t	xUartTxMutex;


/* Blinking LED for debug */
void vTaskLedBlink( void * pvLed )
{
	 for( ;; )
	 {
		vTaskDelay(pdMS_TO_TICKS(500));
		LED_Toogle((LED_t *)pvLed);
	 }
}




/* This task wakes up every 1 second and read a sample from the LIS3MDL sensor */
void vTaskSensorAdqData( void * pvParameters )
{
	float 			sensor_reading_buff[4];
	SensorData_t	sensor_data;
	uint32_t		id_counter=0;

	for( ;; )
	{
		/* Wait one second to read the sensor */
		vTaskDelay(pdMS_TO_TICKS(1000));
		LIS3MDL_Get_XYZT(&sensorLIS3MDL, &sensor_reading_buff);

		/* Construct the SensorData_t type */
		//TODO: construct the ID
		sensor_data.id = id_counter++;
		sensor_data.values.x = 		sensor_reading_buff[0];
		sensor_data.values.y = 		sensor_reading_buff[1];
		sensor_data.values.z = 		sensor_reading_buff[2];
		sensor_data.values.temp = 	sensor_reading_buff[3];
		sensor_data.crc = 0xFFFF;

		/* Send the reading to the queu */
		xQueueSendToBack( xSensorDataQueue, &sensor_data, 0 );
	}
}




/* Memory manager writer task. This task waits for sensor data and save it into the flash memory */
void vTaskMemoryWriter(void * pvParameters)
{
	SensorData_t	sensor_data;

	for( ;; )
	{
		/* Read sensor data queue */
		xQueueReceive( xSensorDataQueue, &sensor_data, portMAX_DELAY );

		/* Try to take control of the flash memory to save the sensor reading */
		xSemaphoreTake( xMemoryAccessMutex, portMAX_DELAY );
		{
			//TODO: memory access
			// Flash_W25Q80DB_Write_Array();
			printf( "Writing Flash ID=%d x=%d\n", sensor_data.id, sensor_data.values.x );
		}
		xSemaphoreGive( xMemoryAccessMutex );

		/* Update the last sensor data that was written in flash memory */
		taskENTER_CRITICAL();
		{
			last_Sensor_Data = sensor_data;
		}
		taskEXIT_CRITICAL();
	}

}



/* Memory manager writer task. This task waits for sensor data and save it into the flash memory */
void vTaskMemoryReader(void * pvParameters)
{
	Sensor_Sample_Id	id_req;
	SensorData_t 		sensor_data;

	for( ;; )
	{
		/* Read ID request from queue */
		xQueueReceive( xIdRequestQueue, &id_req, portMAX_DELAY );

		/* Try to take control of the flash memory to read de IDs values */
		xSemaphoreTake( xMemoryAccessMutex, portMAX_DELAY );
		{
			// Flash_W25Q80DB_Read_Array(&sensor_data);
			//TODO: memory access
		}
		xSemaphoreGive( xMemoryAccessMutex );

		/* Send ID+value to response queue */
		xQueueSendToBack( xIdResponseQueue, &sensor_data, portMAX_DELAY );
	}

}




/* Task to handle the ID requests from UART port */
void vTaskUartReqManager(void * pvParameters)
{
	uint8_t 			rx_uart_array[4];
	Sensor_Sample_Id	id_req = 0;

	for( ;; )
	{
		/* Wait the semaphore from the UART rx ISR */
		xSemaphoreTake( xUartRequestSemaphore, portMAX_DELAY );

		/* Read UART rx buffer and retrieve the ID (uint32_t format) */
		MCU_UART_Read_Array(&uart, rx_uart_array, 4);
		id_req = (rx_uart_array[3]<<24) | (rx_uart_array[2]<<16) | (rx_uart_array[1]<<8) | rx_uart_array[0];
		// TODO: Validate ID number !!!

		/* Send ID request to queue */
		xQueueSendToBack( xIdRequestQueue, &id_req, portMAX_DELAY );
	}

}



/* Task to send ID+Value responses through the UART port */
void vTaskUartRespManager(void * pvParameters)
{
	SensorData_t sensor_data;
	uint8_t buff_out[20];	// 4 bytes of ID + 4 floats with 4 bytes each

	for( ;; )
	{
		/* Read responses data queue */
		xQueueReceive( xIdResponseQueue, &sensor_data, portMAX_DELAY );

		/* Try to take control of the UART Tx channel */
		xSemaphoreTake( xUartTxMutex, portMAX_DELAY );
		{
			/* Read the last sensor data that was written in flash memory */
			memcpy((void*)buff_out, (void*)&sensor_data, 20);
			MCU_UART_Send_Array(&uart, buff_out, 20);
		}
		xSemaphoreGive( xUartTxMutex );

	}

}



/* Task to handle IDs requests from Sw button GPIO */
void vTaskButtonReqManager( void * pvParameters )
{
	uint8_t buff_out[20];	// 4 bytes of ID + 4 floats with 4 bytes each

	for( ;; )
	{
		/* Wait the button ISR to give a semaphore */
		xSemaphoreTake( xButtonRequestBufferSemaphore, portMAX_DELAY );

		/* Try to take control of the UART Tx channel */
		xSemaphoreTake( xUartTxMutex, portMAX_DELAY );
		{
			/* Read the last sensor data that was written in flash memory */
			taskENTER_CRITICAL();
			{
				memcpy((void*)buff_out, (void*)&last_Sensor_Data, 20);
				MCU_UART_Send_Array(&uart, buff_out, 20);
			}
			taskEXIT_CRITICAL();
		}
		xSemaphoreGive( xUartTxMutex );


	}
}




/* Function to init all OS objects */
bool System_OS_Init(void){

	bool init_err = false;

    // Semaphores
    xUartRequestSemaphore = xSemaphoreCreateBinary();
    if( xUartRequestSemaphore == NULL ) init_err = true;
    xButtonRequestBufferSemaphore = xSemaphoreCreateCounting(50, 0);
    if( xButtonRequestBufferSemaphore == NULL ) init_err = true;

    // Queues
    xSensorDataQueue = xQueueCreate( 10, sizeof( SensorData_t ) );
    if( xSensorDataQueue == NULL ) init_err = true;
    xIdRequestQueue = xQueueCreate( 100, sizeof( Sensor_Sample_Id ) );
    if( xIdRequestQueue == NULL ) init_err = true;
    xIdResponseQueue = xQueueCreate( 100, sizeof( SensorData_t ) );
    if( xIdResponseQueue == NULL ) init_err = true;

    // Mutexs
    xMemoryAccessMutex = xSemaphoreCreateMutex();
    if( xMemoryAccessMutex == NULL ) init_err = true;
    xUartTxMutex = xSemaphoreCreateMutex();
	if( xUartTxMutex == NULL ) init_err = true;

	// Tasks
    xTaskCreate( vTaskLedBlink, "LedBlinkRed", configMINIMAL_STACK_SIZE, &led_red  , 1, NULL );

    xTaskCreate( vTaskSensorAdqData, "AdquisitionSensorData", configMINIMAL_STACK_SIZE, NULL , 15, NULL );
    xTaskCreate( vTaskMemoryWriter, "MemotyWrite", configMINIMAL_STACK_SIZE, NULL , 5, NULL );
    xTaskCreate( vTaskMemoryReader, "MemotyWrite", configMINIMAL_STACK_SIZE, NULL , 10, NULL );

    xTaskCreate( vTaskUartReqManager, "ReqUARTManager", configMINIMAL_STACK_SIZE, NULL , 20, NULL );
    xTaskCreate( vTaskUartRespManager, "RespUARTManager", configMINIMAL_STACK_SIZE, NULL , 20, NULL );
    xTaskCreate( vTaskButtonReqManager, "ReqButton", configMINIMAL_STACK_SIZE, NULL , 25, NULL );

    return init_err;
}



