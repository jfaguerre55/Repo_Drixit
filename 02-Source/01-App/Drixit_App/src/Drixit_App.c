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
		6, 11, LED_RED,		// Port - Pin - Color
		LED_MODE_ONF,		// Modo (on/off o level)
		LED_TURN_ON_1,		// Se prende por 1
		100					// Level inicial
};


Sw_Button_t				swISP;
void * 					swISP_When_Pushed(void * pvParameters);
void *					swISP_When_Release(void * pvParameters);
Sw_Button_Init_t		swISP_config = {
		2 , 7,										// Port - Pin
		SW_BUTTON_NA, SW_R_PULLUP, DEBOUNCE_ON,		// NO/NA - PullUp/PullDown - Debounce ON/OFF
		swISP_When_Pushed, NULL,					// Callback cuando se presiona y sus argumentos
		swISP_When_Release, NULL					// Callback cuando se libera y sus argumentos
};

MCU_UART_t 				uart;
MCU_UART_Init_Config_t 	uart_config;
void * uart_rx_cb(void * args);
void * uart_tx_cb(void * args);


// FreeRTOS objects
// Task prototypes + Queues handlers + Mutex handlers
void vTaskLedBlink(void * pvLed );							// Test Blinking LED

// Sensor + memory management
void vTaskSensorAdquisitionData(void * pvParameters);		// Aquisition task
QueueHandle_t 		xSensorDataQueue;						// Sensor data buffer
void vTaskMemoryWriteManager(void * pvParameters);			// Memory write task
void vTaskMemoryReadManager(void * pvParameters);			// Memory read task
SemaphoreHandle_t	xMemoryAccessMutex;						// Memory mutex

// UART + GPIO management
void vTaskButtonHMI(void * pvParameters);
void vTaskUartRxManager(void * pvParameters);
void vTaskUartTxManager(void * pvParameters);
void vTaskUartRxManager(void * pvParameters);



QueueHandle_t 		xIdRequestQueue;
SemaphoreHandle_t	xButtonRequestBufferSemaphore;
#define 			BUTTON_REQUEST_MAX			(50)
SemaphoreHandle_t	xUartRequestSemaphore;
SemaphoreHandle_t	xUartTxMutex;



int main(void) {

    SystemCoreClockUpdate();

    LED_Config(&led_red,   	 &led_red_config);
    SW_BUTTON_Config(&swISP, &swISP_config );

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
    uart_config.tx_port 	= 2;
    uart_config.tx_pin 		= 0;
    uart_config.rx_port 	= 2;
    uart_config.rx_pin 		= 1;
    // RS485 support
    uart_config.rs485_add	= 0xFF;
    uart_config.dir_port 	= 255;
    uart_config.dir_pin 	= 255;
    MCU_UART_Config(&uart, &uart_config);
    MCU_UART_Config_Rx_Callback(&uart, uart_rx_cb, NULL, sizeof(Sensor_Sample_Id));
    MCU_UART_Enable_Rx_Interrupt(&uart);
    MCU_UART_Config_Tx_Callback(&uart, uart_tx_cb, NULL);
    MCU_UART_Enable_NVIC(&uart, 6);


    uint8_t			i = 0;
    SensorData_t	sensor_data;
    SensorData_t	sensor_data2;

	sensor_data.id = i;
	sensor_data.values.x = 		55;
	sensor_data.values.y = 		56;
	sensor_data.values.z = 		57;
	sensor_data.values.temp = 	24;

    printf( "Id = %d\n", i);

    // Configura los pines y el CTRL del SPIFI
    spifi_init();

    // Reset Memory Mode and enter Command Mode
    spifi_command_mode();

    // Escribe SensorData_t bytes en la flash
    for(i=0; i<11; i++){
    	//	spifi_4K_write(SPIFLASH_BASE_ADDRESS, 							(void *)&sensor_data, sizeof(SensorData_t) );
    	sensor_data.id = i;
    	sensor_data.values.x ++;
    	sensor_data.values.y ++;
    	sensor_data.values.z ++;
    	sensor_data.values.temp ++;
//		spifi_4K_write(SPIFLASH_BASE_ADDRESS+ i*sizeof(SensorData_t), (void *)&sensor_data, sizeof(SensorData_t) );
    }

	// Pone a la memoria en Memory Mode
	spifi_memory_mode();


	for(i=0; i<10; i++){
		sensor_data2=*(SensorData_t*)(SPIFLASH_BASE_ADDRESS+i*sizeof(SensorData_t));
		printf( "Id = %d X = %d Y = %d Z = %d  T = %d\n", sensor_data2.id, sensor_data2.values.x,
				sensor_data2.values.y, sensor_data2.values.z, sensor_data2.values.temp);
	}






    //LIS3MDL_Init(&sensorLIS3MDL, &sensorLIS3MDL_config);

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
//    xTaskCreate( vTaskUartRxManager, "CommManager", configMINIMAL_STACK_SIZE, NULL , 20, NULL );
//
    xTaskCreate( vTaskMemoryWriteManager, "MemotyWrite", configMINIMAL_STACK_SIZE, NULL , 5, NULL );
//    xTaskCreate( vTaskMemoryReadManager, "MemotyWrite", configMINIMAL_STACK_SIZE, NULL , 10, NULL );


//    vTaskStartScheduler();

    while(1) {

    }


    return 0 ;
}



void vTaskMemoryWriteManager(void * pvParameters)
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


void vTaskMemoryReadManager(void * pvParameters)
{

	BaseType_t 		xStatus;

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



void * swISP_When_Pushed(void * pvParameters){

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	xSemaphoreGiveFromISR( xButtonRequestBufferSemaphore, &xHigherPriorityTaskWoken );

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

	return NULL;
}


void * swISP_When_Release(void * pvParameters){


	return NULL;
}
