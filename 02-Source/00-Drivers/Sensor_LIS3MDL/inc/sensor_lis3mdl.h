/*
 * sensor_lis3mdl.h
 *
 *  Created on: 14 abr. 2024
 *      Author: JuanAguerre
 *
 *  Diver for ST magnetic sensor LIS3MDL with I2C hardware interface
 *  Digital output magnetic sensor: ultralow-power, high-performance 3-axis magnetometer
 *
 *  WEB: https://www.st.com/en/mems-and-sensors/lis3mdl.html
 *
 */

#ifndef SENSOR_LIS3MDL_H_
#define SENSOR_LIS3MDL_H_


#include "GPIO_lib.h"
#include "timer_manager.h"

#ifdef  LIB_MCU_43xx_m4
	#include <MCU43xx_m4_I2C.h>
	#include "MCU43xx_m4_definiciones.h"
#endif
#ifdef LIB_MCU_43xx_m0
	#include "MCU43xx_m0_I2C.h"
	#include "MCU43xx_m0_definiciones.h"
#endif
#ifdef LIB_MCU_LPC84X_m0
	#include "MCU_LPC84x_I2C.h"
	#include "MCU_LPC84x_definiciones.h"
#endif

// Control registres address
#define 		ADDRESS_CTRL_REG1 		(0x20)
#define 		ADDRESS_CTRL_REG2 		(0x21)
#define 		ADDRESS_CTRL_REG3 		(0x22)
#define 		ADDRESS_CTRL_REG4 		(0x23)
#define 		ADDRESS_CTRL_REG5 		(0x24)
// Status registres address
#define 		ADDRESS_STATUS_REG 		(0x27)
// Output registres address
#define 		ADDRESS_OUT_X_L 		(0x28)
#define 		ADDRESS_OUT_X_H 		(0x29)
#define 		ADDRESS_OUT_Y_L 		(0x2A)
#define 		ADDRESS_OUT_Y_H 		(0x2B)
#define 		ADDRESS_OUT_Z_L 		(0x2C)
#define 		ADDRESS_OUT_Z_H 		(0x2D)
#define 		ADDRESS_OUT_T_L 		(0x2E)
#define 		ADDRESS_OUT_T_H 		(0x2F)
// Int registres address
#define 		ADDRESS_INT_CFG			(0x30)
#define 		ADDRESS_INT_SRC			(0x31)
#define 		ADDRESS_INT_THS_L		(0x32)
#define 		ADDRESS_INT_THS_H		(0x33)


#define		LIS3MDL_TRANSFER_BUFF_SIZE			(8)
#define		LIS3MDL_DRDY_SGN_ISR_PRIORITY		(5)
#define		LIS3MDL_RETRY_SEND_TIME_useg 		(3000)
#define		LIS3MDL_MAX_RETRY_ATTEMPTS			(10)

#define		LIS3MDL_RESET_COMMAND			(0x04)


/**
 * @brief Sensor error data types
 */
typedef enum{LIS3MDL_OK, LIS3MDL_ERROR}	Sensor_LIS3MDL_Status_t;


/**
 * @brief Sensor Full Scale data types [Gauss]
 */
typedef enum{LIS3MDL_FS_4GS, LIS3MDL_FS_8GS, LIS3MDL_FS_12GS, LIS3MDL_FS_16GS}	Sensor_LIS3MDL_FS_t;


/**
 * @brief Data type for listing the axis and temperature
 */
typedef		enum{LIS3MDL_AXIS_X, LIS3MDL_AXIS_Y, LIS3MDL_AXIS_Z, LIS3MDL_AXIS_T}	Sensor_LIS3MDL_Coord_t;



/**
 * @brief States for state machine controller of the LIS3MDL sensor
 */
typedef enum{	LIS3MDL_ST_FREE,
				LIS3MDL_ST_READING,
				LIS3MDL_ST_RESETING,
				LIS3MDL_ST_TRANSFER_ERROR
} Sensor_LIS3MDL_State_t;


/**
 * @brief Events for state machine controller of the LIS3MDL sensor
 */
typedef enum{	LIS3MDL_EV_NONE,
				LIS3MDL_EV_REQ_READ,
				LIS3MDL_EV_REQ_RST,
				LIS3MDL_EV_RESET_SYS
} Sensor_LIS3MDL_Event_t;


/**
 * @brief Sensor reading data type. Readings are in 16bits and 2's complement format
 */
typedef struct{

	int16_t		x;			/*!< X axis magnetic field */
	int16_t		y;			/*!< Y axis magnetic field */
	int16_t		z;			/*!< Z axis magnetic field */
	int16_t 	temp;		/*!< Temperature */

} Sensor_LIS3MDL_Value_t;




/**
 * @brief Sensor LIS3MDL data type
 */
typedef struct{

	// Hardware Setup
	MCU_I2C_t						i2c_controller;			/*!< I2C controller */
	uint8_t							i2c_address;			/*!< I2C address */
	GPIO_t		  					drdy;					/*!< Data ready pin */

	// Process values
	Sensor_LIS3MDL_Value_t			sensor_values;			/*!< Last sensor readings XYZ+temp*/


	// State machine
	uint8_t							timer_id;				/*!< id del timer para reenvio de tranferencias */
	uint8_t							retry_counter;			/*!< Contador de intento de reenvios de tranferencias */
	Sensor_LIS3MDL_State_t			state;					/*!< Estado actual del JS */
	Sensor_LIS3MDL_Event_t			event;					/*!< Último evento recibido */
	bool							pending_read;			/*!< flag para indicar que hay una lectura pendiente */
	// Transfer buffers
	uint8_t 						RxBuff[LIS3MDL_TRANSFER_BUFF_SIZE];
	uint8_t 						TxBuff[LIS3MDL_TRANSFER_BUFF_SIZE];


} Sensor_LIS3MDL_t;




/**
 * @brief Sensor LIS3MDL initialization data type
 *
 */
typedef struct{

	num_puerto 			drdy_port;				    /*!< Port DATA READY pin */
	num_bit             drdy_bit;					/*!< Bit DATA READY pin*/

	uint8_t				i2c_perif_num;				/*!< I2C perif number I2C */
	uint8_t				i2c_priority;				/*!< I2C ISRs Priority */
	uint32_t			i2c_busrate;				/*!< I2C BPS */
	uint8_t				i2c_address;				/*!< I2C address */
	uint8_t    			i2c_sda_port;				/*!< Port SDA */
	uint8_t       		i2c_sda_bit;				/*!< Bit SDA */
	uint8_t    			i2c_scl_port;				/*!< Port SCL */
	uint8_t       		i2c_scl_bit;				/*!< Bit SCL */

} Sensor_LIS3MDL_Config_Init_t;





/*********************************************************/
/***************** Public prototypes *********************/
/*********************************************************/
Sensor_LIS3MDL_Status_t LIS3MDL_Init(Sensor_LIS3MDL_t * sensorLIS3MDL,Sensor_LIS3MDL_Config_Init_t * sensorLIS3MDL_config);
Sensor_LIS3MDL_Status_t LIS3MDL_Get_XYZT(Sensor_LIS3MDL_t * sensorLIS3MDL, int16_t (*values)[4]);
Sensor_LIS3MDL_Status_t LIS3MDL_Get_Axis_Value(Sensor_LIS3MDL_t * sensorLIS3MDL, Sensor_LIS3MDL_Coord_t coord, int16_t * value);
Sensor_LIS3MDL_Status_t LIS3MDL_Reset_HW(Sensor_LIS3MDL_t * sensorLIS3MDL);
Sensor_LIS3MDL_Status_t LIS3MDL_Reset_SM(Sensor_LIS3MDL_t * sensorLIS3MDL);

/*********************************************************/
/***************** Private prototypes ********************/
/*********************************************************/
void *  __LIS3MDL_Update_State_Machine(void * p_sensor);
void * 	__LIS3MDL_DRDY_Callback(void* p_sensor);
void  	__LIS3MDL_Retry_Transfer(Sensor_LIS3MDL_t * sensorLIS3MDL);


#endif /* SENSOR_LIS3MDL_H_ */
