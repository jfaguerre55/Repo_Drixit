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




/**
 * @brief Sensor error data types
 */
typedef enum{SENSOR_LIS3MDL_OK, SENSOR_LIS3MDL_ERROR}	Sensor_LIS3MDL_Status_t;



/**
 * @brief Sensor Full Scale data types [Gauss]
 */
typedef enum{LIS3MDL_FS_4GS, LIS3MDL_FS_8GS, LIS3MDL_FS_12GS, LIS3MDL_FS_16GS}	Sensor_LIS3MDL_FS_t;


/**
 * @brief Data type for listing the axis and temperature
 */
typedef		enum{LIS3MDL_AXIS_X, LIS3MDL_AXIS_Y, LIS3MDL_AXIS_Z, LIS3MDL_AXIS_T}	Sensor_LIS3MDL_Coord_t;



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

	// I2C Setup
	MCU_I2C_t						i2c_controller;			/*!< I2C controller */
	uint8_t							i2c_address;			/*!< I2C address */
	GPIO_t		  					intn;					/*!< */

	Sensor_LIS3MDL_Value_t			sensor_values;			/*!< Last sensor readings XYZ+temp*/

} Sensor_LIS3MDL_t;




/**
 * @brief Sensor LIS3MDL initialization data type
 *
 */
typedef struct{

	num_puerto 			intn_port;				    /*!< Port INTn*/
	num_bit             intn_bit;					/*!< Bit INTn*/

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
Sensor_LIS3MDL_Status_t LIS3MDL_Get_Coord(Sensor_LIS3MDL_t * sensorLIS3MDL, Sensor_LIS3MDL_Coord_t coord, int16_t * value);
Sensor_LIS3MDL_Status_t LIS3MDL_Reset(Sensor_LIS3MDL_t * sensorLIS3MDL);

/*********************************************************/
/***************** Private prototypes ********************/
/*********************************************************/
void *  __LIS3MDL_Update_State_Machine(void * p_sensor);



#endif /* SENSOR_LIS3MDL_H_ */
