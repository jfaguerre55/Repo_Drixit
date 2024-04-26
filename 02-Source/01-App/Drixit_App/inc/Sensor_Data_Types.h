/*
 * Sensor_Data_Types.h
 *
 *  Created on: 25 abr. 2024
 *      Author: JuanAguerre
 */

#ifndef SENSOR_DATA_TYPES_H_
#define SENSOR_DATA_TYPES_H_


#define 	MAX_SENSOR_VARS		(4)				// Max number of variables a sensor can produce


/**
 * @brief Application sensor types/models
 */
typedef enum{	SENSOR_TYPE_LIS3MDL, 			/*!< Magnetics sensor */
				SENSOR_TYPE_LSM6DSV32X, 		/*!< IMU sensor */
				SENSOR_TYPE_BQ76XX,				/*!< BMS */
				SENSOR_TYPE_JY67A				/*!< Joystick */
} Sensor_Type_t;


/**
 * @brief Sensor error types
 */
typedef enum{	SENSOR_ERR_NONE, 				/*!< No error */
				SENSOR_ERR_A, 					/*!< ERROR A */
				SENSOR_ERR_B, 					/*!< ERROR B */
				SENSOR_ERR_C, 					/*!< ERROR C */
				SENSOR_ERR_D 					/*!< ERROR D */
} Sensor_Err_t;



/**
 * @brief Data produced by the any type of sensor
 */
typedef	struct{float data[MAX_SENSOR_VARS];} Sensor_Data_t;



/**
 * @brief Application sensor unique identification number (to support multiple sensor instances)
 */
typedef		uint8_t		Sensor_Number_t;


/**
 * @brief Application sensor model s
 */
typedef struct{
  Sensor_Type_t     	type;					/*!< Sensor type */
  Sensor_Number_t  		id;						/*!< Sensor id */
} Sensor_Id_t;



/**
 * @brief ID for sensor produced data (sample id)
 */
typedef 	uint32_t 	Sensor_Sample_Id_t;


/**
 * @brief Application sensor sample. This binds a sample with the sensor that produced it.
 */
typedef struct{
	Sensor_Sample_Id_t			id;				/*!< Unique id asigned to the sample */
	Sensor_Id_t					sensor;			/*!< Sensor that produced this sample */
	Sensor_Data_t				data;			/*!< Values for data. Array of MAX_SENSOR_VARS floats */
	uint32_t					time;			/*!< Timestamp for the sample (RTC ??) */
	Sensor_Err_t				err;			/*!< Error code */
	uint16_t					crc;			/*!< CRC error detection */
} Sensor_Sample_t;





#endif /* SENSOR_DATA_TYPES_H_ */
