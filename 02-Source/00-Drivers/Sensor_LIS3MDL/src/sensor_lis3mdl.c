/*
 * sensor_lis3mdl.c
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

#include "sensor_lis3mdl.h"


/**
 * @brief	Sensor LIS3MDL initialization function
 * @param	sensorLIS3MDL			: pointer to sensor structure
 * @param	sensorLIS3MDL_config	: pointer to configuration structure
 * @return	Sensor_LIS3MDL_Status_t
 */
Sensor_LIS3MDL_Status_t LIS3MDL_Init(Sensor_LIS3MDL_t * sensorLIS3MDL,Sensor_LIS3MDL_Config_Init_t * sensorLIS3MDL_config){

	i2c(ADDRESS_CTRL_REG1, 0b1001 1100);	// TEMP_EN=enable, OM=Low Power, DO=80Hz, ODR=off, ST=off
	i2c(ADDRESS_CTRL_REG2, 0b0100 0000);	// FS=12gauss, REBOOT=RST=0
	i2c(ADDRESS_CTRL_REG3, 0x0000 0001);	// LP=0, SIM=0, MD0=01 (single conversion)
	i2c(ADDRESS_CTRL_REG4, 0x00);	// OMZ=Lox Power, BLE=0
	i2c(ADDRESS_CTRL_REG5, 0x00);	// FR=disable, BDU=continuos

	i2c(ADDRESS_INT_CFG, 0x00);		// INT disable s


	return SENSOR_LIS3MDL_OK;

}


/**
 * @brief	Sensor LIS3MDL get last XYZ and temperature readings
 * @param	sensorLIS3MDL		: pointer to sensor structure
 * @param	values				: pointer to array to return 4 readings
 * @return	Sensor_LIS3MDL_Status_t
 */
Sensor_LIS3MDL_Status_t LIS3MDL_Get_XYZT(Sensor_LIS3MDL_t * sensorLIS3MDL, int16_t (*values)[4]){

	return SENSOR_LIS3MDL_OK;

}

/**
 * @brief	Sensor LIS3MDL get one axis reading
 * @param	sensorLIS3MDL			: pointer to sensor structure
 * @param	Sensor_LIS3MDL_Coord_t	: coordinate
 * @param	value					: pointer to 16 bit int to return the value
 * @return	Sensor_LIS3MDL_Status_t
 */
Sensor_LIS3MDL_Status_t LIS3MDL_Get_Coord(Sensor_LIS3MDL_t * sensorLIS3MDL, Sensor_LIS3MDL_Coord_t coord, int16_t * value){

	return SENSOR_LIS3MDL_OK;

}


Sensor_LIS3MDL_Status_t LIS3MDL_Reset(Sensor_LIS3MDL_t * sensorLIS3MDL){


	return SENSOR_LIS3MDL_OK;
}


void *  __LIS3MDL_Update_State_Machine(void * p_sensor);
