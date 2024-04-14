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

//	i2c(ADDRESS_CTRL_REG1, 0b1001 1100);	// TEMP_EN=enable, OM=Low Power, DO=80Hz, ODR=off, ST=off
//	i2c(ADDRESS_CTRL_REG2, 0b0100 0000);	// FS=12gauss, REBOOT=RST=0
//	i2c(ADDRESS_CTRL_REG3, 0x0000 0000);	// LP=0, SIM=0, MD0=01 (continuos conversion)
//	i2c(ADDRESS_CTRL_REG4, 0x00);	// OMZ=Lox Power, BLE=0
//	i2c(ADDRESS_CTRL_REG5, 0x00);	// FR=disable, BDU=continuos
//
//	i2c(ADDRESS_INT_CFG, 0x00);		// INT disable s


	return LIS3MDL_OK;

}


/**
 * @brief	Sensor LIS3MDL get last XYZ and temperature readings
 * @param	sensorLIS3MDL		: pointer to sensor structure
 * @param	values				: pointer to array to return 4 readings
 * @return	Sensor_LIS3MDL_Status_t
 */
Sensor_LIS3MDL_Status_t LIS3MDL_Get_XYZT(Sensor_LIS3MDL_t * sensorLIS3MDL, int16_t (*values)[4]){

	if(sensorLIS3MDL == NULL) return LIS3MDL_ERROR;

	(*values)[LIS3MDL_AXIS_X] = sensorLIS3MDL->sensor_values.x;
	(*values)[LIS3MDL_AXIS_Y] = sensorLIS3MDL->sensor_values.y;
	(*values)[LIS3MDL_AXIS_Z] = sensorLIS3MDL->sensor_values.z;
	(*values)[LIS3MDL_AXIS_T] = sensorLIS3MDL->sensor_values.temp;

	return LIS3MDL_OK;

}

/**
 * @brief	Sensor LIS3MDL get one axis reading
 * @param	sensorLIS3MDL			: pointer to sensor structure
 * @param	Sensor_LIS3MDL_Coord_t	: coordinate
 * @param	value					: pointer to 16 bit int to return the value
 * @return	Sensor_LIS3MDL_Status_t
 */
Sensor_LIS3MDL_Status_t LIS3MDL_Get_Axis_Value(Sensor_LIS3MDL_t * sensorLIS3MDL, Sensor_LIS3MDL_Coord_t coord, int16_t * value){

	if(sensorLIS3MDL == NULL) return LIS3MDL_ERROR;

	switch(coord){
		case LIS3MDL_AXIS_X:
			*value = sensorLIS3MDL->sensor_values.x;
			break;
		case LIS3MDL_AXIS_Y:
			*value = sensorLIS3MDL->sensor_values.x;
			break;
		case LIS3MDL_AXIS_Z:
			*value = sensorLIS3MDL->sensor_values.z;
			break;
		case LIS3MDL_AXIS_T:
			*value = sensorLIS3MDL->sensor_values.temp;
		default: break;
	}

	return LIS3MDL_OK;

}


/**
 * @brief	Function to reset the sensor HW
 * @param	sensorLIS3MDL				: pointer to sensor structure
 * @return	Sensor_LIS3MDL_Status_t
 */
Sensor_LIS3MDL_Status_t LIS3MDL_Reset_HW(Sensor_LIS3MDL_t * sensorLIS3MDL){

	if(sensorLIS3MDL == NULL) return LIS3MDL_ERROR;

	sensorLIS3MDL->TxBuff[0] = ADDRESS_CTRL_REG2;		// Register Address
	sensorLIS3MDL->TxBuff[1] = 0x04;					// Data for SOFT RESET device

	sensorLIS3MDL->event = LIS3MDL_EV_REQ_RST;
	__LIS3MDL_Update_State_Machine((void*)sensorLIS3MDL);

	return LIS3MDL_OK;
}


/**
 * @brief	Function to reset the state machine in case of an error
 * @param	sensorLIS3MDL				: pointer to sensor structure
 * @return	Sensor_LIS3MDL_Status_t
 */
Sensor_LIS3MDL_Status_t LIS3MDL_Reset_SM(Sensor_LIS3MDL_t * sensorLIS3MDL){

	if(sensorLIS3MDL == NULL) return LIS3MDL_ERROR;

	sensorLIS3MDL->event = LIS3MDL_EV_RESET_SYS;
	__LIS3MDL_Update_State_Machine((void*)sensorLIS3MDL);

	return LIS3MDL_OK;
}





/**PRIVATE
 * @brief	Callback of GPIO data ready pin (ISR)
 * @param	p_sensor		: pointer to sensor structure
 * @return	void*
 */
void * 	__LIS3MDL_INTn_Callback(void* p_sensor){

	Sensor_LIS3MDL_t * sensorLIS3MDL = (Sensor_LIS3MDL_t *)p_sensor;
	if(sensorLIS3MDL == NULL) return NULL;

	sensorLIS3MDL->event = LIS3MDL_EV_REQ_READ;
	__LIS3MDL_Update_State_Machine(p_sensor);

	return NULL;
}



/**PRIVATE
 * @brief	This function hamdles the Timer and the transfer counter attempts
 * @param	p_sensor		: pointer to sensor structure
 * @return	void
 */
void  	__LIS3MDL_Retry_Transfer(Sensor_LIS3MDL_t * sensorLIS3MDL){

	// Restart timer event
	Timer_Manager_Restart_Event(sensorLIS3MDL->timer_id);

	// Check the counter for max attempts limit
	sensorLIS3MDL->retry_counter++;
	if(sensorLIS3MDL->retry_counter == LIS3MDL_MAX_RETRY_ATTEMPTS ){
		sensorLIS3MDL->retry_counter = 0;
		sensorLIS3MDL->state = LIS3MDL_ST_TRANSFER_ERROR;
		sensorLIS3MDL->event = LIS3MDL_EV_NONE;
	}

}




/**PRIVATE
 * @brief	State machine to control the LIS3MDL sensor
 * @param	sensor			: pointer to sensor structure
 * @return	void*
 */
void *  __LIS3MDL_Update_State_Machine(void * p_sensor){

	Sensor_LIS3MDL_t * sensorLIS3MDL = (Sensor_LIS3MDL_t *)p_sensor;
	if(sensorLIS3MDL == NULL) return NULL;

	switch(sensorLIS3MDL->state){

	case LIS3MDL_ST_FREE:
		////////////////////////////////////////////////////////////////////////
		/////////////////////////////  STATE: FREE   ///////////////////////////
		////////////////////////////////////////////////////////////////////////
		break;

	case LIS3MDL_ST_READING:
		////////////////////////////////////////////////////////////////////////
		///////////////////////////// STATE: READING ///////////////////////////
		////////////////////////////////////////////////////////////////////////
		break;
	case LIS3MDL_ST_RESETING:
		////////////////////////////////////////////////////////////////////////
		//////////////////////////// STATE: RESETING ///////////////////////////
		////////////////////////////////////////////////////////////////////////
		break;
	case LIS3MDL_ST_TRANSFER_ERROR:
		////////////////////////////////////////////////////////////////////////
		/////////////////////// STATE: TRANSFER ERROR //////////////////////////
		////////////////////////////////////////////////////////////////////////
		break;

	default: break;
	}

	return NULL;
}








