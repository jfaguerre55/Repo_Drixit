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
 * Dev address: SDO/SA1 pin is connected to ground -> 0001 1100 b  = 0x1C
 * Dev address: SDO/SA1 pin is connected to VCC    -> 0001 1110 b  = 0x1E

 */

#include "sensor_lis3mdl.h"

/**
 * @brief	Sensor LIS3MDL initialization function
 * @param	sensorLIS3MDL			: pointer to sensor structure
 * @param	sensorLIS3MDL_config	: pointer to configuration structure
 * @return	Sensor_LIS3MDL_Status_t
 */
Sensor_LIS3MDL_Status_t LIS3MDL_Init(Sensor_LIS3MDL_t * sensorLIS3MDL, Sensor_LIS3MDL_Config_Init_t * sensorLIS3MDL_config){

	GPIO_Error_t gpio_err = GPIO_ERROR_t;
	MCU_Status_t i2c_err = MCU_ERROR_t;

	if(sensorLIS3MDL == NULL || sensorLIS3MDL_config==NULL) return LIS3MDL_ERROR;

	// Init internal struct data
	sensorLIS3MDL->lsb_div = sensorLIS3MDL_config->lsb_div;
	sensorLIS3MDL->sensor_values.x = 0;
	sensorLIS3MDL->sensor_values.y = 0;
	sensorLIS3MDL->sensor_values.z = 0;
	sensorLIS3MDL->sensor_values.temp = 0;
	sensorLIS3MDL->i2c_address = sensorLIS3MDL_config->i2c_address;
	sensorLIS3MDL->state = LIS3MDL_ST_FREE;
	sensorLIS3MDL->event = LIS3MDL_EV_NONE;
	sensorLIS3MDL->pending_read = false;
	sensorLIS3MDL->retry_counter = 0;

	// Data ready GPIO configuration
	GPIO_Mode_Config_t intn_mode={GPIO_IN, GPIO_MODE_IN_NONE, GPIO_MODE_OUT_NONE};
	gpio_err = GPIO_Conf(&(sensorLIS3MDL->drdy), sensorLIS3MDL_config->drdy_port, sensorLIS3MDL_config->drdy_bit, &intn_mode);
	if(GPIO_ERROR_t == gpio_err) return LIS3MDL_ERROR;
	gpio_err = GPIO_IRQ_Init(&(sensorLIS3MDL->drdy), GPIO_INT_FALLING, LIS3MDL_DRDY_SGN_ISR_PRIORITY, NULL, NULL, __LIS3MDL_DRDY_Callback,(void*) sensorLIS3MDL);
	if(GPIO_ERROR_t == gpio_err) return LIS3MDL_ERROR;
	GPIO_IRQ_Disable(&(sensorLIS3MDL->drdy));

	// I2C hardware configuration
	MCU_I2C_Init_t i2c_init = {
		.id = sensorLIS3MDL_config->i2c_perif_num,
		.priority = sensorLIS3MDL_config->i2c_priority,
		.speed = sensorLIS3MDL_config->i2c_busrate,
		.sda_port = sensorLIS3MDL_config->i2c_sda_port,
		.sda_bit = sensorLIS3MDL_config->i2c_sda_bit,
		.scl_port = sensorLIS3MDL_config->i2c_scl_port,
		.scl_bit = sensorLIS3MDL_config->i2c_scl_bit
	};
	i2c_err = MCU_I2C_Master_Init(&(sensorLIS3MDL->i2c_controller), &i2c_init);
	if(MCU_ERROR_t == i2c_err) return LIS3MDL_ERROR;

	// Timer manager configuration
	Timer_Manager_Init();
	Timer_Manager_Add_Event(&(sensorLIS3MDL->timer_id), LIS3MDL_RETRY_SEND_TIME_useg, __LIS3MDL_Update_State_Machine, (void*)sensorLIS3MDL);
	Timer_Manager_Disable_Event(sensorLIS3MDL->timer_id);


	// State machine entry point
	sensorLIS3MDL->state = LIS3MDL_ST_INIT;
	sensorLIS3MDL->event = LIS3MDL_EV_NONE;
	__LIS3MDL_Update_State_Machine((void *)sensorLIS3MDL);


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
	sensorLIS3MDL->TxBuff[1] = LIS3MDL_RESET_COMMAND;	// Data for SOFT RESET device

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
void * 	__LIS3MDL_DRDY_Callback(void* p_sensor){

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
		if(LIS3MDL_EV_REQ_READ == sensorLIS3MDL->event){
			sensorLIS3MDL->TxBuff[0] = ADDRESS_OUT_X_L;		// Register Address
			if(MCU_ERROR_t == MCU_I2C_Execute_Transfer(	&(sensorLIS3MDL->i2c_controller), sensorLIS3MDL->i2c_address, sensorLIS3MDL->TxBuff, 1, sensorLIS3MDL->RxBuff, 8, __LIS3MDL_Update_State_Machine, p_sensor)){
				// No se pudo inicar la escritura: no se cambia el estado ni el evento y se intenta la transferencia luego
				__LIS3MDL_Retry_Transfer(sensorLIS3MDL);
				break;
			}
			// Se pudo iniciar la transferencia: el nuevo estado es leyendo. Se limpia el evento
			sensorLIS3MDL->retry_counter = 0;
			sensorLIS3MDL->state = LIS3MDL_ST_READING;
			sensorLIS3MDL->event = LIS3MDL_EV_NONE;
			break;
		}
		if(LIS3MDL_EV_REQ_RST == sensorLIS3MDL->event){
			sensorLIS3MDL->TxBuff[0] = ADDRESS_CTRL_REG2;		// Register Address
			sensorLIS3MDL->TxBuff[1] = LIS3MDL_RESET_COMMAND;	// Data
			if(MCU_ERROR_t == MCU_I2C_Execute_Transfer(	&(sensorLIS3MDL->i2c_controller), sensorLIS3MDL->i2c_address, sensorLIS3MDL->TxBuff, 2, NULL, 0, __LIS3MDL_Update_State_Machine, p_sensor)){
				// No se pudo inicar la escritura: no se cambia el estado ni el evento y se intenta la transferencia luego
				__LIS3MDL_Retry_Transfer(sensorLIS3MDL);
				break;
			}
			// Se pudo iniciar la transferencia: el estado es reseteando y se limpia el evento
			sensorLIS3MDL->retry_counter = 0;
			sensorLIS3MDL->state = LIS3MDL_ST_RESETING;
			sensorLIS3MDL->event = LIS3MDL_EV_NONE;
			break;
		}
		break;

	case LIS3MDL_ST_READING:
		////////////////////////////////////////////////////////////////////////
		///////////////////////////// STATE: READING ///////////////////////////
		////////////////////////////////////////////////////////////////////////
		if(LIS3MDL_EV_NONE == sensorLIS3MDL->event){
			// Reading cycle complete. Assemble the readings from de buffer.
			sensorLIS3MDL->sensor_values.x = 	(float)(((uint16_t)sensorLIS3MDL->RxBuff[1] << 8) | sensorLIS3MDL->RxBuff[0])/sensorLIS3MDL->lsb_div;
			sensorLIS3MDL->sensor_values.y = 	(float)(((uint16_t)sensorLIS3MDL->RxBuff[3] << 8) | sensorLIS3MDL->RxBuff[2])/sensorLIS3MDL->lsb_div;
			sensorLIS3MDL->sensor_values.z = 	(float)(((uint16_t)sensorLIS3MDL->RxBuff[5] << 8) | sensorLIS3MDL->RxBuff[4])/sensorLIS3MDL->lsb_div;
			sensorLIS3MDL->sensor_values.temp = (float)(((uint16_t)sensorLIS3MDL->RxBuff[7] << 8) | sensorLIS3MDL->RxBuff[6])/sensorLIS3MDL->lsb_div;
			//TODO: aca se podria implementar in filtro de las últimas N mediciones
			if( sensorLIS3MDL->pending_read == false)
				// Si no hay una lectura pendiente el estado es "free"
				sensorLIS3MDL->state = LIS3MDL_ST_FREE;
			else{
				// Si hay una lectura pendiente se debe iniciar otro ciclo de lectura
				sensorLIS3MDL->pending_read = false;
				sensorLIS3MDL->TxBuff[0] = ADDRESS_OUT_X_L;		// Register Address
				if(MCU_ERROR_t == MCU_I2C_Execute_Transfer(	&(sensorLIS3MDL->i2c_controller), sensorLIS3MDL->i2c_address, sensorLIS3MDL->TxBuff, 1, sensorLIS3MDL->RxBuff, 8, __LIS3MDL_Update_State_Machine, p_sensor)){
					// No se pudo inicar la escritura: se inicia el timer, se cambia el estado a free y el evento a read
					sensorLIS3MDL->state = LIS3MDL_ST_FREE;
					sensorLIS3MDL->event = LIS3MDL_EV_REQ_READ;
					__LIS3MDL_Retry_Transfer(sensorLIS3MDL);
					break;
				}
				// Se pudo iniciar la transferencia: el estado es leyendo y se limpia el evento
				sensorLIS3MDL->retry_counter = 0;
				sensorLIS3MDL->state = LIS3MDL_ST_READING;
				sensorLIS3MDL->event = LIS3MDL_EV_NONE;
				break;
			}
			break;
		}
		if(LIS3MDL_EV_REQ_READ == sensorLIS3MDL->event){
			// Si se está leyendo y vuelve a llegar DRDY se activa un flag para que al finalizar el ciclo actual de lectura se inicie otro
			sensorLIS3MDL->pending_read = true;
			sensorLIS3MDL->event = LIS3MDL_EV_NONE;
			break;
		}
		if(LIS3MDL_EV_REQ_RST == sensorLIS3MDL->event){
			// Si se está leyendo el joystick y llega un request de Reset se ignora (el bus está ocupado)
			sensorLIS3MDL->event = LIS3MDL_EV_NONE;
			break;
		}
		break;

	case LIS3MDL_ST_RESETING:
		////////////////////////////////////////////////////////////////////////
		//////////////////////////// STATE: RESETING ///////////////////////////
		////////////////////////////////////////////////////////////////////////
		if(LIS3MDL_EV_REQ_READ == sensorLIS3MDL->event){
			sensorLIS3MDL->TxBuff[0] = ADDRESS_OUT_X_L;		// Register Address
			if(MCU_ERROR_t == MCU_I2C_Execute_Transfer(	&(sensorLIS3MDL->i2c_controller), sensorLIS3MDL->i2c_address, sensorLIS3MDL->TxBuff, 1, sensorLIS3MDL->RxBuff, 8, __LIS3MDL_Update_State_Machine, p_sensor)){
				// No se pudo inicar la escritura: no se cambia el estado ni el evento y se intenta la transferencia luego
				__LIS3MDL_Retry_Transfer(sensorLIS3MDL);
				break;
			}
			// Se pudo iniciar la transferencia: el estado es leyendo y se limpia el evento
			sensorLIS3MDL->retry_counter = 0;
			sensorLIS3MDL->state = LIS3MDL_ST_READING;
			sensorLIS3MDL->event = LIS3MDL_EV_NONE;
			break;
		}
		if(LIS3MDL_EV_REQ_RST == sensorLIS3MDL->event){
			sensorLIS3MDL->TxBuff[0] = ADDRESS_CTRL_REG2;		// Register Address
			sensorLIS3MDL->TxBuff[1] = LIS3MDL_RESET_COMMAND;	// Data
			if(MCU_ERROR_t == MCU_I2C_Execute_Transfer(	&(sensorLIS3MDL->i2c_controller), sensorLIS3MDL->i2c_address, sensorLIS3MDL->TxBuff, 2, NULL, 0, __LIS3MDL_Update_State_Machine, p_sensor)){
				// No se pudo inicar la escritura: no se cambia el estado ni el evento y se intenta la transferencia luego
				__LIS3MDL_Retry_Transfer(sensorLIS3MDL);
				break;
			}
			// Se pudo iniciar la transferencia: el estado es reseteando y se limpia el evento. Dentro de 300mseg deberia activarse INTn y va a ingresar un evento de read
			sensorLIS3MDL->retry_counter = 0;
			sensorLIS3MDL->state = LIS3MDL_ST_RESETING;
			sensorLIS3MDL->event = LIS3MDL_EV_NONE;
			break;
		}
		break;


	case LIS3MDL_ST_INIT:
		////////////////////////////////////////////////////////////////////////
		/////////////////////////////  STATE: INIT   ///////////////////////////
		////////////////////////////////////////////////////////////////////////
		sensorLIS3MDL->TxBuff[0] = ADDRESS_CTRL_REG1;		// CTRL_REG1 Register Address
		sensorLIS3MDL->TxBuff[1] = LIS3MDL_CTRL_REG1;		// CTRL_REG1 -> TEMP_EN=enable, OM=Low Power, DO=80Hz, ODR=off, ST=off
		sensorLIS3MDL->TxBuff[2] = LIS3MDL_CTRL_REG2;		// CTRL_REG2 -> FS=12gauss, REBOOT=RST=0
		sensorLIS3MDL->TxBuff[3] = LIS3MDL_CTRL_REG3;		// CTRL_REG3 -> LP=0, SIM=0, MD0=01 (continuos conversion)
		sensorLIS3MDL->TxBuff[4] = LIS3MDL_CTRL_REG4;		// CTRL_REG4 -> OMZ=Lox Power, BLE=0
		sensorLIS3MDL->TxBuff[5] = LIS3MDL_CTRL_REG5;		// CTRL_REG5 -> FR=disable, BDU=continuos
		if(MCU_ERROR_t == MCU_I2C_Execute_Transfer(	&(sensorLIS3MDL->i2c_controller), sensorLIS3MDL->i2c_address, sensorLIS3MDL->TxBuff, 6, NULL, 0, __LIS3MDL_Update_State_Machine, p_sensor)){
			// No se pudo inicar la escritura: no se cambia el estado ni el evento y se intenta la transferencia luego
			__LIS3MDL_Retry_Transfer(sensorLIS3MDL);
			break;
		}
		// Se pudo iniciar la transferencia: el estado es free y se limpia el evento
		GPIO_IRQ_Enable(&(sensorLIS3MDL->drdy));
		sensorLIS3MDL->retry_counter = 0;
		sensorLIS3MDL->state = LIS3MDL_ST_FREE;
		sensorLIS3MDL->event = LIS3MDL_EV_NONE;
		break;
		break;

	case LIS3MDL_ST_TRANSFER_ERROR:
		////////////////////////////////////////////////////////////////////////
		/////////////////////// STATE: TRANSFER ERROR //////////////////////////
		////////////////////////////////////////////////////////////////////////
		if(LIS3MDL_EV_RESET_SYS == sensorLIS3MDL->event){
			sensorLIS3MDL->state = LIS3MDL_ST_RESETING;
			sensorLIS3MDL->event = LIS3MDL_EV_REQ_RST;
			Timer_Manager_Restart_Event(sensorLIS3MDL->timer_id);
		}
		break;

	default: break;
	}

	return NULL;
}








