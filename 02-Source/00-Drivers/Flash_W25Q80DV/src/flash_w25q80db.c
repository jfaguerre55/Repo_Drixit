/*
 * flash_w25q80db.c
 *
 *  Created on: 15 abr. 2024
 *      Author: JuanAguerre
 */

#include "flash_w25q80db.h"



Flash_W25Q80DB_Status_t 	Flash_W25Q80DB_Init(Flash_W25Q80DB_t * mem){

	MCU_SPIFI_Init();
	MCU_SPIFI_Reset();


	return FLASH_W25Q80L_OK;
}



Flash_W25Q80DB_Status_t 	Flash_W25Q80DB_Write_Array(Flash_W25Q80DB_t * mem, uint32_t address, void * buff, size_t buff_size){

	MCU_SPIFI_Enter_Command_Mode();
	MCU_SPIFI_Write_Page(MCU43XX_SPIFI_BASE_ADDRESS+address, buff, buff_size);

	return FLASH_W25Q80L_OK;
}


Flash_W25Q80DB_Status_t 	Flash_W25Q80DB_Read_Array(Flash_W25Q80DB_t * mem, uint32_t address, void * buff, size_t buff_size){

	uint32_t i=0;
	MCU_SPIFI_Enter_Memory_Mode();

	for(i=0; i< buff_size; i++){
		((uint8_t*)buff)[i] = ((uint8_t*)(MCU43XX_SPIFI_BASE_ADDRESS*address))[i];
	}

	return FLASH_W25Q80L_OK;
}
