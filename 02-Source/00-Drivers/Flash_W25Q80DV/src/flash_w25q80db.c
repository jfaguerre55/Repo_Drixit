/*
 * flash_w25q80db.c
 *
 *  Created on: 15 abr. 2024
 *      Author: JuanAguerre
 */

#include "flash_w25q80db.h"



Flash_W25Q80DB_Status_t 	Flash_W25Q80DB_Init(Flash_W25Q80DB_t * mem){

	if(mem == NULL) return FLASH_W25Q80_ERROR;

	mem->lot_index = 0ul;
	mem->sector_index = FLASH_W25Q80_LOT_TOT+1;	// Starts out of range to force erase the first sector to be written

	MCU_SPIFI_Init();
	MCU_SPIFI_Reset();

	return FLASH_W25Q80_OK;
}



Flash_W25Q80DB_Status_t 	Flash_W25Q80DB_Write_Lot(Flash_W25Q80DB_t * mem, uint32_t index, void * buff ){

	if(mem == NULL || index >= FLASH_W25Q80_LOT_TOT) return FLASH_W25Q80_ERROR;

	/* If an index selects a new sector, this sector must be erased */
	if(mem->sector_index != ((FLASH_W25Q80_LOT_SIZE*index)/FLASH_W25Q80_SECTOR_SIZE)){
		mem->sector_index = (FLASH_W25Q80_LOT_SIZE*index)/FLASH_W25Q80_SECTOR_SIZE;
		MCU_SPIFI_Erase_Sector(FLASH_W25Q80_BASE_ADDRESS + FLASH_W25Q80_SECTOR_SIZE*mem->sector_index);
	}

	mem->lot_index = index;

	MCU_SPIFI_Enter_Command_Mode();
	MCU_SPIFI_Write_Page(FLASH_W25Q80_BASE_ADDRESS + FLASH_W25Q80_LOT_SIZE*index , buff, FLASH_W25Q80_LOT_SIZE);

	return FLASH_W25Q80_OK;
}





Flash_W25Q80DB_Status_t 	Flash_W25Q80DB_Read_Lot(Flash_W25Q80DB_t * mem, uint32_t index, void * buff ){

	if(mem == NULL) return FLASH_W25Q80_ERROR;

	uint32_t i=0;
	MCU_SPIFI_Enter_Memory_Mode();

	for(i=0; i< FLASH_W25Q80_LOT_SIZE; i++){
		((uint8_t*)buff)[i] = ((uint8_t*)(FLASH_W25Q80_BASE_ADDRESS + FLASH_W25Q80_LOT_SIZE*index))[i];
	}

	return FLASH_W25Q80_OK;
}




Flash_W25Q80DB_Status_t 	Flash_W25Q80DB_Get_Current_Index(Flash_W25Q80DB_t * mem, uint32_t * lot_index){

	if(mem == NULL) return FLASH_W25Q80_ERROR;

	*lot_index = mem->lot_index;

	return FLASH_W25Q80_OK;
}



Flash_W25Q80DB_Status_t 	Flash_W25Q80DB_Get_Current_Sector(Flash_W25Q80DB_t * mem, uint32_t * sector_index){

	if(mem == NULL) return FLASH_W25Q80_ERROR;

	*sector_index = mem->sector_index;

	return FLASH_W25Q80_OK;
}
