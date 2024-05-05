/*
 * flash_w25q80db.h
 *
 *  Created on: 15 abr. 2024
 *      Author: JuanAguerre
 *
 * Driver for W25Q80DV Winbond serial memory with SPIFI interface
 *
 * WEB: https://www.winbond.com/hq/product/code-storage-flash-memory/serial-nor-flash/?__locale=en&partNo=W25Q80DV
 *
 */

#include "flash_w25q80db.h"


/**
 * @brief	Memory driver initialization function
 * @param	Flash_W25Q80DB_t			: pointer to memory structure
 * @return	Flash_W25Q80DB_Status_t
 */
Flash_W25Q80DB_Status_t 	Flash_W25Q80DB_Init(Flash_W25Q80DB_t * mem){

	if(mem == NULL) return FLASH_W25Q80_ERROR;

	mem->lot_index = 0ul;
	mem->sector_index = 0ul;
	mem->mode = FLASH_W25Q80_MODE_COMMAND;

	MCU_SPIFI_Init();
//	MCU_SPIFI_Reset();
	MCU_SPIFI_Enter_Command_Mode();
	MCU_SPIFI_Erase_Sector(FLASH_W25Q80_BASE_ADDRESS + FLASH_W25Q80_SECTOR_SIZE*mem->sector_index);

	return FLASH_W25Q80_OK;
}




/**
 * @brief	Memory driver function to write a lot
 * @param	mem:	pointer to memory structure
 * @param	index:	memory index to write the lot
 * @param	buff:	pointer to the lot to be written
 * @return	Flash_W25Q80DB_Status_t
 */
Flash_W25Q80DB_Status_t 	Flash_W25Q80DB_Write_Lot(Flash_W25Q80DB_t * mem, uint32_t index, Flash_W25Q80DB_Lot_t * lot){

	uint32_t	sector_new = 0ul;

	if(mem==NULL || index >= FLASH_W25Q80_LOT_PER_MEM || lot==NULL) return FLASH_W25Q80_ERROR;

	/* If index selects a new sector, this sector must be erased */
	sector_new = (FLASH_W25Q80_LOT_SIZE*index)/FLASH_W25Q80_SECTOR_SIZE;
	if(mem->sector_index != sector_new){
		mem->sector_index = sector_new;
		MCU_SPIFI_Enter_Command_Mode();
		mem->mode = FLASH_W25Q80_MODE_COMMAND;
		MCU_SPIFI_Erase_Sector(FLASH_W25Q80_BASE_ADDRESS + FLASH_W25Q80_SECTOR_SIZE * sector_new);
	}

	/* Write the lot */
	MCU_SPIFI_Enter_Command_Mode();
	mem->mode = FLASH_W25Q80_MODE_COMMAND;
	MCU_SPIFI_Write_Page(FLASH_W25Q80_BASE_ADDRESS + FLASH_W25Q80_LOT_SIZE*index , (void*)lot, FLASH_W25Q80_LOT_SIZE);
	mem->lot_index = index + 1;

	return FLASH_W25Q80_OK;
}



/**
 * @brief	Memory driver function to write an array of lots
 * @param	mem:	pointer to memory structure
 * @param	index:	memory index to write the lots
 * @param	lots:	pointer to array of lots
 * @param	size:	number of lots to write
 * @return	Flash_W25Q80DB_Status_t
 */
Flash_W25Q80DB_Status_t 	Flash_W25Q80DB_Write_Lot_Array(Flash_W25Q80DB_t * mem, uint32_t index, Flash_W25Q80DB_Lot_t * lots, size_t size){

	uint32_t	sector_new = 0ul;
	uint32_t	i = 0ul;

	if(mem == NULL || index >= FLASH_W25Q80_LOT_PER_MEM || lots==NULL || size==0) return FLASH_W25Q80_ERROR;

	/* All write operation must be done in command mode */
	if(mem->mode != FLASH_W25Q80_MODE_COMMAND){
		MCU_SPIFI_Enter_Command_Mode();
		mem->mode = FLASH_W25Q80_MODE_COMMAND;
	}

	/* Loop to erase all necesary sectors of the memory */
	for(i=index; i<index+size ; i=(i|0x7F)+1){ 		// i|0x7F => i redondeado para arriba hasta el multiplo mas cercano de 128
		sector_new = (FLASH_W25Q80_LOT_SIZE*i)/FLASH_W25Q80_SECTOR_SIZE;
		if(mem->sector_index != sector_new){
			mem->sector_index = sector_new;
			MCU_SPIFI_Erase_Sector(FLASH_W25Q80_BASE_ADDRESS + FLASH_W25Q80_SECTOR_SIZE * sector_new);
		}
	}

	/* Write the lots */
	MCU_SPIFI_Write_Segment(FLASH_W25Q80_BASE_ADDRESS + FLASH_W25Q80_LOT_SIZE*index , (void*)lots, FLASH_W25Q80_LOT_SIZE*size);
	mem->lot_index = index + size;

	return FLASH_W25Q80_OK;
}






// Previo a escribir, hay que borrar todos los sector involucrados menos el actual
// El controlador debe:
//		- Asumir que el sector que contiene la direccion de escritura está en uso (no hay que borrarlo)
//		- Si se va a pasar del sector actual, borrar los sectores siguientes que sean necesarios
//	uint32_t	SA = 0;			// Sector Address
//	uint32_t	LSA = 0;		// Local Sector Address
//	SA  = address_dest & (~0xFFF);		// When the lasts 12 bits are 0's it is the address of the sector
//	LSA = address_dest & 0xFFF;			// Last 12 bits indicate the address into the sector
//	FLASH_W25Q80_SECTOR_SIZE - LSA;		// Cuantos bytes entrarian en este sector
//	if ( BUFF_SIZE > (FLASH_W25Q80_SECTOR_SIZE - LSA) )	{	// Si lo que quiero escribir es mas que lo que entra en el sector actual
//		( BUFF_SIZE - (FLASH_W25Q80_SECTOR_SIZE - LSA)  + 0xFFF)  / FLASH_W25Q80_SECTOR_SIZE;	// Cuantos sectores en total se usarian
//
//		for(i=1; i<BUFF_SIZE; i++)
//			MCU_SPIFI_Erase_Sector(SA + i*FLASH_W25Q80_SECTOR_SIZE);
//	}



/**
 * @brief	Memory driver read lot function
 * @param	mem: 	pointer to memory structure
 * @param	index:	index of the lot to be accessed
 * @param	buff:	pointer to FLASH_W25Q80_LOT_SIZE array of bytes
 * @return	Flash_W25Q80DB_Status_t
 */
Flash_W25Q80DB_Status_t 	Flash_W25Q80DB_Read_Lot(Flash_W25Q80DB_t * mem, uint32_t index, Flash_W25Q80DB_Lot_t * lot){

	if(mem == NULL) return FLASH_W25Q80_ERROR;

	if(mem->mode != FLASH_W25Q80_MODE_MEMORY){
		mem->mode = FLASH_W25Q80_MODE_MEMORY;
		MCU_SPIFI_Enter_Memory_Mode();
	}

	*lot = *((Flash_W25Q80DB_Lot_t*)(FLASH_W25Q80_BASE_ADDRESS+FLASH_W25Q80_LOT_SIZE*index));

	return FLASH_W25Q80_OK;
}



/**
 * @brief	Memory driver lot index getter function (returns the next index to the last written).
 * @param	mem: 		pointer to memory structure
 * @param	lot_index:	pointer to variable to return the actual memory index (last index used in a W operation)
 * @note	This getter allows the user to get the next free index and write the memory in the progresive manner
 * @return	Flash_W25Q80DB_Status_t
 */
Flash_W25Q80DB_Status_t 	Flash_W25Q80DB_Get_Free_Index(Flash_W25Q80DB_t * mem, uint32_t * lot_index){

	if(mem == NULL) return FLASH_W25Q80_ERROR;

	*lot_index = mem->lot_index;

	return FLASH_W25Q80_OK;
}


/**
 * @brief	Memory driver Sector getter function
 * @param	mem: 			pointer to memory structure
 * @param	sector_index:	pointer to variable to return the actual memory sector (last sector used in a W operation)
 * @return	Flash_W25Q80DB_Status_t
 */
Flash_W25Q80DB_Status_t 	Flash_W25Q80DB_Get_Current_Sector(Flash_W25Q80DB_t * mem, uint32_t * sector_index){

	if(mem == NULL) return FLASH_W25Q80_ERROR;

	*sector_index = mem->sector_index;

	return FLASH_W25Q80_OK;
}
