/*
 * flash_w25q80db.h
 *
 *  Created on: 15 abr. 2024
 *      Author: JuanAguerre
 */

#ifndef FLASH_W25Q80DB_H_
#define FLASH_W25Q80DB_H_


#include "MCU43xx_m4_SPIFI.h"


#define		FLASH_W25Q80_MEM_SIZE			(1*1024*1024)	// 1 Mbyte
#define		FLASH_W25Q80_BLOCK_SIZE			(64*1024)		// 64 kbyte
#define		FLASH_W25Q80_SECTOR_SIZE		(4*1024)		// 4 kbyte
#define		FLASH_W25Q80_PAGE_SIZE			(256)			// bytes




/**
 * @brief Memory error data types
 */
typedef enum{FLASH_W25Q80L_OK, FLASH_W25Q80_ERROR}	Flash_W25Q80DB_Status_t;



/**
 * @brief Flash memory W25Q80DB data type
 */
typedef struct{

	uint32_t		head;
	uint32_t		tail;


} Flash_W25Q80DB_t;



Flash_W25Q80DB_Status_t 	Flash_W25Q80DB_Init(Flash_W25Q80DB_t * mem);
Flash_W25Q80DB_Status_t 	Flash_W25Q80DB_Write_Array(Flash_W25Q80DB_t * mem, void * buff, size_t buff_size);
Flash_W25Q80DB_Status_t 	Flash_W25Q80DB_Read_Array(Flash_W25Q80DB_t * mem, uint32_t address, void * buff, size_t buff_size);


#endif /* FLASH_W25Q80DB_H_ */
