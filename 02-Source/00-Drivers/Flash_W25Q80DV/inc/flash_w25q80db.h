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

#ifndef FLASH_W25Q80DB_H_
#define FLASH_W25Q80DB_H_


#include "MCU43xx_m4_SPIFI.h"


#define		FLASH_W25Q80_BASE_ADDRESS		MCU43XX_SPIFI_BASE_ADDRESS	// Hardware base address

#define		FLASH_W25Q80_MEM_SIZE			(1*1024*1024)	// 1 Mbytes
#define		FLASH_W25Q80_BLOCK_SIZE			(64*1024)		// 64 kbytes
#define		FLASH_W25Q80_SECTOR_SIZE		(4*1024)		// 4 kbytes
#define		FLASH_W25Q80_PAGE_SIZE			(256)			// 256 bytes

#define		FLASH_W25Q80_LOT_SIZE			(32)			// 32 bytes
#define		FLASH_W25Q80_LOT_TOT			(FLASH_W25Q80_SECTOR_SIZE/FLASH_W25Q80_LOT_SIZE)			// 32 bytes



/**
 * @brief Memory error data types
 */
typedef enum{FLASH_W25Q80_OK, FLASH_W25Q80_ERROR}	Flash_W25Q80DB_Status_t;



/**
 * @brief Flash memory W25Q80DB data type
 * @notes
 * 		0- El driver está diseñado para utilizar la memoria R/W en pequeños lotes de 32 bytes para guardar datos (ID+VALOR)
 * 		1- Las operaciones de RW se realizan por lotes de 32 bytes (en cada página de la memoria entran 8 lotes)
 * 		2- Las operaciones de Write deben estar alineadas en 32 bytes
 * 		3- La memoria tiene:
 * 			whole: 1Mb/32bytes = 32k lotes
 * 			sector:	4k/32bytes = 128 lotes
 * 		4- La memoria se direcciona por un index del lote (de 0 a 32k-1)
 * 		5- Si el index del lote que se quiere escribir no coincide el driver de la memoria realiza un erase del lote completo
 * 		6- La escritura de la memoria debe hacerse de forma progresiva. Desde el index  0 al 32k-1.
 * 		7- Si se intenta escribir en otro sector distinto al actual se va a borrar el contenido completo del sector
 */
typedef struct{

	uint32_t		lot_index;		/* Index of the next lot to be written */
	uint32_t		sector_index;	/* Current memory sector index */

} Flash_W25Q80DB_t;



typedef struct{

	uint8_t		lot[FLASH_W25Q80_LOT_SIZE];		/* Memory lot data type */

} Flash_W25Q80DB_Lot_t;



/*********************************************************/
/***************** Public prototypes *********************/
/*********************************************************/
Flash_W25Q80DB_Status_t 	Flash_W25Q80DB_Init(Flash_W25Q80DB_t * mem);
Flash_W25Q80DB_Status_t 	Flash_W25Q80DB_Write_Lot(Flash_W25Q80DB_t * mem, uint32_t address, Flash_W25Q80DB_Lot_t * lot );
Flash_W25Q80DB_Status_t 	Flash_W25Q80DB_Read_Lot(Flash_W25Q80DB_t * mem, uint32_t address, Flash_W25Q80DB_Lot_t * lot );

Flash_W25Q80DB_Status_t 	Flash_W25Q80DB_Get_Current_Index(Flash_W25Q80DB_t * mem, uint32_t * lot_index);
Flash_W25Q80DB_Status_t 	Flash_W25Q80DB_Get_Current_Sector(Flash_W25Q80DB_t * mem, uint32_t * sector_index);



#endif /* FLASH_W25Q80DB_H_ */
