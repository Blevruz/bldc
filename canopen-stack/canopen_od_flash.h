#ifndef CANOPEN_OD_FLASH_H_
#define CANOPEN_OD_FLASH_H_

// Useful operations for running a dynamic OD in flash memory

/******************************************************************************
* INCLUDES
******************************************************************************/

#include "flash_helper.h"
#include "co_core.h"

#include <stdint.h>

/******************************************************************************
* PRIVATE VARIABLES
******************************************************************************/

#define OD_MAX_SIZE	255	//fairly arbitrary, watch for int size
#define OD_ENTRY_SIZE	12	//unit: byte

#define OD_NVM_START	0x08080000	//section 8
#define OD_NVM_END	0x0809FFFF	//beginning of section 9
#define OD_MAX_NVM_SPACE	OD_NVM_END - OD_NVM_START	//128kB atow
#define OD_NVM_MAX_NB	OD_MAX_NVM_SPACE / (OD_MAX_SIZE * OD_ENTRY_SIZE)
#define OD_NVM_START_WRITING	OD_NVM_START + OD_NVM_MAX_NB + 4

#define OD_NVM_END_DW	0xCAFECAFE	//double word marking the end of an OD

/******************************************************************************
* PUBLIC TYPES
******************************************************************************/

typedef struct {
	CO_OBJ	buffer[OD_MAX_SIZE];
	uint8_t	used;
} OD_BUFF;

/******************************************************************************
* PRIVATE FUNCTIONS
******************************************************************************/

int ODf_CheckFormatting(void);

/******************************************************************************
* PUBLIC FUNCTIONS
******************************************************************************/


/**
 * @brief	Writes pointed object to buffer for later OD insertion
 *
 * @param to_write
 * pointer to an object to write in OD
 *
 * @param obj_buff
 * pointer to temp buffer
 *
 * @return >0 if success, <0 if failure
 */
int ODf_EntryToBuffer	(CO_OBJ* to_write, OD_BUFF* obj_buff);

/**
 * @brief	Writes data stored in buffer to NVM OD
 *
 * @param driver
 * pointer to canopen drivers
 *
 * @param obj_buff
 * pointer to temp buffer
 *
 * @return >0 if success, <0 if failure
 */
int ODf_BufferToNvm	(CO_IF_DRV* driver, OD_BUFF* obj_buff);

/**
 * @brief	Writes data stored in NVM to buffer
 *
 * @param driver
 * pointer to canopen drivers
 *
 * @param obj_buff
 * pointer to temp buffer
 *
 * @return >0 if success, <0 if failure
 */
int ODf_NvmToBuffer	(CO_IF_DRV* driver, OD_BUFF* obj_buff);

/**
 * @brief	Erases the NVM region dedicated to ODs
 *
 * @return >0 if success, <0 if failure
 */
int ODf_EraseNvm	(void);

/**
 * @brief	Provides the index of the current active OD
 *
 * @param index
 * pointer to an int to which function writes the active OD's size index
 *
 * @param offset
 * pointer to an int to which function writes the active OD's byte offset
 *
 * @return >0 if success, <0 if failure
 */
int ODf_GetActiveOD	(int* index, int* offset);

#endif
