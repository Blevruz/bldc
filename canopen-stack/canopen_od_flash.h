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
#define OD_ENTRY_SIZE	sizeof(CO_OBJ)	//unit: byte

#define OD_NVM_START	0x08080000	//start of section 8
#define OD_NVM_END	0x0809FFFF	//end of section 8
#define OD_MAX_NVM_SPACE	(OD_NVM_END - OD_NVM_START)	//128kB atow
#define OD_NVM_MAX_NB	(OD_MAX_NVM_SPACE / (OD_MAX_SIZE * OD_ENTRY_SIZE))
#define OD_NVM_START_WRITING	(OD_NVM_START + OD_NVM_MAX_NB + 4)

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
 * @brief	Erases buffer content
 *
 * @param obj_buff
 * pointer to temp buffer
 *
 * @return >0 if success, <0 if failure
 */
int ODf_ClearBuffer	(OD_BUFF* obj_buff);

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
 * @brief	Adds a formatted CO_OBJ to the OD buffer
 *
 * @param key
 * Word containing the index, the subindex and the tags for the entry.
 * Use the CO_KEY macro to generate it
 *
 * @param type
 * Custom CO_T_ type, otherwise 0
 *
 * @param data
 * Either constant with size up to 32 bit, or a pointer to the variable
 * (be sure to tag accordingly)
 *
 * @param obj_buff
 * The buffer to which the data is written.
 * The use of a buffer means that a call to ODf_BufferToNvm is necessary
 * after the buffer is full enough.
 *
 * @return >0 if success, <0 if failure
 */
int ODf_AddUpdate	(uint32_t key, const CO_OBJ_TYPE *type, CO_DATA data, OD_BUFF* obj_buff);

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
