

/******************************************************************************
* INCLUDES
******************************************************************************/

#include "canopen_od_flash.h"

/******************************************************************************
* PRIVATE FUNCTIONS
******************************************************************************/

/*
 * Q: 	What format are we expecting?
 * A: 	For a memory region capable of holding N object dictionaries, we expect
 * 	N bytes allocated to store the sizes of these ODs, 4 bytes to mark the
 * 	end of that section, then any number of past ODs of known length, then
 * 	the same 4 byte dword marking the ends of each, and finally our current
 * 	active OD.
 * 	(reasoning: we want to handle updates to the OD without too much wear
 * 	from repeated erase cycles, so we append new ODs. to keep track of 
 * 	which OD we are at, we write down the length of past ODs in the first
 * 	bytes)
 */

int ODf_CheckFormatting() {
	uint32_t read = 0;

	//TODO: when adapting for VESC_PKG, switch `flash_helper_read_nvm`
	//	for `VESC_IF->read_nvm`
	
	// Step 1: look for first END_DW at the end of size list
	
	if (!flash_helper_read_nvm((uint8_t*)&read, 4, OD_NVM_MAX_NB)) {
		return -1;	// formatting unknown: read error
	}
	if (read != OD_NVM_END_DW) {
		return -2;	// not formatted: lacking END_DW in size list
	}

	// Step 2: check given sizes
	
	bool done = false;
	int nb_od = 0;
	for (int i = 0; i < OD_NVM_MAX_NB; i++) {
		read = 0;
		if (!flash_helper_read_nvm((uint8_t*)&read, 1, i)) {
			return -1;	// formatting unknown: read error
		}
		if (done) {
			if (read == 0xFF) {
				continue;
			}
			return -3;	// size entry after the end of the list
		}
		if (read == 0xFF) {
			done = true;	// reached the end of the size list:
			nb_od = i;	// any non-0xFF entries after this
			continue;	// are an error
		}
		if (read <= OD_MAX_SIZE) {
			int size = read * OD_ENTRY_SIZE;
			// look for the end of the associated OD
			if (!flash_helper_read_nvm((uint8_t*)&read,
				4, 
				OD_NVM_MAX_NB + 4*(i+1) + size)){
				return -1;	// read error
			}
			if (read != OD_NVM_END_DW) {
				done = true;
				continue;
				//return -4;	// not formatted: lacking END_DW
			}
		}
	}
	return nb_od;	//we are in the clear! return the number of ODs
}

/******************************************************************************
* PUBLIC FUNCTIONS
******************************************************************************/

/*
* see function definition
*/

int ODf_EntryToBuffer	(CO_OBJ* to_write, OD_BUFF* obj_buff) {
	for (volatile uint8_t i = 0; i < obj_buff->used; i++) {	//volatile keyword to keep this loop in spite of optimisation
		if ((obj_buff->buffer[i].Key & ~0xFF) == (to_write->Key & ~0xFF)) {	// If other object with same Index&Subindex:
			obj_buff->buffer[i] = *to_write;			// Replace it with more recent obj
			return 3;
		}
	}
	if (obj_buff->used < OD_MAX_SIZE) {
		obj_buff->buffer[obj_buff->used++] = *to_write;
		return 2;
	}
	return -1;	//error: no more room
}

/*
* see function definition
*/

int ODf_BufferToNvm	(CO_IF_DRV* driver, OD_BUFF* obj_buff) {
	// CHECKING
	// Can we append?
	int 	index = 0;
	int	offset = 0;
	if (ODf_GetActiveOD(&index, &offset) < 0) {
		return -1;
	}

	if (index > OD_NVM_MAX_NB) {	// no room: need to wipe
		if (ODf_EraseNvm() < 0) {
			return -1;
		}
		index = 0;
		offset = OD_NVM_MAX_NB + 4;
	}

	// WRITING
	// step 1: write size to size list
	if (!flash_helper_write_nvm((uint8_t*)&(obj_buff->used), 1, index)) {
		return -2;
	}
	// step 2: write entries to NVM OD 
	for (int i = 0; i < obj_buff->used; i++) {
		if (!flash_helper_write_nvm((uint8_t*)&(obj_buff->buffer[i]),
				12,
				offset + i*OD_ENTRY_SIZE)) {
			return -3;
		}
	}
	return 1;
}

/*
* see function definition
*/

int ODf_NvmToBuffer	(CO_IF_DRV* driver, OD_BUFF* obj_buff) {
	int index = 0;
	int offset = 0;
	ODf_GetActiveOD(&index, &offset);
	uint8_t size = 0;
	flash_helper_read_nvm((uint8_t*)&size, 1, index);
	return flash_helper_read_nvm((uint8_t*)obj_buff->buffer, size*OD_ENTRY_SIZE, offset);
}

/*
* see function definition
*/

int ODf_EraseNvm	(void) {
	return flash_helper_wipe_nvm() ? 1 : -1;
}

/*
* see function definition
*/

int ODf_GetActiveOD	(int* index, int* offset) {
	uint8_t read = 0;
	int t_index = 0;
	int t_offset = OD_NVM_MAX_NB + 4;
	for (int i = 0; i < OD_NVM_MAX_NB; i++) {
		t_offset += read*12 + 4;
		t_index = i;
		if (!flash_helper_read_nvm((uint8_t*)&read, i, 1)) {
			return -1;
		}
		if (read == 0xFF) {
			break;
		}
	}
	*offset = t_offset;
	*index = t_index;
	return 1;
}
