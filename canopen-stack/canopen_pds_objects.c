#include "canopen_pds_objects.h"


/*	CONTROL WORD: See CiA 402 part 2 section 8.4.1
 *		Structure:
 *	Bit 0: 		so	switch on
 *	Bit 1:		ev	enable voltage
 *	Bit 2:		qs	quick stop
 *	Bit 3:		eo	enable operation
 *	Bit 4 - 6:	oms	operation mode specific
 *	Bit 7:		fr	fault reset
 *	Bit 8:		h	halt
 *	Bit 9:		oms	operation mode specific
 *	Bit 10:		r	reserved
 *	Bit 11 - 16:	ms	manufacturer specific
 *	
 *	See Table 27 from CiA 402 for behaviours depending on value.
 *	NOTE: would be greatly facilitated by a CiA 402-compliant Finite State Automaton
 */
uint16_t ControlWord = 0;	// XXX temporary workaround until proper FSA is done
uint32_t CONTROL_WORD_Size(CO_OBJ *obj, CO_NODE *node, uint32_t width) {
	(void)obj; (void)node; (void)width;
	return 2;	// control word is 2 bytes
}

CO_ERR	CONTROL_WORD_Write (CO_OBJ *obj, CO_NODE *node, void *buffer, uint32_t size) {
	static int fault_reset_up = 0;
	COObjWrDirect(obj, buffer, CONTROL_WORD_Size(obj, node, size));
	uint16_t command = *(uint16_t*)obj->Data & CONTROL_WORD_COMMAND_MASK;

	//TODO: handle ms, oms, and h here.

	if (command & CONTROL_WORD_FAULT_RESET) {
		if (fault_reset_up) {
			return CO_ERR_NONE;
		}
		fault_reset_up = 1;
		//TODO: actually reset faults
		return FsaAttemptTransition(15) >= 0 ? CO_ERR_NONE : -1;
	}	// we now know that bit 7 is low
	fault_reset_up = 0;
	if ((command & CONTROL_WORD_DISABLE_VOLTAGE_N) == 0) {
	        //TODO: disable voltage
		if (FsaAttemptTransition( 9) > 0)	return CO_ERR_NONE;
	        if (FsaAttemptTransition( 7) > 0)	return CO_ERR_NONE;
	        if (FsaAttemptTransition(12) > 0)	return CO_ERR_NONE;
		return -1;	//TODO: find better error value
	}       // we now know that bit 1 is high
	if ((command & CONTROL_WORD_QUICK_STOP_N) == 0) {
	        //TODO: implement more nuanced setup described in 402 part 2 8.4.1
	        //mc_interface_set_current_rel(0.0);
		if (FsaAttemptTransition(11) > 0)	return CO_ERR_NONE;
	        if (FsaAttemptTransition( 7) > 0)	return CO_ERR_NONE;
		return -1;	//TODO: find better error value
	}       // we now know that bit 2 is high
	if ((command & CONTROL_WORD_SHUTDOWN_N) == 0) {
		if (FsaAttemptTransition( 2) > 0)	return CO_ERR_NONE;
		if (FsaAttemptTransition( 6) > 0)	return CO_ERR_NONE;
		if (FsaAttemptTransition( 8) > 0)	return CO_ERR_NONE;
		return -1;	//TODO: find better error value
	}       // we now know that bit 0 is high
	if ((command & CONTROL_WORD_SWITCH_ON_N) == 0) {
		if (FsaAttemptTransition( 3) > 0)	return CO_ERR_NONE;
		return -1;	//TODO: find better error value
	}	//we now know that bit 3 is high
	//TODO: implement switch on + enable operation OR enable operation depending on FSA state
	return CO_ERR_NONE;
}

const CO_OBJ_TYPE COTCONTROLWORD = {
	CONTROL_WORD_Size,
	0,
	0,
	CONTROL_WORD_Write
};



uint32_t DUTY_COMMAND_Size (CO_OBJ *obj, CO_NODE *node, uint32_t width) {
	(void)obj; (void)node; (void)width;
	return 2;
}
CO_ERR   DUTY_COMMAND_Init (CO_OBJ *obj, CO_NODE *node) {
	(void)node;
	obj->Data = 0;
	return CO_ERR_NONE;
}
CO_ERR   DUTY_COMMAND_Read (CO_OBJ *obj, CO_NODE *node, void *buffer, uint32_t size) {
	(void)node;
	if (size < 2)
		return CO_ERR_OBJ_SIZE;
	*(float*)buffer = *(float*)(obj->Data);
	return CO_ERR_NONE;
}
CO_ERR   DUTY_COMMAND_Write(CO_OBJ *obj, CO_NODE *node, void *buffer, uint32_t size) {
	(void)node;
	
	if (((ControlWord & CONTROL_WORD_COMMAND_MASK)& ~0x9) == 0x2) return CO_ERR_NONE;	//kludge for control word-rpm command interaction
	//format: from 16 bit signed fixed point to float
	float o_data = (float)(*(int16_t*)(buffer)) / 0xFFFF;
	mc_interface_set_duty(o_data);
	timeout_reset();
	*(float*)obj->Data = o_data;
	return CO_ERR_NONE;
}

const CO_OBJ_TYPE COTDUTYCOMMAND = {
	DUTY_COMMAND_Size,
	0,
	DUTY_COMMAND_Read,
	DUTY_COMMAND_Write
};

uint32_t CURRENT_COMMAND_Size (CO_OBJ *obj, CO_NODE *node, uint32_t width) {
	(void)obj; (void)node; (void)width;
	return 2;
}
CO_ERR   CURRENT_COMMAND_Init (CO_OBJ *obj, CO_NODE *node) {
	(void)node;
	obj->Data = 0;
	return CO_ERR_NONE;
}
CO_ERR   CURRENT_COMMAND_Read (CO_OBJ *obj, CO_NODE *node, void *buffer, uint32_t size) {
	(void)node;
	if (size < 2)
		return CO_ERR_OBJ_SIZE;
	*(float*)buffer = *(float*)(obj->Data);
	return CO_ERR_NONE;
}
CO_ERR   CURRENT_COMMAND_Write(CO_OBJ *obj, CO_NODE *node, void *buffer, uint32_t size) {
	(void)node;
	
	if (((ControlWord & CONTROL_WORD_COMMAND_MASK)& ~0x9) == 0x2) return CO_ERR_NONE;	//kludge for control word-rpm command interaction
	//format: from 16 bit signed fixed point to float
	float o_data = (float)(*(int16_t*)(buffer)) / 0xFF;
	mc_interface_set_current(o_data);
	timeout_reset();
	*(float*)obj->Data = o_data;
	return CO_ERR_NONE;
}

const CO_OBJ_TYPE COTCURRENTCOMMAND = {
	CURRENT_COMMAND_Size,
	0,
	CURRENT_COMMAND_Read,
	CURRENT_COMMAND_Write
};

