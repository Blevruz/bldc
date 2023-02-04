#include "canopen_pds_objects.h"

//useful for motor speed stuff
static float mc_enc_ratio = 0;

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
 *	Bit 11 - 15:	ms	manufacturer specific
 *	
 *	See Table 27 from CiA 402 for behaviours depending on value.
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

	if (command & CONTROL_WORD_FAULT_RESET) {	// fault reset
		if (fault_reset_up) {
			return CO_ERR_NONE;
		}
		fault_reset_up = 1;
		//TODO: actually reset faults
		return FsaAttemptTransition(15, d) >= 0 ? CO_ERR_NONE : -1;
	}	// we now know that bit 7 is low
	fault_reset_up = 0;
	if ((command & CONTROL_WORD_DISABLE_VOLTAGE_N) == 0) {	// disable voltage
	        //TODO: disable voltage
		if (FsaAttemptTransition( 9, d) > 0)	return CO_ERR_NONE;
	        if (FsaAttemptTransition( 7, d) > 0)	return CO_ERR_NONE;
	        if (FsaAttemptTransition(12, d) > 0)	return CO_ERR_NONE;
		return -1;	//TODO: find better error value
	}       // we now know that bit 1 is high
	if ((command & CONTROL_WORD_QUICK_STOP_N) == 0) {	// quick stop
	        //TODO: implement more nuanced setup described in 402 part 2 8.4.1
	        //mc_interface_set_current_rel(0.0);
		if (FsaAttemptTransition(11, d) > 0)	return CO_ERR_NONE;
	        if (FsaAttemptTransition( 7, d) > 0)	return CO_ERR_NONE;
		return -1;	//TODO: find better error value
	}       // we now know that bit 2 is high
	if ((command & CONTROL_WORD_SHUTDOWN_N) == 0) {		// shutdown
		if (FsaAttemptTransition( 2, d) > 0)	return CO_ERR_NONE;
		if (FsaAttemptTransition( 6, d) > 0)	return CO_ERR_NONE;
		if (FsaAttemptTransition( 8, d) > 0)	return CO_ERR_NONE;
		return -1;	//TODO: find better error value
	}       // we now know that bit 0 is high
	if ((command & CONTROL_WORD_SWITCH_ON_N) == 0) {	// switch on / disable op
		if (FsaAttemptTransition( 3, d) > 0)	return CO_ERR_NONE;
		return -1;	//TODO: find better error value
	}	//we now know that bit 3 is high
								// switch on + enable op
	//TODO: implement switch on + enable operation OR enable operation depending on FSA state
	
	if (FsaAttemptTransition( 4, d) > 0)	return CO_ERR_NONE;
	if (FsaAttemptTransition(16, d) > 0)	return CO_ERR_NONE;
	if (FsaAttemptTransition( 3, d) > 0)	return CO_ERR_NONE;

	return -1;
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
	(void)size;
	
	if (!FSA_CHECK_MOTOR_ON)	return CO_ERR_NONE;

	//format: from 16 bit signed fixed point to float
	float o_data = (float)(*(int16_t*)(buffer)) / 0x7FFF;
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
	(void)size;
	
	if (!FSA_CHECK_MOTOR_ON)	return CO_ERR_NONE;

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



uint32_t RPM_COMMAND_Size (CO_OBJ *obj, CO_NODE *node, uint32_t width) {
	(void)obj; (void)node; (void)width;
	return 2;
}
CO_ERR   RPM_COMMAND_Init (CO_OBJ *obj, CO_NODE *node) {
	(void)node;
	obj->Data = 0;
	return CO_ERR_NONE;
}
CO_ERR   RPM_COMMAND_Read (CO_OBJ *obj, CO_NODE *node, void *buffer, uint32_t size) {
	(void)node;
	if (size < 2)
		return CO_ERR_OBJ_SIZE;
	*(float*)buffer = *(float*)(obj->Data);
	return CO_ERR_NONE;
}
CO_ERR   RPM_COMMAND_Write(CO_OBJ *obj, CO_NODE *node, void *buffer, uint32_t size) {
	(void)node;
	(void)size;

	if (mc_enc_ratio == 0) {
		mc_configuration mc_config;
		conf_general_read_mc_configuration(&mc_config, false);
		mc_enc_ratio = mc_config.foc_encoder_ratio;
	}

	if (!FSA_CHECK_MOTOR_ON)	return CO_ERR_NONE;

	int32_t o_data = *(int16_t*)(buffer) * mc_enc_ratio;
	mc_interface_set_pid_speed((int32_t)o_data);
	timeout_reset();
	*(uint32_t*)obj->Data = o_data;
	return CO_ERR_NONE;
}

const CO_OBJ_TYPE COTRPMCOMMAND = {
	RPM_COMMAND_Size,
	0,
	RPM_COMMAND_Read,
	RPM_COMMAND_Write
};


uint16_t StatusWord = 0;

/*	STATUS WORD: See CiA 402 part 2 section 8.4.2
 *		Structure:
 *	Bit 0: 		rtso	ready to switch on
 *	Bit 1:		so	switched on
 *	Bit 2:		oe	operation enabled
 *	Bit 3:		f	fault
 *	Bit 4:		ve	voltage enabled
 *	Bit 5:		qs	quick stop
 *	Bit 6:		sod	switch on disabled
 *	Bit 7:		w	warning
 *	Bit 8:		ms	manufacturer specific
 *	Bit 9:		rm	remote
 *	Bit 10:		tr	target reached
 *	Bit 11:		ila	internal limit active
 *	Bit 12 - 13:	oms	operation mode specific 
 *	Bit 14 - 15:	ms	manufacturer specific 
 *	
 *	see table 30 for patterns for bits 0, 1, 2, 3, 5 and 6
 */
uint32_t STATUS_WORD_Size(CO_OBJ *obj, CO_NODE *node, uint32_t width) {
	(void)obj; (void)node; (void)width;
	return 2;	// status word is 2 bytes
}

CO_ERR	STATUS_WORD_Read (CO_OBJ *obj, CO_NODE *node, void *buffer, uint32_t size) {
	(void)size;
	uint16_t new_state = *(uint16_t*)obj->Data;
	switch(fsa_state) {
		case FSA_S_NOT_READY_TO_SWITCH_ON:
			new_state &= ~0x4F;
			break;
		case FSA_S_SWITCH_ON_DISABLED:
			new_state &= ~0x0F;
			new_state |= 0x40;
			break;
		case FSA_S_READY_TO_SWITCH_ON:
			new_state &= ~0x4E;
			new_state |= 0x21;
			break;
		case FSA_S_SWITCHED_ON:
			new_state &= ~0x4C;
			new_state |= 0x23;
			break;
		case FSA_S_OPERATION_ENABLED:
			new_state &= ~0x48;
			new_state |= 0x27;
			break;
		case FSA_S_QUICK_STOP_ACTIVE:
			new_state &= ~0x68;
			new_state |= 0x07;
			break;
		case FSA_S_FAULT_REACTION_ACTIVE:
			new_state &= ~0x40;
			new_state |= 0x0F;
			break;
		case FSA_S_FAULT:
			new_state &= ~0x47;
			new_state |= 0x08;
			break;
		default:
			break;
	}	//bits 0, 1, 2, 3, 5, 6 set by now

	//bit 4: in practice, we always have voltage enabled, so leave it at 1
	//TODO: have it set by the FSA?
	new_state |= 1<<4;

	//bit 7: TODO allow it to be set by random functions
	
	//bit 9: TODO implement actual way to toggle controlword processing
	new_state |= 1<<9;

	//bit 10: compares actual speed and target speed, within a certain margin
	//TODO: get that margin from somewhere in the OD
	int16_t target_speed = 0;
	CODictRdWord(&node->Dict, CO_DEV(0x6042, 1), (uint16_t*)&target_speed);
	
	if (mc_enc_ratio) {
		target_speed /= mc_enc_ratio;	//adjusting for ERPM to RPM transition
	}
	int16_t measured_speed = 0;
	CODictRdWord(&node->Dict, CO_DEV(0x6044, 0), (uint16_t*)&measured_speed);
	int16_t margin = 10;	//acceptable percentage deviation

	if (measured_speed*100 >= target_speed*(100-margin)  && measured_speed*100 <= target_speed*(100+margin)) {
		new_state |= 1<<10;
	} else {
		new_state &= ~(1<<10);
	}

	//bit 11: TODO compare actual speed and max/min speed
	new_state &= ~(1<<11);

	*(uint16_t*)obj->Data = new_state;
	*(uint16_t*)buffer = new_state;

	return CO_ERR_NONE;
}

const CO_OBJ_TYPE COTSTATUSWORD = {
	STATUS_WORD_Size,
	0,
	STATUS_WORD_Read,
	0
};
