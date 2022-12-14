#include "canopen_pds_fsa.h"

#define CO_FSA_AUTO_TRANSITION	1

static int	on_transition_0	();
static int	on_transition_1	();
static int	on_transition_2	();
static int	on_transition_3	();
static int	on_transition_4	();
static int	on_transition_5	();
static int	on_transition_6	();
static int	on_transition_7	();
static int	on_transition_8	();
static int	on_transition_9	();
static int	on_transition_10();
static int	on_transition_11();
static int	on_transition_12();
static int	on_transition_13();
static int	on_transition_14();
static int	on_transition_15();
static int	on_transition_16();

int fsa_state = 0;
int motor_control = 0;

const transition_t fsa_transition_list[FSA_NB_TRANSITIONS] = {
	{FSA_S_START, FSA_S_NOT_READY_TO_SWITCH_ON, on_transition_0},			//0
	{FSA_S_NOT_READY_TO_SWITCH_ON, FSA_S_SWITCH_ON_DISABLED, on_transition_1},	//1
	{FSA_S_SWITCH_ON_DISABLED, FSA_S_READY_TO_SWITCH_ON, on_transition_2},		//2
	{FSA_S_READY_TO_SWITCH_ON, FSA_S_SWITCHED_ON, on_transition_3},			//3
	{FSA_S_SWITCHED_ON, FSA_S_OPERATION_ENABLED, on_transition_4},			//4
	{FSA_S_OPERATION_ENABLED, FSA_S_SWITCHED_ON, on_transition_5},			//5
	{FSA_S_SWITCHED_ON, FSA_S_READY_TO_SWITCH_ON, on_transition_6},			//6
	{FSA_S_READY_TO_SWITCH_ON, FSA_S_SWITCH_ON_DISABLED, on_transition_7},		//7
	{FSA_S_OPERATION_ENABLED, FSA_S_READY_TO_SWITCH_ON, on_transition_8},		//8
	{FSA_S_OPERATION_ENABLED, FSA_S_SWITCH_ON_DISABLED, on_transition_9},		//9
	{FSA_S_SWITCHED_ON, FSA_S_SWITCH_ON_DISABLED, on_transition_10},		//10
	{FSA_S_OPERATION_ENABLED, FSA_S_QUICK_STOP_ACTIVE, on_transition_11},		//11
	{FSA_S_QUICK_STOP_ACTIVE, FSA_S_SWITCH_ON_DISABLED, on_transition_12},		//12
	{FSA_S_NO_FAULT, FSA_S_FAULT_REACTION_ACTIVE, on_transition_13},		//13
	{FSA_S_FAULT_REACTION_ACTIVE, FSA_S_FAULT, on_transition_14},			//14
	{FSA_S_FAULT, FSA_S_SWITCH_ON_DISABLED, on_transition_15},			//15
	{FSA_S_QUICK_STOP_ACTIVE, FSA_S_OPERATION_ENABLED, on_transition_16}		//16
};

// transition 0: self test and init
static int 	on_transition_0	() {
	return canopen_driver_init();
}

// transition 1: activate commm
static int 	on_transition_1	() {
	comm_can_set_sid_rx_callback(canopen_sid_callback);
	/*	TODO
	timer_id = COTmrCreate(	&(co_node.Tmr),
			100,
			100,
			TPdo_Callback,
			0);
	*/
#if CO_FSA_AUTO_TRANSITION == 1
	return FsaAttemptTransition(2);
#endif
	return 1;
}

// transition 2: none
static int 	on_transition_2	() {	
#if CO_FSA_AUTO_TRANSITION == 1
	return FsaAttemptTransition(3);
#endif
	return 1;
}

// transition 3: motor power on
static int 	on_transition_3	() {	//TODO
#if CO_FSA_AUTO_TRANSITION == 1
	return FsaAttemptTransition(4);
#endif
	return 1;
}

// transition 4: enable motor control 
static int 	on_transition_4	() {	//TODO
	motor_control = 1;
	return 1;
}

// transition 5: disable motor control 
static int 	on_transition_5	() {	//TODO
	motor_control = 0;
	return 1;
}

// transition 6: motor power off 
static int 	on_transition_6	() {	//TODO
	return 1;
}

// transition 7: none 
static int 	on_transition_7	() {	//TODO
	return 1;
}

// transition 8: motor power off + disable motor control 
static int 	on_transition_8	() {	
	return (on_transition_6() + on_transition_5()) == 2 ? 1 : -1;
}

// transition 9: motor power off + disable motor control 
static int 	on_transition_9 () {
	return on_transition_8();
}

// transition 10: motor power off  
static int 	on_transition_10() {
	return on_transition_6();
}

// transition 11: quickstop 
static int 	on_transition_11() {
	mc_interface_set_current(0.0);
	return 1;
}

// transition 12: motor power off + disable motor control 
static int 	on_transition_12() {
	return on_transition_8();
}

// transition 13: use configured fault handler function 
static int 	on_transition_13() {	//TODO
	return 1;
}

// transition 14: motor power off + disable motor control 
static int 	on_transition_14() {
	return on_transition_8();
}

// transition 15: reset fault
static int 	on_transition_15() {	//TODO
	return 1;
}

//transition 16: enable motor control
static int 	on_transition_16() {
	return on_transition_4();
}

int FsaAttemptTransition(int transition_number) {
	if (transition_number >= FSA_NB_TRANSITIONS) return -1;	// early return for an invalid transition number
	transition_t t = fsa_transition_list[transition_number];

	if (t.prev_state == FSA_S_NO_FAULT && (fsa_state & FSA_FAULT_BIT)) return -1;	//early return for fault state
	if (t.prev_state != fsa_state) return -1;	// early return for wrong state
	int result = 1;
	fsa_state = t.next_state;	// TODO: ensure right state during transition AND automatic transitions
	if (t.on_transition_cb) result = t.on_transition_cb();	// most transitions should include a callback
	return result;
}
