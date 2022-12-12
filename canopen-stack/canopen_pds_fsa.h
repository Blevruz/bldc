#ifndef CANOPEN_PDS_FSA_H_
#define CANOPEN_PDS_FSA_H_

#include "canopen_driver.h"
#include "comm_can.h"

#define FSA_FAULT_BIT		(1<<7)	//marks fault handling modes

typedef enum {
	FSA_S_START = 0,
	FSA_S_NOT_READY_TO_SWITCH_ON,
	FSA_S_SWITCH_ON_DISABLED,
	FSA_S_READY_TO_SWITCH_ON,
	FSA_S_SWITCHED_ON,
	FSA_S_OPERATION_ENABLED,
	FSA_S_QUICK_STOP_ACTIVE,
	FSA_S_NO_FAULT = FSA_FAULT_BIT - 1,
	FSA_S_FAULT,	
	FSA_S_FAULT_REACTION_ACTIVE,
} fsa_states;
extern int fsa_state;

typedef struct {
	int prev_state;
	int next_state;
	int (*on_transition_cb)();
} transition_t;

#define FSA_NB_TRANSITIONS	17
extern const transition_t fsa_transition_list[FSA_NB_TRANSITIONS];

int FsaAttemptTransition(int transition_number);	//handles state transitions and associated function calls

extern int motor_control;

#endif //CANOPEN_PDS_FSA_H_
