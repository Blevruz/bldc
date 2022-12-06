
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
int fsa_state = 0;

typedef struct {
	int prev_state;
	int next_state;
	int (*on_transition_cb)();
} transition_t;

#define FSA_NB_TRANSITIONS	17
static transition fsa_transition_list[FSA_NB_TRANSITIONS] = {
	{FSA_S_START, FSA_S_NOT_READY_TO_SWITCH_ON, NULL},			//0
	{FSA_S_NOT_READY_TO_SWITCH_ON, FSA_S_SWITCH_ON_DISABLED, NULL},		//1
	{FSA_S_SWITCH_ON_DISABLED, FSA_S_READY_TO_SWITCH_ON, NULL},		//2
	{FSA_S_READY_TO_SWITCH_ON, FSA_S_SWITCHED_ON, NULL},			//3
	{FSA_S_SWITCHED_ON, FSA_S_OPERATION_ENABLED, NULL},			//4
	{FSA_S_OPERATION_ENABLED, FSA_S_SWITCHED_ON, NULL},			//5
	{FSA_S_SWITCHED_ON, FSA_S_READY_TO_SWITCH_ON, NULL},			//6
	{FSA_S_READY_TO_SWITCH_ON, FSA_S_SWITCH_ON_DISABLED, NULL},		//7
	{FSA_S_OPERATION_ENABLED, FSA_S_READY_TO_SWITCH_ON, NULL},		//8
	{FSA_S_OPERATION_ENABLED, FSA_S_SWITCH_ON_DISABLED, NULL},		//9
	{FSA_S_SWITCHED_ON, FSA_S_SWITCH_ON_DISABLED, NULL},			//10
	{FSA_S_OPERATION_ENABLED, FSA_S_QUICK_STOP_ACTIVE, NULL},		//11
	{FSA_S_QUICK_STOP_ACTIVE, FSA_S_SWITCH_ON_DISABLED, NULL},		//12
	{FSA_S_NO_FAULT, FSA_S_FAULT_REACTION_ACTIVE, NULL},			//13
	{FSA_S_FAULT_REACTION_ACTIVE, FSA_F_FAULT, NULL},			//14
	{FSA_S_FAULT, FSA_S_SWITCH_ON_DISABLED, NULL},				//15
	{FSA_S_QUICK_STOP_ACTIVE, FSA_S_OPERATION_ENABLED, NULL}		//16
}

int FsaAttemptTransition(int transition_number) {
	if (transition_number >= FSA_NB_TRANSITIONS) return -1;
	transition t = fsa_transitions[transition_number];

	if (t.prev_state == FSA_S_NO_FAULT && (fsa_state & FSA_S_FAULT)) return -1;
	if (t.prev_state != fsa_state) return -1;
	fsa_state = t.next_state;
	if (t.on_transition_cb) return t.on_transition_cb();
	return 1;
}
