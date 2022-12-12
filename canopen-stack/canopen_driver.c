#include "canopen_driver.h"

#define STATIC 	0
#define DYNAMIC 1
#define OD 	DYNAMIC

#define OD_SIZE 256	//TODO: replace with better value
#define OD_END_DW	0xCAFECAFE	//double word that marks the end of the dictionary

co_ring_buffer can_ring_buffer;
CO_NODE co_node;
struct CO_IF_DRV_T AppDriver = {
    &VescPkgCanDriver,
    &SwCycleTimerDriver,
    &ChOSNvmDriver
};
 
/* Specify the EMCY-IDs for the application */
enum EMCY_CODES {	//HUGE TODO: actually do error handling
    APP_ERR_ID_SOMETHING = 0,
    APP_ERR_ID_HOT,

    APP_ERR_ID_NUM            /* number of EMCY error codes in application */
};
 
/* Specify the EMCY error codes with the corresponding
 * error register bit. There is a collection of defines
 * for the predefined emergency codes CO_EMCY_CODE...
 * and for the error register bits CO_EMCY_REG... for
 * readability. You can use plain numbers, too.
 */
static CO_EMCY_TBL AppEmcyTbl[APP_ERR_ID_NUM] = {
    { CO_EMCY_REG_GENERAL, CO_EMCY_CODE_GEN_ERR          }, /* APP_ERR_CODE_SOMETHING */
    { CO_EMCY_REG_TEMP   , CO_EMCY_CODE_TEMP_AMBIENT_ERR }  /* APP_ERR_CODE_HAPPENS   */
};

#define APP_TMR_N         16u                 /* Number of software timers   */
CO_TMR_MEM AppTmrMem[APP_TMR_N];
#define APP_TICKS_PER_SEC 1000u               /* Timer clock frequency in Hz */
static uint8_t SdoSrvMem[CO_SSDO_N * CO_SDO_BUF_BYTE];

//  OD SECTION
//declaring dynamic OD type

/* allocate global variables for runtime value of objects */
static uint8_t  Obj1001_00_08 = 0;

#define APP_OBJ_N	256u
OD_DYN AppOD = {
	(CO_OBJ*)NVM_CHOS_ADDRESS,
	APP_OBJ_N,
	0
};

static uint8_t set_od_root (CO_OBJ* new_root) {	//function to synchronise the co_node_spec and AppOD root values
	// root can only be within designated NVM sector (by default sector 8)
	if ((uint32_t)new_root < NVM_CHOS_ADDRESS || (uint32_t)new_root - NVM_CHOS_ADDRESS >= NVM_CHOS_SIZE)
		return -1;
	ChOSNvmDriver_offset = (uint32_t)new_root - NVM_CHOS_ADDRESS;
	AppOD.Root = new_root;
	co_node_spec.Dict = new_root;
	co_node.Dict.Root = new_root;	//TODO: check which ones can safely be removed (redundancy)
	return 1;
}

static void ObjSet(CO_OBJ *obj, uint32_t key, const CO_OBJ_TYPE *type, CO_DATA data)
{
  obj->Key  = key;
  obj->Type = type;
  obj->Data = data;
}

static void ObjCpy(CO_OBJ *a, CO_OBJ *b)
{
  a->Key  = b->Key;
  a->Type = b->Type;
  a->Data = b->Data;
}

/*
static void ObjSwap(CO_OBJ *a, CO_OBJ *b)
{
  CO_OBJ x;

  ObjCpy(&x,  a);
  ObjCpy( a,  b);
  ObjCpy( b, &x);
}

static int16_t ObjCmp(CO_OBJ *a, CO_OBJ *b)
{
  int16_t result = 1;

  if (CO_GET_DEV(a->Key) == CO_GET_DEV(b->Key)) {
    result = 0;

  } else if (CO_GET_DEV(a->Key) < CO_GET_DEV(b->Key)) {
    result = -1;
  }

  return (result);
}
*/

#define TEMP_BUFFER_SIZE 512
CO_OBJ  t_buffer[TEMP_BUFFER_SIZE];
uint32_t	t_used = 0;

void	ODEraseNvm() {
	/*
		FLASH_Unlock();
		FLASH_ClearFlag(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
				FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
		FLASH_EraseSector(8 << 3, (uint8_t)((PWR->CSR & PWR_CSR_PVDO) ? VoltageRange_2 : VoltageRange_3));
			//Note the `8 << 3`; this function doesnt bitshift the sector number
		FLASH_Lock();
	*/
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
			FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
	
	if (get_canopen_ready()) {

		if (nrf_driver_ext_nrf_running()) {
			nrf_driver_pause(10000);
		}

		mc_interface_unlock();
	
		mc_interface_ignore_input_both(10000);
		mc_interface_release_motor_override_both();

		mc_interface_lock();
	
		while (!mc_interface_wait_for_motor_release_both(3));
	
		utils_sys_lock_cnt();
		timeout_configure_IWDT_slowest();
	}

	uint16_t res = FLASH_EraseSector(8 << 3, (uint8_t)((PWR->CSR & PWR_CSR_PVDO) ? VoltageRange_2 : VoltageRange_3));
			//Note the `8 << 3`; this function doesnt bitshift the sector number

	FLASH_Lock();
	if (get_canopen_ready()) {
		timeout_configure_IWDT();
		mc_interface_ignore_input_both(100);
		utils_sys_unlock_cnt();
	}
}


/**
 * @brief Writes pointed object to either dictionary or buffer depending on OD staticity.
 *
 * @param driver
 * pointer to CANopen driver
 *
 * @param self
 * pointer to CANopen object dictionary
 *
 * @param to_write
 * pointer to the CANopen object to write in the buffer
 *
 * @return 1 if static, 2 if dynamic addition, 3 if dynamic replacement, -1 if failure
 */
int8_t ODEntryToBuffer (CO_IF_DRV* driver, OD_DYN* self, CO_OBJ* to_write) {
#if OD == STATIC
	if (driver->Nvm->Write((self->Used++)*sizeof(CO_OBJ), to_write, sizeof(CO_OBJ)) != FLASH_COMPLETE)
		return -1;
	return 1;
#else
	(void)driver;
	(void)self;
	for (volatile int i = 0; i < t_used; i++) {	//volatile keyword to keep this loop in spite of optimisation
		if ((t_buffer[i].Key & ~0xFF) == (to_write->Key & ~0xFF)) {	// If other object with same Index&Subindex:
			t_buffer[i] = *to_write;			// Replace it with more recent obj
			return 3;
		}
	}
	if (t_used < TEMP_BUFFER_SIZE) {
		t_buffer[t_used++] = *to_write;
		return 2;
	}
	return -1;	//error: no more room
#endif
}

/**
 * Writes the buffer's content to NVM if OD is dynamic (does nothing if static)
 *
 * @param driver
 * pointer to CANopen driver
 *
 * @param self
 * pointer to CANopen object dictionary
 */
void ODBufferToNvm(CO_IF_DRV* driver, OD_DYN* self) {
#if OD == STATIC
	return;
#else	
	// Can we append?
	const unsigned int size = sizeof(CO_OBJ);
	if (((uint32_t)self->Root - NVM_CHOS_ADDRESS) + self->Used*size + t_used*size < NVM_CHOS_SIZE &&
			self->Used != 0) {
		uint32_t end_dw = OD_END_DW;	// If so: change dictionary root
		const unsigned int size = sizeof(uint32_t);

		driver->Nvm->Write((self->Used)*3*size, (uint8_t*)&end_dw, size);
		set_od_root((uint32_t)self->Root + (self->Used*3 + 1)*size);

		self->Used = 0;
	} else {
		// If not: wipe and write
		ODEraseNvm();
		set_od_root(NVM_CHOS_ADDRESS);
	}
//	ChOSNvmDriver_offset = (uint32_t)self->Root - NVM_CHOS_ADDRESS;
	uint32_t used = 0;
	uint32_t max = 0xFFFFFFFF;
	uint32_t min = 0;
	uint32_t ival; int bi = -1;
	while (t_used > used) {
		for (unsigned int i = 0; i < t_used; i++) {	//O(n^2) but rare
			ival = CO_GET_DEV(t_buffer[i].Key);
			if (ival > min && ival < max) {
				max = ival;
				bi = i;
			}
		}
		if (bi == -1) break;	// something is wrong
		min = CO_GET_DEV(t_buffer[bi].Key);
		max = 0xFFFFFFFF;
		driver->Nvm->Write((used++)*size, (uint8_t*)&(t_buffer[bi]), size);
	}
	self->Used = used;
#endif
}

void ODNvmToBuffer(CO_IF_DRV* driver, OD_DYN* self) {
#if OD == STATIC
	return;
#else
	const unsigned int size = sizeof(CO_OBJ);
	CO_OBJ obj_buffer;
	int i = 0, key = 0;
	//for (unsigned int i = 0; i < self->Used; i++) {
	for(;;) {
		driver->Nvm->Read(i++*size, (uint8_t*)&obj_buffer, size);
		key = ((uint8_t*)&obj_buffer)[3];
		if (key >= 0xC0) break;
		//t_buffer[t_used++] = obj_buffer;
		ODEntryToBuffer(driver, self, &obj_buffer);
	}
#endif
}

void ODClearBuffer() {
#if OD == STATIC
	return;
#else
	t_used = 0;
#endif
}

void ODInit (OD_DYN *self, CO_OBJ *root, uint32_t length)
{
    uint32_t  idx;
    CO_OBJ    end = CO_OBJ_DIR_ENDMARK;
    CO_OBJ   *od;

    idx = 0;
    od  = root;
    while (idx < length) {
        ObjCpy(self->Root, &end);
        od++;
        idx++;
    }

    self->Root   = root;
    self->Length = length - 1;
    self->Used   = 0;

}

int ODAddUpdate(OD_DYN *self, uint32_t key, const CO_OBJ_TYPE *type, CO_DATA data, CO_IF_DRV* driver)
{	//Copied wholesale from canopen-stack/example/dynamic-od/app/app_dict.c then modified
    CO_OBJ  temp;

    if ((key == 0) ||
        (self->Used >= self->Length)) {
        return;
    }

    //CO_OBJ *od = self->Root;
    ObjSet(&temp, key, type, data);

    return ODEntryToBuffer(driver, self, &temp);
}

static void ODCreateSDOServer(OD_DYN *self, uint8_t srv, uint32_t request, uint32_t response, CO_IF_DRV* driver)
{
    if (srv == 0) {
        request  = (uint32_t)0x600;
        response = (uint32_t)0x580;
    }
    ODAddUpdate(self, CO_KEY(0x1200+srv, 0, CO_UNSIGNED8 |CO_OBJ_D__R_), 0, (CO_DATA)(0x02), driver);
    ODAddUpdate(self, CO_KEY(0x1200+srv, 1, CO_UNSIGNED32|CO_OBJ_DN_R_), 0, (CO_DATA)(request), driver);
    ODAddUpdate(self, CO_KEY(0x1200+srv, 2, CO_UNSIGNED32|CO_OBJ_DN_R_), 0, (CO_DATA)(response), driver);
}

//mandatory entries for CiA301 compliance
#define ODLIST_SIZE 9
uint32_t ODList [ODLIST_SIZE] = {	CO_KEY(0x1000, 0, CO_UNSIGNED32|CO_OBJ_D__R_),	// Device Type
                      			CO_KEY(0x1001, 0, CO_UNSIGNED8 |CO_OBJ___PR_),	// Error Register
                      			CO_KEY(0x1005, 0, CO_UNSIGNED32|CO_OBJ_D__R_),	// COB-ID SYNC
                      			CO_KEY(0x1017, 0, CO_UNSIGNED16|CO_OBJ_D__R_),	// Heartbeat Producer
                      			CO_KEY(0x1018, 0, CO_UNSIGNED8 |CO_OBJ_D__R_),	// Identity Object
                      			CO_KEY(0x1018, 1, CO_UNSIGNED32|CO_OBJ_D__R_),	//  | Vendor ID
                      			CO_KEY(0x1018, 2, CO_UNSIGNED32|CO_OBJ_D__R_),	//  | Product code
                      			CO_KEY(0x1018, 3, CO_UNSIGNED32|CO_OBJ_D__R_),	//  | Revision number
                      			CO_KEY(0x1018, 4, CO_UNSIGNED32|CO_OBJ_D__R_)};	//  | Serial number

/* function to setup the quickstart object dictionary */
static void ODCreateDict(OD_DYN *self, CO_IF_DRV* driver)
{

	//TODO:
	//[X]	check NVM for any data (should be 0xFFFFFFFF words if not)
	//[~]	check if that data is an OD (3 word ensembles where the first word is readable as a key?)
	//[X]	check if that OD contains the necessary entries
	//[X] 	if so, just return
	//[ ]	if not:
	//  |	[ ]	if content is an OD: store in buffer and add necessary entries
	//  |	[ ]	else erase and write necessary entries

    int size = sizeof(uint32_t);
    int matches = 0;
    int odindex = 0;
    int odoffset = 0;
    while (matches < ODLIST_SIZE && odindex < OD_SIZE) {
	uint32_t buffer;
	driver->Nvm->Read((odindex + odoffset)*size, (uint8_t*)&buffer, size);
	if (buffer == ODList[matches]) {	//if we found one of our entries, move onto the next
		matches++;
		odindex += 3;
		continue;
	}
	if (buffer == OD_END_DW) {		//if we found the end of this dictionary, reset values and go on
		odoffset = odindex + 1;		//end dword takes up 1 address
		odindex = 0;
		matches = 0;
		continue;
	}
	if (buffer >= 0xBFFFFFFF || (odindex + odoffset) >= NVM_CHOS_SIZE) break;	//we've reached the end of NVM (0xBFFF is the max index)
	odindex += 3;	//assume it must have been a valid entry and move on
    }
    //self->Used += odindex/3;
    if (self->Used < odindex/3) self->Used = odindex/3;
    //if (matches == ODLIST_SIZE) return;		//we have a functional minimal OD: nothing to do here

    if (matches != ODLIST_SIZE) {

   	 if (set_od_root((CO_OBJ*)((uint32_t)self->Root + odoffset*sizeof(CO_OBJ*))) == -1)
   	         for(;;);	//for ease of debugging ( XXX do something cleaner)

   	 Obj1001_00_08 = 0;

   	 ODAddUpdate(self, CO_KEY(0x1000, 0, CO_UNSIGNED32|CO_OBJ_D__R_), 0, (CO_DATA)(0x00000000), driver);
   	 ODAddUpdate(self, CO_KEY(0x1001, 0, CO_UNSIGNED8 |CO_OBJ___PR_), 0, (CO_DATA)(&Obj1001_00_08), driver);
   	 ODAddUpdate(self, CO_KEY(0x1005, 0, CO_UNSIGNED32|CO_OBJ_D__R_), 0, (CO_DATA)(0x80), driver);	//TODO: add node ID to that
   	 ODAddUpdate(self, CO_KEY(0x1017, 0, CO_UNSIGNED16|CO_OBJ_D__R_), 0, (CO_DATA)(0), driver);
   	 ODAddUpdate(self, CO_KEY(0x1018, 0, CO_UNSIGNED8 |CO_OBJ_D__R_), 0, (CO_DATA)(4), driver);
   	 ODAddUpdate(self, CO_KEY(0x1018, 1, CO_UNSIGNED32|CO_OBJ_D__R_), 0, (CO_DATA)(0), driver);
   	 ODAddUpdate(self, CO_KEY(0x1018, 2, CO_UNSIGNED32|CO_OBJ_D__R_), 0, (CO_DATA)(0), driver);
   	 ODAddUpdate(self, CO_KEY(0x1018, 3, CO_UNSIGNED32|CO_OBJ_D__R_), 0, (CO_DATA)(0), driver);
   	 ODAddUpdate(self, CO_KEY(0x1018, 4, CO_UNSIGNED32|CO_OBJ_D__R_), 0, (CO_DATA)(0), driver);

   	 ODCreateSDOServer(self, 0, CO_COBID_SDO_REQUEST(), CO_COBID_SDO_RESPONSE(), driver);
	
   }
   ODNvmToBuffer(driver, self);	//we shouldnt have written to NVM by now, but just in case...
   if (!get_canopen_ready()) {
	   ODEraseNvm();	// wiping nvm causes crash after startup for some reason
	   self->Used = 0;
   }
   ODBufferToNvm(driver, self);
   ODClearBuffer();
}

// END OD SECTION

CO_NODE_SPEC co_node_spec = {
    0x01,					/* default Node-Id (arbitrary)		*/
    APPCONF_CAN_BAUD_RATE,   			/* default Baudrate			*/
    (CO_OBJ*)NVM_CHOS_ADDRESS,			/* pointer to object dictionary  	*/
    APP_OBJ_N,        				/* object dictionary max length  	*/
    &AppEmcyTbl[0],				/* EMCY code & register bit table	*/
    &AppTmrMem[0],				/* pointer to timer memory blocks	*/
    APP_TMR_N,					/* number of timer memory blocks 	*/
    APP_TICKS_PER_SEC,				/* timer clock frequency in Hz   	*/
    &AppDriver,					/* select drivers for application	*/
    &SdoSrvMem[0]				/* SDO Transfer Buffer Memory    	*/
};

//virtual timer handling
virtual_timer_t co_vt;
void co_vt_update(void *p) {
	//TODO: add a way to shut it down
	(void)p;
	COTmrProcess(&(co_node.Tmr));
	COTmrService(&(co_node.Tmr));
	chVTSetI(&co_vt, MS2ST(1), co_vt_update, NULL);

}

int	canopen_driver_init() {

	co_node_spec.NodeId = app_get_configuration()->controller_id;
#if OD == STATIC
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
			FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
	FLASH_EraseSector(8 << 3, (uint8_t)((PWR->CSR & PWR_CSR_PVDO) ? VoltageRange_2 : VoltageRange_3));
		//Note the `8 << 3`; this function doesnt bitshift the sector number
	FLASH_Lock();
#endif
	/* Clear all entries in object dictionary */
	//ODInit(&AppOD, (CO_OBJ*)NVM_CHOS_ADDRESS, APP_OBJ_N);
	
	/* Setup the object dictionary during runtime */
	ODCreateDict(&AppOD, &AppDriver);
	
	/* Create a node with generated object dictionary */
	CONodeInit(&co_node, &co_node_spec);
	
	CO_ERR err = CONodeGetErr(&co_node);
	if (err != CO_ERR_NONE) {
		return -1;
		//while(1);
	}
	CONodeStart(&co_node);
	CONmtSetMode(&co_node.Nmt, CO_OPERATIONAL);
	chVTSetI(&co_vt, MS2ST(1), co_vt_update, NULL);
	return 1;
}

bool canopen_sid_callback(uint32_t id, uint8_t *data, uint8_t len) {
	//stores frame in buffer so it can be accessed by CAN driver
	if ((can_ring_buffer.wp+1)%CAN_RINGBUFFER_SIZE != can_ring_buffer.rp){
		t_can_frame *write_frame = &(can_ring_buffer.data[can_ring_buffer.wp]);
		write_frame->id = id;
		write_frame->len = len;
		memcpy(write_frame->data, data, write_frame->len);

		can_ring_buffer.wp = (++can_ring_buffer.wp)%CAN_RINGBUFFER_SIZE;
	}
	//calls CONodeProcess to call its CAN driver to process the frame
	CONodeProcess(&co_node);	//TODO: put in args
	
}

static uint8_t _ready = 0;
uint8_t get_canopen_ready() {return _ready;}
void set_canopen_ready(uint8_t ready) {_ready = ready;}
