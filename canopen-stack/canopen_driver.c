#include "canopen_driver.h"

#define STATIC 	0
#define DYNAMIC 1
#define OD 	DYNAMIC

CO_NODE co_node;
struct CO_IF_DRV_T AppDriver = {
    &ChOSCanDriver,
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

#define TEMP_BUFFER_SIZE 256
CO_OBJ  t_buffer[TEMP_BUFFER_SIZE];
uint32_t	t_used = 0;

int8_t ODEntryToBuffer (CO_IF_DRV* driver, OD_DYN* self, CO_OBJ* to_write) {
#if OD == STATIC
	driver->Nvm->Write((self->Used++)*sizeof(CO_OBJ), to_write, sizeof(CO_OBJ));
	return 1;
#else
	(void)driver;
	(void)self;
	if (t_used < TEMP_BUFFER_SIZE) {
		t_buffer[t_used++] = *to_write;
	}
	return 1;
#endif
}

void ODBufferToNvm(CO_IF_DRV* driver, OD_DYN* self) {
#if OD == STATIC
	return;
#else
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
			FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
	FLASH_EraseSector(8 << 3, (uint8_t)((PWR->CSR & PWR_CSR_PVDO) ? VoltageRange_2 : VoltageRange_3));
		//Note the `8 << 3`; this function doesnt bitshift the sector number
	FLASH_Lock();
	uint32_t used = 0;
	const unsigned int size = sizeof(CO_OBJ);
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
	for (unsigned int i = 0; i < self->Used; i++) {
		driver->Nvm->Read(i*size, (uint8_t*)&obj_buffer, size);
		t_buffer[t_used++] = obj_buffer;
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

void ODAddUpdate(OD_DYN *self, uint32_t key, const CO_OBJ_TYPE *type, CO_DATA data, CO_IF_DRV* driver)
{	//Copied wholesale from canopen-stack/example/dynamic-od/app/app_dict.c then modified
    CO_OBJ  temp;

    if ((key == 0) ||
        (self->Used >= self->Length)) {
        return;
    }

    //CO_OBJ *od = self->Root;
    ObjSet(&temp, key, type, data);

    ODEntryToBuffer(driver, self, &temp);
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

/*
static void ODCreateTPDOCom(OD_DYN *self, uint8_t num, uint32_t id, uint8_t type, uint16_t inhibit, uint16_t evtimer, CO_IF_DRV* driver)
{
    ODAddUpdate(self, CO_KEY(0x1800+num, 0, CO_UNSIGNED8 |CO_OBJ_D__R_), 0, (CO_DATA)(0x05), driver);
    ODAddUpdate(self, CO_KEY(0x1800+num, 1, CO_UNSIGNED32|CO_OBJ_DN_R_), 0, (CO_DATA)(id), driver);
    ODAddUpdate(self, CO_KEY(0x1800+num, 2, CO_UNSIGNED8 |CO_OBJ_D__R_), 0, (CO_DATA)(type), driver);
    ODAddUpdate(self, CO_KEY(0x1800+num, 3, CO_UNSIGNED16|CO_OBJ_D__RW), 0, (CO_DATA)(inhibit), driver);
    ODAddUpdate(self, CO_KEY(0x1800+num, 5, CO_UNSIGNED16|CO_OBJ_D__RW), CO_TEVENT, (CO_DATA)(evtimer), driver);
}

static void ODCreateTPDOMap(OD_DYN *self, uint8_t num, uint32_t *map, uint8_t len, CO_IF_DRV* driver)
{
    uint8_t n;

    ODAddUpdate(self, CO_KEY(0x1A00+num, 0, CO_UNSIGNED8 |CO_OBJ_D__R_), 0, (CO_DATA)(len), driver);
    for (n = 0; n < len; n++) {
        ODAddUpdate(self, CO_KEY(0x1A00+num, 1+n, CO_UNSIGNED32|CO_OBJ_D__R_), 0, (CO_DATA)(map[n]), driver);
    }
}
*/

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

#define OD_SIZE 256	//TODO: replace with better value

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
    while (matches < ODLIST_SIZE && odindex < OD_SIZE) {
	uint32_t buffer;
	driver->Nvm->Read((odindex)*size, (uint8_t*)&buffer, size);
	if (buffer == ODList[matches]) {	//if we found one of our entries, move onto the next
		matches++;
		odindex += 3;
		continue;
	}
	if (buffer == 0xFFFFFFFF) break;	//we've reached the end of NVM
	odindex++;
    }
    if (matches == ODLIST_SIZE) return;	//we have a functional minimal OD: nothing to do here



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

    /*
    uint32_t map[3];

    ODCreateTPDOCom(self, 0, CO_COBID_TPDO_DEFAULT(0), 254, 0, 0, driver);

    map[0] = CO_LINK(0x2100, 1, 32);
    map[1] = CO_LINK(0x2100, 2,  8);
    map[2] = CO_LINK(0x2100, 3,  8);
    ODCreateTPDOMap(self, 0, map, 3, driver);

    ODAddUpdate(self, CO_KEY(0x2100, 0, CO_UNSIGNED8 |CO_OBJ_D__R_), 0, (CO_DATA)(3), driver);
    ODAddUpdate(self, CO_KEY(0x2100, 1, CO_UNSIGNED32|CO_OBJ___PR_), 0, (CO_DATA)(&Obj2100_01_20), driver);
    ODAddUpdate(self, CO_KEY(0x2100, 2, CO_UNSIGNED8 |CO_OBJ___PR_), 0, (CO_DATA)(&Obj2100_02_08), driver);
    ODAddUpdate(self, CO_KEY(0x2100, 3, CO_UNSIGNED8 |CO_OBJ___PR_), CO_TASYNC, (CO_DATA)(&Obj2100_03_08), driver);
    */

    ODNvmToBuffer(driver, self);	//we shouldnt have written to NVM by now, but just in case...
    ODBufferToNvm(driver, self);
    ODClearBuffer();
}
// END OD SECTION

struct CO_NODE_SPEC_T co_node_spec = {
    0x11,	/* default Node-Id (currently arbitrary)*/
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

void	canopen_driver_init() {
#if OD == STATIC
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
			FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
	FLASH_EraseSector(8 << 3, (uint8_t)((PWR->CSR & PWR_CSR_PVDO) ? VoltageRange_2 : VoltageRange_3));
		//Note the `8 << 3`; this function doesnt bitshift the sector number
	FLASH_Lock();
#endif
	/* Clear all entries in object dictionary */
	ODInit(&AppOD, (CO_OBJ*)NVM_CHOS_ADDRESS, APP_OBJ_N);
	
	/* Setup the object dictionary during runtime */
	ODCreateDict(&AppOD, &AppDriver);
	
	/* Create a node with generated object dictionary */
	CONodeInit(&co_node, &co_node_spec);
	
	CO_ERR err = CONodeGetErr(&co_node);
	if (err != CO_ERR_NONE) {
		while(1);
	}
	CONodeStart(&co_node);
	CONmtSetMode(&co_node.Nmt, CO_OPERATIONAL);
	chVTSetI(&co_vt, MS2ST(1), co_vt_update, NULL);

}
