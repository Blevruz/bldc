#ifndef	CANOPEN_DRIVER_H_
#define CANOPEN_DRIVER_H_

#include "conf_general.h"
#include "co_core.h"
#include "co_can_vpkg.h"
#include "co_nvm_vpkg.h"
#include "co_timer_swcycle.h"
#include "app.h"
#include "mc_interface.h"
#include "timeout.h"
#include "utils_sys.h"
#include "nrf_driver.h"
#include "flash_helper.h"
#include "canopen_od_flash.h"

#include <stdint.h>
#include <string.h>

//TODO: move all canopen-related data structure to this, to aid movement to pkg
typedef struct {
	OD_BUFF obj_buff;
	CO_NODE co_node;
} co_data;

extern co_data *d;	//NOTE: not possible in pkg. need to figure out a way to get the data
			// to callbacks, e.g. in canopen_pds_objects

typedef struct {
	uint32_t id;
	uint8_t data[8];
	uint8_t len;
} t_can_frame;

#define CAN_RINGBUFFER_SIZE	32

typedef struct t_co_ring_buffer{
	t_can_frame data[CAN_RINGBUFFER_SIZE];
	uint8_t wp;	//write pos
	uint8_t rp;	//read pos
} co_ring_buffer;

extern co_ring_buffer can_ring_buffer;
extern CO_NODE_SPEC co_node_spec;
extern CO_NODE co_node;
typedef struct {
  uint32_t	Root;
  uint32_t	Length;
  uint32_t	Used;
} OD_DYN;
extern OD_DYN AppOD;


int	canopen_driver_init(co_data* d);

void co_vt_update(void *p);

#if 0
void ODEraseNvm(void);								//handled by canopen_od_flash

int8_t ODEntryToBuffer (CO_IF_DRV* driver, OD_DYN* self, CO_OBJ* to_write);	//handled by canopen_od_flash

void ODBufferToNvm(CO_IF_DRV* driver, OD_DYN* self);				//handled by canopen_od_flash

void ODNvmToBuffer(CO_IF_DRV* driver, OD_DYN* self);				//handled by canopen_od_flash

void ODClearBuffer(void);							//handled by canopen_od_flash
int ODAddUpdate(OD_DYN *self, uint32_t key, const CO_OBJ_TYPE *type, CO_DATA data, CO_IF_DRV* driver);
#endif

bool canopen_sid_callback(uint32_t id, uint8_t *data, uint8_t len);

uint8_t	get_canopen_ready(void);
void	set_canopen_ready(uint8_t ready);

#endif //CANOPEN_DRIVER_H_
