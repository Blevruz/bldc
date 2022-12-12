#ifndef	CANOPEN_DRIVER_H_
#define CANOPEN_DRIVER_H_

#include "conf_general.h"
#include "co_core.h"
#include "co_can_vpkg.h"
#include "co_nvm_chos.h"
#include "co_timer_swcycle.h"
#include "app.h"
#include "nrf_driver.h"
#include <stdint.h>
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
  CO_OBJ    *Root;
  uint32_t   Length;
  uint32_t   Used;
} OD_DYN;
extern OD_DYN AppOD;


int	canopen_driver_init(void);

void co_vt_update(void *p);

void ODEraseNvm();

int8_t ODEntryToBuffer (CO_IF_DRV* driver, OD_DYN* self, CO_OBJ* to_write);

void ODBufferToNvm(CO_IF_DRV* driver, OD_DYN* self);

void ODNvmToBuffer(CO_IF_DRV* driver, OD_DYN* self);

void ODClearBuffer(void);

void ODInit (OD_DYN *self, CO_OBJ *root, uint32_t length);

int ODAddUpdate(OD_DYN *self, uint32_t key, const CO_OBJ_TYPE *type, CO_DATA data, CO_IF_DRV* driver);

bool canopen_sid_callback(uint32_t id, uint8_t *data, uint8_t len);

uint8_t	get_canopen_ready(void);
void	set_canopen_ready(uint8_t ready);

#endif //CANOPEN_DRIVER_H_
