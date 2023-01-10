/******************************************************************************
   Copyright 2020 Embedded Office GmbH & Co. KG

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
******************************************************************************/

/******************************************************************************
* INCLUDES
******************************************************************************/

#include "co_can_vpkg.h"
//#include "vesc_c_if.h"
#include "hal.h"
#include "hw.h"

#include <string.h>

/******************************************************************************
* PRIVATE DEFINES
******************************************************************************/

/* TODO: place here your CAN controller register definitions */

/******************************************************************************
* PRIVATE FUNCTIONS
******************************************************************************/

static void    DrvCanInit   (void);
static void    DrvCanEnable (uint32_t baudrate);
static int16_t DrvCanSend   (CO_IF_FRM *frm);
static int16_t DrvCanRead   (CO_IF_FRM *frm);
static void    DrvCanReset  (void);
static void    DrvCanClose  (void);

/******************************************************************************
* PUBLIC VARIABLE
******************************************************************************/

/* TODO: rename the variable to match the naming convention:
 *   <device>CanDriver
 */
const CO_IF_CAN_DRV VescPkgCanDriver = {
    DrvCanInit,
    DrvCanEnable,
    DrvCanRead,
    DrvCanSend,
    DrvCanReset,
    DrvCanClose
};

/******************************************************************************
* PRIVATE FUNCTIONS
******************************************************************************/

static void DrvCanInit(void)
{
    /* TODO: initialize the CAN controller (don't enable communication) */
}

static void DrvCanEnable(uint32_t baudrate)
{
    (void)baudrate;

    /* TODO: set the given baudrate to the CAN controller */
    	//Handled by VESC BLDC's comm/comm_can.c
}

static int16_t DrvCanSend(CO_IF_FRM *frm)
{
    CANTxFrame txmsg;
    txmsg.IDE = CAN_IDE_STD;		// standard ID size
    txmsg.SID = frm->Identifier;
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.DLC = frm->DLC;
    memcpy(txmsg.data8, frm->Data, txmsg.DLC);
    msg_t ok = canTransmit(&HW_CAN_DEV, CAN_ANY_MAILBOX, &txmsg, TIME_INFINITE);
    switch (ok) {
	case MSG_OK:
		return 1;
	case MSG_TIMEOUT:
		return 0;
	case MSG_RESET:
		return -1;
	default:
		return -1;
    }
//    return (ok == MSG_OK);
}

static int16_t DrvCanRead (CO_IF_FRM *frm)
{
    //CANRxFrame rxmsg;
    //msg_t ok = canReceive(&HW_CAN_DEV, CAN_ANY_MAILBOX, &rxmsg, TIME_IMMEDIATE);
    
    msg_t ok = can_ring_buffer.wp != can_ring_buffer.rp ? MSG_OK : MSG_TIMEOUT;

    switch (ok) {
	case MSG_OK:
		{
		t_can_frame read_frame = can_ring_buffer.data[can_ring_buffer.rp];
    		frm->Identifier = read_frame.id;
    		frm->DLC = read_frame.len;
    		memcpy(frm->Data, read_frame.data, frm->DLC);
		can_ring_buffer.rp = (can_ring_buffer.rp + 1)%CAN_RINGBUFFER_SIZE;
		return 1;
		}
	case MSG_TIMEOUT:
		return 0;
	case MSG_RESET:
		return -1;
	default:
		return -1;
    }
//    return (ok == MSG_OK);
}

static void DrvCanReset(void)
{
    /* TODO: reset CAN controller while keeping baudrate */
}

static void DrvCanClose(void)
{
    /* TODO: remove CAN controller from CAN network */
}
