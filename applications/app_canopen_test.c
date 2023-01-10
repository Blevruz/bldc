/*
	Copyright 2019 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#define DUNE_NODE	0
#define MULTITHRUSTER	1
#define OTHER_DUNE_NODE	2

#define DUNE_OD		OTHER_DUNE_NODE
#define VCSW		1	//whether to expect a status word in the first 2 bytes of the velocity command (1) or not (0)

#ifdef USE_CANOPEN

#include "app.h"
#include "ch.h"
#include "hal.h"

// Some useful includes
#include "co_core.h"
#include "mc_interface.h"
#include "utils_math.h"
#include "encoder/encoder.h"
#include "terminal.h"
#include "comm_can.h"
#include "hw.h"
#include "commands.h"
#include "timeout.h"
//#include "canopen_driver.h"
#include "canopen_pds_fsa.h"
#include "canopen_pds_objects.h"	

//#include "TEMP_ring_buffer.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

/*
 * 500KBaud, automatic wakeup, automatic recover
 * from abort mode.
 * See section 22.7.7 on the STM32 reference manual.
 * (copy-pasted from comm/comm_can.c)
 */
static CANConfig cancfg = {
		CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
		CAN_BTR_SJW(3) | CAN_BTR_TS2(2) |
		CAN_BTR_TS1(9) | CAN_BTR_BRP(5)
};

static void app_cot_stop_can(void) {
#ifdef HW_CAN2_DEV
	canStop(&CAND1);
	canStop(&CAND2);
#else
	canStop(&HW_CAN_DEV);
#endif
}

static void app_cot_start_can(void) {
#ifdef HW_CAN2_DEV
	canStart(&CAND1, &cancfg);
	canStart(&CAND2, &cancfg);
#else
	canStart(&HW_CAN_DEV, &cancfg);
#endif
}

// User CANopen object type

uint32_t RPMFreq = 0;
float DutyCommand = 0;
float CurrentCommand = 0;
static float mc_enc_ratio = 0;

#if DUNE_OD == DUNE_NODE
int32_t RPMMeasurement = 0;
int8_t FetTempMeasurement = 0;
float MotorTempMeasurement = 0;
int16_t InputCurrentMeasurement = 0;
#elif DUNE_OD == MULTITHRUSTER
int16_t InputCurrentMeasurement = 0;
int32_t RPMMeasurement = 0;
#elif DUNE_OD == OTHER_DUNE_NODE
int16_t RPMMeasurement = 0;
int8_t FetTempMeasurement = 0;
float MotorTempMeasurement = 0;
uint32_t VdcBus_kV = 0;
uint32_t InputCurrentMeasurement = 0;
#endif

// Threads
static THD_FUNCTION(cot_thread, arg);
static THD_WORKING_AREA(cot_thread_wa, 1024);

// Private functions
static void pwm_callback(void);
static void terminal_test(int argc, const char **argv);
static void terminal_set_node_id(int argc, const char **argv);
static void terminal_update_od(int argc, const char **argv);

// Private variables
static volatile bool stop_now = true; static volatile bool is_running = false;
static volatile bool do_rpm_callback = true;

#define MAGIC_FLOAT_TO_UINT32	0x1000000

void RPM_TPdo_Callback(void* arg) {
	(void)arg;
	if (!do_rpm_callback) return;
#if DUNE_OD == DUNE_NODE
	RPMMeasurement = mc_interface_get_rpm() / mc_enc_ratio;
	RPMMeasurement = (RPMMeasurement&0x000000FF<<24) | (RPMMeasurement&0x0000FF00<<8) | (RPMMeasurement&0x00FF0000>>8) | (RPMMeasurement&0xFF000000>>24);
	COTPdoTrigPdo((CO_TPDO*)(&co_node.TPdo), 0);

	FetTempMeasurement = mc_interface_temp_fet_filtered();
	MotorTempMeasurement = mc_interface_temp_motor_filtered();
	COTPdoTrigPdo((CO_TPDO*)(&co_node.TPdo), 1);

	InputCurrentMeasurement = (int16_t)(mc_interface_get_tot_current_in_filtered() * 1e1);
	COTPdoTrigPdo((CO_TPDO*)(&co_node.TPdo), 2);

#elif DUNE_OD == OTHER_DUNE_NODE
	RPMMeasurement = mc_interface_get_rpm() / mc_enc_ratio;
	RPMMeasurement = (RPMMeasurement&0x00FF<<8) | (RPMMeasurement&0xFF00>>8);
	COTPdoTrigPdo((CO_TPDO*)(&co_node.TPdo), 0);

	FetTempMeasurement = mc_interface_temp_fet_filtered();
	MotorTempMeasurement = mc_interface_temp_motor_filtered();
	COTPdoTrigPdo((CO_TPDO*)(&co_node.TPdo), 1);

	VdcBus_kV = mc_interface_get_input_voltage_filtered() * MAGIC_FLOAT_TO_UINT32 / 1000;	//divided by 1000 because this message is in kilovolts
	InputCurrentMeasurement = (uint32_t)(mc_interface_get_tot_current_in_filtered() * MAGIC_FLOAT_TO_UINT32);
	COTPdoTrigPdo((CO_TPDO*)(&co_node.TPdo), 2);

#elif DUNE_OD == MULTITHRUSTER
	RPMMeasurement = mc_interface_get_rpm();
	RPMMeasurement = (RPMMeasurement&0x000000FF<<24) | (RPMMeasurement&0x0000FF00<<8) | (RPMMeasurement&0x00FF0000>>8) | (RPMMeasurement&0xFF000000>>24);
	InputCurrentMeasurement = (int16_t)(mc_interface_get_tot_current_in_filtered() * 1e1);
	COTPdoTrigPdo((CO_TPDO*)(&co_node.TPdo), 0);
#endif
}

void update_od(void) {

	ODNvmToBuffer(co_node_spec.Drv, &AppOD);
	
	app_cot_stop_can();	// stopping can to avoid issues caused by writing to nvm
	ODBufferToNvm(co_node_spec.Drv, &AppOD);
	app_cot_start_can();	// starting it back up
	ODClearBuffer();

	CORPdoInit(co_node.RPdo, &co_node);
	COTPdoInit(co_node.TPdo, &co_node);
}

// Called when the canopen_test application is started. Start our
// threads here and set up callbacks.
void app_canopen_test_start(void) {

	if (FsaAttemptTransition(0) == -1) for(;;);	//try startup transition; loop is for ease of debugging
	//canopen_driver_init();	//moved to canopen_pds_fsa.c

	mc_interface_set_pwm_callback(pwm_callback);

	mc_configuration mc_config;
	conf_general_read_mc_configuration(&mc_config, false);
	mc_enc_ratio = mc_config.foc_encoder_ratio;

	if (!get_canopen_ready()) {
		// TPDO
	
#if DUNE_OD == DUNE_NODE
		// TPDO #1: 2 bytes unsigned int status word (unknown), 2 bytes signed int RPM (, 3 bytes unsigned ints various status things)
		ODAddUpdate(&AppOD, CO_KEY(0x1800, 0, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0x05), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1800, 1, CO_UNSIGNED32	| CO_OBJ_DN_R_), 0, (CO_DATA)(0x00000180 | CO_TPDO_COBID_REMOTE), co_node_spec.Drv);	//see evobldc EDS
		ODAddUpdate(&AppOD, CO_KEY(0x1800, 2, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0xFE), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1800, 3, CO_UNSIGNED16	| CO_OBJ_D__R_), 0, (CO_DATA)(0), co_node_spec.Drv);
					//subindex 4 reserved
		ODAddUpdate(&AppOD, CO_KEY(0x1800, 5, CO_UNSIGNED16	| CO_OBJ_D__R_), 0, (CO_DATA)(0), co_node_spec.Drv);
	
		ODAddUpdate(&AppOD, CO_KEY(0x3116, 0, CO_UNSIGNED8	| CO_OBJ_D__RW), 0, (CO_DATA)(0xAA), co_node_spec.Drv);	//status word
		ODAddUpdate(&AppOD, CO_KEY(0x6041, 0, CO_UNSIGNED16	| CO_OBJ____R_), CO_T_STATUS_WORD, (CO_DATA)(&StatusWord), co_node_spec.Drv);	//status word
		
		ODAddUpdate(&AppOD, CO_KEY(0x6044, 0, CO_UNSIGNED32	| CO_OBJ____RW), 0, (CO_DATA)(&RPMMeasurement), co_node_spec.Drv);	//TODO: replace with a type that handles the speed measurement on read
	
		ODAddUpdate(&AppOD, CO_KEY(0x1A00, 0, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(2), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1A00, 1, CO_UNSIGNED32	| CO_OBJ_D__RW), 0, CO_LINK(0x6044, 0, 32), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1A00, 2, CO_UNSIGNED32	| CO_OBJ_D__R_), 0, CO_LINK(0x6041, 0, 16), co_node_spec.Drv);
		/*
		ODAddUpdate(&AppOD, CO_KEY(0x1A00, 2, CO_UNSIGNED32	| CO_OBJ_D__R_), 0, CO_LINK(0x3116, 0, 8), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1A00, 3, CO_UNSIGNED32	| CO_OBJ_D__R_), 0, CO_LINK(0x3116, 0, 8), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1A00, 4, CO_UNSIGNED32	| CO_OBJ_D__R_), 0, CO_LINK(0x3116, 0, 8), co_node_spec.Drv);
		*/
	
		// TPDO #2: goal: 1 byte humidity (unknown), 1 bytes FET temp, 4 bytes motor temp (float)
		ODAddUpdate(&AppOD, CO_KEY(0x1801, 0, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0x05), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1801, 1, CO_UNSIGNED32	| CO_OBJ_DN_R_), 0, (CO_DATA)(0x00000380 | CO_TPDO_COBID_REMOTE), co_node_spec.Drv);	//see evobldc EDS
		ODAddUpdate(&AppOD, CO_KEY(0x1801, 2, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0xFE), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1801, 3, CO_UNSIGNED16	| CO_OBJ_D__R_), 0, (CO_DATA)(0), co_node_spec.Drv);
					//subindex 4 reserved
		ODAddUpdate(&AppOD, CO_KEY(0x1801, 5, CO_UNSIGNED16	| CO_OBJ_D__R_), 0, (CO_DATA)(0), co_node_spec.Drv);
	
		ODAddUpdate(&AppOD, CO_KEY(0x3102, 0, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0x03), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x3102, 1, CO_UNSIGNED32	| CO_OBJ_D__RW), 0, (CO_DATA)(0), co_node_spec.Drv);	//Pressure (dummy)
		ODAddUpdate(&AppOD, CO_KEY(0x3102, 2, CO_UNSIGNED8	| CO_OBJ____RW), 0, (CO_DATA)(&FetTempMeasurement), co_node_spec.Drv);	//Temperature mosfet, TODO: replace with at type that handles the temp measurement on read
		ODAddUpdate(&AppOD, CO_KEY(0x3102, 3, CO_UNSIGNED32	| CO_OBJ____RW), 0, (CO_DATA)(&MotorTempMeasurement), co_node_spec.Drv);//Temperature motor, TODO: replace with at type that handles the temp measurement on read
	
																		
		ODAddUpdate(&AppOD, CO_KEY(0x1A01, 0, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0x03), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1A01, 1, CO_UNSIGNED32	| CO_OBJ_D__RW), 0, CO_LINK(0x3116, 0, 8), co_node_spec.Drv);	// Humidity (dummy)
		ODAddUpdate(&AppOD, CO_KEY(0x1A01, 2, CO_UNSIGNED32	| CO_OBJ_D__RW), 0, CO_LINK(0x3102, 2, 8), co_node_spec.Drv);	// Fet temperature
		ODAddUpdate(&AppOD, CO_KEY(0x1A01, 3, CO_UNSIGNED32	| CO_OBJ_D__RW), 0, CO_LINK(0x3102, 3, 32), co_node_spec.Drv);	// Motor Temperature
	
		ODAddUpdate(&AppOD, CO_KEY(0x6042, 0, CO_UNSIGNED16	| CO_OBJ_D__R_), 0, (CO_DATA)(0x01), co_node_spec.Drv);
	
	
		// TPDO #3: 16 bit flag, 16 bit voltage, 16 bit current in, 16 bit current out (Amps*1000)
		ODAddUpdate(&AppOD, CO_KEY(0x1802, 0, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0x05), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1802, 1, CO_UNSIGNED32	| CO_OBJ_DN_R_), 0, (CO_DATA)(0x00000186 | CO_TPDO_COBID_REMOTE), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1802, 2, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0xFE), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1802, 3, CO_UNSIGNED16	| CO_OBJ_D__R_), 0, (CO_DATA)(0), co_node_spec.Drv);
					//subindex 4 reserved
		ODAddUpdate(&AppOD, CO_KEY(0x1802, 5, CO_UNSIGNED16	| CO_OBJ_D__R_), 0, (CO_DATA)(0), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x6078, 0, CO_UNSIGNED16	| CO_OBJ____R_), 0, (CO_DATA)(&InputCurrentMeasurement), co_node_spec.Drv);	// Input current
		ODAddUpdate(&AppOD, CO_KEY(0x1A02, 0, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0x04), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1A02, 1, CO_UNSIGNED32	| CO_OBJ_D__R_), 0, CO_LINK(0x6041, 0, 16), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1A02, 2, CO_UNSIGNED32	| CO_OBJ_D__R_), 0, CO_LINK(0x6041, 0, 16), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1A02, 3, CO_UNSIGNED32	| CO_OBJ_D__RW), 0, CO_LINK(0x6078, 0, 16), co_node_spec.Drv);	// Input current (mapping)
		ODAddUpdate(&AppOD, CO_KEY(0x1A02, 4, CO_UNSIGNED32	| CO_OBJ_D__R_), 0, CO_LINK(0x6041, 0, 16), co_node_spec.Drv);
	
#elif DUNE_OD == MULTITHRUSTER
		// TPDO #1: 2 bytes unsigned int status word (unknown), 2 bytes signed int current, 4 bytes signed int RPM
		ODAddUpdate(&AppOD, CO_KEY(0x1800, 0, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0x05), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1800, 1, CO_UNSIGNED32	| CO_OBJ_DN_R_), 0, (CO_DATA)(0x00000180 | CO_TPDO_COBID_REMOTE), co_node_spec.Drv);	//see evobldc EDS
		ODAddUpdate(&AppOD, CO_KEY(0x1800, 2, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0xFE), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1800, 3, CO_UNSIGNED16	| CO_OBJ_D__R_), 0, (CO_DATA)(0), co_node_spec.Drv);
					//subindex 4 reserved
		ODAddUpdate(&AppOD, CO_KEY(0x1800, 5, CO_UNSIGNED16	| CO_OBJ_D__R_), 0, (CO_DATA)(0), co_node_spec.Drv);
	
		ODAddUpdate(&AppOD, CO_KEY(0x6041, 0, CO_UNSIGNED16	| CO_OBJ____R_), CO_T_STATUS_WORD, (CO_DATA)(&StatusWord), co_node_spec.Drv);	//status word
		
		ODAddUpdate(&AppOD, CO_KEY(0x6044, 0, CO_UNSIGNED32	| CO_OBJ____RW), 0, (CO_DATA)(&RPMMeasurement), co_node_spec.Drv);	//TODO: replace with a type that handles the speed measurement on read
		ODAddUpdate(&AppOD, CO_KEY(0x6078, 0, CO_UNSIGNED16	| CO_OBJ____RW), 0, (CO_DATA)(&InputCurrentMeasurement), co_node_spec.Drv);	// Input current
	
		ODAddUpdate(&AppOD, CO_KEY(0x1A00, 0, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(3), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1A00, 1, CO_UNSIGNED32	| CO_OBJ_D__R_), 0, CO_LINK(0x6041, 0, 16), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1A00, 2, CO_UNSIGNED32	| CO_OBJ_D__RW), 0, CO_LINK(0x6078, 0, 16), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1A00, 3, CO_UNSIGNED32	| CO_OBJ_D__RW), 0, CO_LINK(0x6044, 0, 32), co_node_spec.Drv);
	
#elif DUNE_OD == OTHER_DUNE_NODE
		// TPDO #1: 2 bytes unsigned int status word (unknown), 2 bytes signed int RPM, 3 bytes status words
		ODAddUpdate(&AppOD, CO_KEY(0x1800, 0, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0x05), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1800, 1, CO_UNSIGNED32	| CO_OBJ_DN_R_), 0, (CO_DATA)(0x00000180 | CO_TPDO_COBID_REMOTE), co_node_spec.Drv);	//see evobldc EDS
		ODAddUpdate(&AppOD, CO_KEY(0x1800, 2, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0xFE), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1800, 3, CO_UNSIGNED16	| CO_OBJ_D__R_), 0, (CO_DATA)(0), co_node_spec.Drv);
					//subindex 4 reserved
		ODAddUpdate(&AppOD, CO_KEY(0x1800, 5, CO_UNSIGNED16	| CO_OBJ_D__R_), 0, (CO_DATA)(0), co_node_spec.Drv);
	
		ODAddUpdate(&AppOD, CO_KEY(0x3116, 0, CO_UNSIGNED8	| CO_OBJ_D__RW), 0, (CO_DATA)(0xAA), co_node_spec.Drv);		//status word
		ODAddUpdate(&AppOD, CO_KEY(0x6041, 0, CO_UNSIGNED16	| CO_OBJ____R_), CO_T_STATUS_WORD, (CO_DATA)(&StatusWord), co_node_spec.Drv);	//status word
		
		ODAddUpdate(&AppOD, CO_KEY(0x6044, 0, CO_UNSIGNED16	| CO_OBJ____RW), 0, (CO_DATA)(&RPMMeasurement), co_node_spec.Drv);	//TODO: replace with a type that handles the speed measurement on read
	
		ODAddUpdate(&AppOD, CO_KEY(0x1A00, 0, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(4), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1A00, 1, CO_UNSIGNED32	| CO_OBJ_D__R_), 0, CO_LINK(0x6041, 0, 16), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1A00, 2, CO_UNSIGNED32	| CO_OBJ_D__RW), 0, CO_LINK(0x6044, 0, 16), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1A00, 3, CO_UNSIGNED32	| CO_OBJ_D__RW), 0, CO_LINK(0x6041, 0, 16), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1A00, 4, CO_UNSIGNED32	| CO_OBJ_D__RW), 0, CO_LINK(0x3116, 0, 8), co_node_spec.Drv);
	
		// TPDO #2: 1 byte unsigned int humidity, 1 byte signed int fet temperature, 4 bytes float motor temp
		ODAddUpdate(&AppOD, CO_KEY(0x1801, 0, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0x05), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1801, 1, CO_UNSIGNED32	| CO_OBJ_DN_R_), 0, (CO_DATA)(0x00000380 | CO_TPDO_COBID_REMOTE), co_node_spec.Drv);	//see evobldc EDS
		ODAddUpdate(&AppOD, CO_KEY(0x1801, 2, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0xFE), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1801, 3, CO_UNSIGNED16	| CO_OBJ_D__R_), 0, (CO_DATA)(0), co_node_spec.Drv);
					//subindex 4 reserved
		ODAddUpdate(&AppOD, CO_KEY(0x1801, 5, CO_UNSIGNED16	| CO_OBJ_D__R_), 0, (CO_DATA)(0), co_node_spec.Drv);
	
		ODAddUpdate(&AppOD, CO_KEY(0x3102, 0, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0x03), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x3102, 1, CO_UNSIGNED32	| CO_OBJ_D__RW), 0, (CO_DATA)(0), co_node_spec.Drv);	//Pressure (dummy)
		ODAddUpdate(&AppOD, CO_KEY(0x3102, 2, CO_UNSIGNED8	| CO_OBJ____RW), 0, (CO_DATA)(&FetTempMeasurement), co_node_spec.Drv);	//Temperature mosfet, TODO: replace with at type that handles the temp measurement on read
		ODAddUpdate(&AppOD, CO_KEY(0x3102, 3, CO_UNSIGNED32	| CO_OBJ____RW), 0, (CO_DATA)(&MotorTempMeasurement), co_node_spec.Drv);//Temperature motor, TODO: replace with at type that handles the temp measurement on read
	
		ODAddUpdate(&AppOD, CO_KEY(0x1A01, 0, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(3), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1A01, 1, CO_UNSIGNED32	| CO_OBJ_D__RW), 0, CO_LINK(0x3116, 0, 8), co_node_spec.Drv);	// Humidity (dummy)
		ODAddUpdate(&AppOD, CO_KEY(0x1A01, 2, CO_UNSIGNED32	| CO_OBJ_D__RW), 0, CO_LINK(0x3102, 2, 8), co_node_spec.Drv);	// Fet temperature
		ODAddUpdate(&AppOD, CO_KEY(0x1A01, 3, CO_UNSIGNED32	| CO_OBJ_D__RW), 0, CO_LINK(0x3102, 3, 32), co_node_spec.Drv);	// Motor Temperature
	
		// TPDO #3: 4 bytes unsigned int motor voltage, 4 bytes unsigned int input current
		ODAddUpdate(&AppOD, CO_KEY(0x1802, 0, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0x05), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1802, 1, CO_UNSIGNED32	| CO_OBJ_DN_R_), 0, (CO_DATA)(0x000001C0 | CO_TPDO_COBID_REMOTE), co_node_spec.Drv);	//see evobldc EDS
		ODAddUpdate(&AppOD, CO_KEY(0x1802, 2, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0xFE), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1802, 3, CO_UNSIGNED16	| CO_OBJ_D__R_), 0, (CO_DATA)(0), co_node_spec.Drv);
					//subindex 4 reserved
		ODAddUpdate(&AppOD, CO_KEY(0x1802, 5, CO_UNSIGNED16	| CO_OBJ_D__R_), 0, (CO_DATA)(0), co_node_spec.Drv);
		
		ODAddUpdate(&AppOD, CO_KEY(0x311C, 0x02, CO_UNSIGNED32	| CO_OBJ____RW), 0, (CO_DATA)(&InputCurrentMeasurement), co_node_spec.Drv);	// Input current
		ODAddUpdate(&AppOD, CO_KEY(0x3116, 0x2F, CO_UNSIGNED32	| CO_OBJ____RW), 0, (CO_DATA)(&VdcBus_kV), co_node_spec.Drv);	// Motor Voltage
	
		ODAddUpdate(&AppOD, CO_KEY(0x1A02, 0, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(2), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1A02, 1, CO_UNSIGNED32	| CO_OBJ_D__RW), 0, CO_LINK(0x311C, 0x02, 32), co_node_spec.Drv);	// Input Current
		ODAddUpdate(&AppOD, CO_KEY(0x1A02, 2, CO_UNSIGNED32	| CO_OBJ_D__RW), 0, CO_LINK(0x3116, 0x2F, 32), co_node_spec.Drv);	// Motor Voltage
#endif
	
		// RPDO #1: 2 byte int control word, 2 bytes int set RPM
		ODAddUpdate(&AppOD, CO_KEY(0x1400, 0, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0x02), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1400, 1, CO_UNSIGNED32	| CO_OBJ_DN_R_), 0, (CO_DATA)(0x00000200), co_node_spec.Drv);	//see evobldc EDS
		ODAddUpdate(&AppOD, CO_KEY(0x1400, 2, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0xFE), co_node_spec.Drv);
	
		ODAddUpdate(&AppOD, CO_KEY(0x6040, 0, CO_UNSIGNED16	| CO_OBJ____RW), CO_T_CONTROL_WORD, (CO_DATA)(&ControlWord), co_node_spec.Drv);

		ODAddUpdate(&AppOD, CO_KEY(0x6042, 0, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0x01), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x6042, 1, CO_UNSIGNED16	| CO_OBJ____RW), CO_T_RPM_COMMAND, (CO_DATA)(&RPMFreq), co_node_spec.Drv);
	
		ODAddUpdate(&AppOD, CO_KEY(0x1600, 0, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0x02), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1600, 1, CO_UNSIGNED32	| CO_OBJ_D__RW), 0, CO_LINK(0x6040, 0, 16), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1600, 2, CO_UNSIGNED32	| CO_OBJ_D__RW), 0, CO_LINK(0x6042, 1, 16), co_node_spec.Drv);
		
		// RPDO #2: 1 byte int do readings
		ODAddUpdate(&AppOD, CO_KEY(0x1401, 0, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0x02), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1401, 1, CO_UNSIGNED32	| CO_OBJ_DN_R_), 0, (CO_DATA)(0x00000100), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1401, 2, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0xFE), co_node_spec.Drv);
	
		ODAddUpdate(&AppOD, CO_KEY(0x4444, 0, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0x01), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x4444, 1, CO_UNSIGNED8	| CO_OBJ____RW), 0, (CO_DATA)(&do_rpm_callback), co_node_spec.Drv);
	
		ODAddUpdate(&AppOD, CO_KEY(0x1601, 0, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0x01), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1601, 1, CO_UNSIGNED32	| CO_OBJ_D__RW), 0, CO_LINK(0x4444, 1, 8), co_node_spec.Drv);
		
		// RPDO #3: 2 byte int control word, 2 byte int set PWM duty cycle (fixed point, multiplied by 0x7FFF)
		ODAddUpdate(&AppOD, CO_KEY(0x1402, 0, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0x02), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1402, 1, CO_UNSIGNED32	| CO_OBJ_DN_R_), 0, (CO_DATA)(0x00000300), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1402, 2, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0xFE), co_node_spec.Drv);
	
		ODAddUpdate(&AppOD, CO_KEY(0x6050, 0, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0x01), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x6050, 1, CO_UNSIGNED16	| CO_OBJ____RW), CO_T_DUTY_COMMAND, (CO_DATA)(&DutyCommand), co_node_spec.Drv);
	
		ODAddUpdate(&AppOD, CO_KEY(0x1602, 0, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0x02), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1602, 1, CO_UNSIGNED32	| CO_OBJ_D__RW), 0, CO_LINK(0x6040, 0, 16), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1602, 2, CO_UNSIGNED32	| CO_OBJ_D__RW), 0, CO_LINK(0x6050, 1, 16), co_node_spec.Drv);
	
		// RPDO #4: 2 byte int control word, 2 byte int set current (fixed point in A, multiplied by 0xFF)
		ODAddUpdate(&AppOD, CO_KEY(0x1403, 0, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0x02), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1403, 1, CO_UNSIGNED32	| CO_OBJ_DN_R_), 0, (CO_DATA)(0x00000400), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1403, 2, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0xFE), co_node_spec.Drv);
	
		ODAddUpdate(&AppOD, CO_KEY(0x6060, 0, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0x01), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x6060, 1, CO_UNSIGNED16	| CO_OBJ____RW), CO_T_CURRENT_COMMAND, (CO_DATA)(&CurrentCommand), co_node_spec.Drv);
	
		ODAddUpdate(&AppOD, CO_KEY(0x1603, 0, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0x02), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1603, 1, CO_UNSIGNED32	| CO_OBJ_D__RW), 0, CO_LINK(0x6040, 0, 16), co_node_spec.Drv);
		ODAddUpdate(&AppOD, CO_KEY(0x1603, 2, CO_UNSIGNED32	| CO_OBJ_D__RW), 0, CO_LINK(0x6060, 1, 16), co_node_spec.Drv);
	
		stop_now = false;
		chThdCreateStatic(cot_thread_wa, sizeof(cot_thread_wa),
				NORMALPRIO, cot_thread, NULL);
		
		
		if (FsaAttemptTransition(1) == -1) for(;;);	//try startup transition; loop is for ease of debugging
		//comm_can_set_sid_rx_callback(canopen_sid_callback);
		update_od();
	}

	// Terminal commands for the VESC Tool terminal can be registered.
	terminal_register_command_callback(
			"canopen_test_cmd",
			"Print the number d",
			"[d]",
			terminal_test);

	terminal_register_command_callback(
			"set_node_id",
			"Set the device's CANopen Node ID",
			"[d]",
			terminal_set_node_id);

	terminal_register_command_callback(
			"update_od",
			"Reloads the device's CANopen object dictionary",
			"",
			terminal_update_od);

	COTmrCreate(	&(co_node.Tmr),
			100,
			100,
			RPM_TPdo_Callback,
			0);

	set_canopen_ready(1);
}

// Called when the canopen_test application is stopped. Stop our threads
// and release callbacks.
void app_canopen_test_stop(void) {
	mc_interface_set_pwm_callback(0);
	terminal_unregister_callback(terminal_test);

	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}
}

void app_canopen_test_configure(app_configuration *conf) {	//TODO
	(void)conf;
	//co_node_spec.NodeId = conf->controller_id;
}

static THD_FUNCTION(cot_thread, arg) {
	(void)arg;

	chRegSetThreadName("App Canopen Test");

	is_running = true;

	for(;;) {
		// Check if it is time to stop.
		if (stop_now) {
			is_running = false;
			return;
		}

		// Run your logic here. A lot of functionality is available in mc_interface.h.
		chThdSleepMilliseconds(10);
	}
}

static void pwm_callback(void) {
	// Called for every control iteration in interrupt context.
}

// Callback function for the terminal command with arguments.
static void terminal_test(int argc, const char **argv) {
	if (argc == 2) {
		int d = -1;
		sscanf(argv[1], "%d", &d);

		commands_printf("You have entered %d", d);

		// For example, read the ADC inputs on the COMM header.
		commands_printf("ADC1: %.2f V ADC2: %.2f V",
				(double)ADC_VOLTS(ADC_IND_EXT), (double)ADC_VOLTS(ADC_IND_EXT2));
	} else {
		commands_printf("This command requires one argument.\n");
	}
}

// Callback function to change the NodeID (NOTE: Doesn't update object dictionary!)
static void terminal_set_node_id(int argc, const char **argv) {
	if (argc == 2) {
		int d = -1;
		sscanf(argv[1], "%d", &d);

		co_node.NodeId = d;

		commands_printf("Node ID set to %d", d);
	} else {
		commands_printf("Usage: %s [integer]\n", argv[0]);
	}
}

// Callback function to update the object dictionary
static void terminal_update_od(int argc, const char **argv) {
	(void)argc;
	(void)argv;
	commands_printf("Starting OD rewrite, may take some time... ");

	update_od();

	commands_printf("OD rewrite done!\n");
}
#undef VCSW
#endif //USE_CANOPEN
