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

#define DUNE_OD		DUNE_NODE

#ifdef USE_CANOPEN

#include "app.h"
#include "ch.h"
#include "hal.h"

// Some useful includes
#include "co_core.h"
#include "mc_interface.h"
#include "mcpwm_foc.h"
#include "utils_math.h"
#include "encoder/encoder.h"
#include "terminal.h"
#include "comm_can.h"
#include "hw.h"
#include "commands.h"
#include "timeout.h"
#include "canopen_driver.h"

#include <math.h>
#include <string.h>
#include <stdio.h>


// User CANopen object type

uint32_t ERPMSize (CO_OBJ *obj, CO_NODE *node, uint32_t width) {
	(void)obj; (void)node; (void)width;
	return 4;
}
CO_ERR   ERPMInit (CO_OBJ *obj, CO_NODE *node) {
	(void)node;
	obj->Data = 0;
	return CO_ERR_NONE;
}
CO_ERR   ERPMRead (CO_OBJ *obj, CO_NODE *node, void *buffer, uint32_t size) {
	(void)node;
	if (size < 4)
		return CO_ERR_OBJ_SIZE;
	*(float*)buffer = *(float*)(obj->Data);
	return CO_ERR_NONE;
}
CO_ERR   ERPMWrite(CO_OBJ *obj, CO_NODE *node, void *buffer, uint32_t size) {
	(void)node;
	if (size != 4)
		return CO_ERR_OBJ_SIZE;
	uint32_t o_data = 0;
	/*
	for (int i = 0; i < 4; i++)
		o_data += ((uint8_t*)buffer)[3-i] << i*8;
	*/
	o_data = *(uint32_t*)buffer;
//	mcpwm_foc_beep(*(float*)obj->Data, 1.0f, 0.5f);
	mc_interface_set_pid_speed((int32_t)o_data);//motor->m_conf->foc_encoder_ratio);
	if (o_data == 0)
		mc_interface_set_current_rel(0.0);
	timeout_reset();
	*(uint32_t*)obj->Data = o_data;
	return CO_ERR_NONE;
}
CO_ERR   ERPMReset(CO_OBJ *obj, CO_NODE *node, uint32_t para) {
	(void)node;
	*(float*)obj->Data = (float)para;
	return CO_ERR_NONE;
}

CO_OBJ_TYPE COTERPM = {
	ERPMSize,
	0,
	ERPMRead,
	ERPMWrite/*,
	ERPMReset*/
};

float ERPMFreq = 0;
int32_t ERPMMeasurement = 0;
int8_t FetTempMeasurement = 0;
float MotorTempMeasurement = 0;
int16_t InputCurrentMeasurement = 0;

#define CO_TERPM ((CO_OBJ_TYPE*)&COTERPM)

// Threads
static THD_FUNCTION(cot_thread, arg);
static THD_WORKING_AREA(cot_thread_wa, 1024);

// Private functions
static void pwm_callback(void);
static void terminal_test(int argc, const char **argv);

// Private variables
static volatile bool stop_now = true; static volatile bool is_running = false;

void ERPM_TPdo_Callback(void* arg) {
#if DUNE_OD == DUNE_NODE
	(void)arg;
	ERPMMeasurement = mcpwm_foc_get_rpm();//mc_interface_get_rpm();
	ERPMMeasurement = (ERPMMeasurement&0x000000FF<<24) | (ERPMMeasurement&0x0000FF00<<8) | (ERPMMeasurement&0x00FF0000>>8) | (ERPMMeasurement&0xFF000000>>24);
	COTPdoTrigPdo((CO_TPDO*)(&co_node.TPdo), 0);

	FetTempMeasurement = mc_interface_temp_fet_filtered();
	MotorTempMeasurement = mc_interface_temp_motor_filtered();
	COTPdoTrigPdo((CO_TPDO*)(&co_node.TPdo), 1);

	InputCurrentMeasurement = (int16_t)(mc_interface_get_tot_current_in_filtered() * 1e1);
	COTPdoTrigPdo((CO_TPDO*)(&co_node.TPdo), 2);
#elif DUNE_OD == MULTITHRUSTER
	(void)arg;
	ERPMMeasurement = mcpwm_foc_get_rpm();//mc_interface_get_rpm();
	ERPMMeasurement = (ERPMMeasurement&0x000000FF<<24) | (ERPMMeasurement&0x0000FF00<<8) | (ERPMMeasurement&0x00FF0000>>8) | (ERPMMeasurement&0xFF000000>>24);
	InputCurrentMeasurement = (int16_t)(mc_interface_get_tot_current_in_filtered() * 1e1);
	COTPdoTrigPdo((CO_TPDO*)(&co_node.TPdo), 0);
#endif
}

// Called when the canopen_test application is started. Start our
// threads here and set up callbacks.
void app_canopen_test_start(void) {
	mc_interface_set_pwm_callback(pwm_callback);

	// TPDO
	
#if DUNE_OD == DUNE_NODE
	// TPDO #1: 2 bytes unsigned int status word (unknown), r bytes signed int (E?)RPM (, 3 bytes unsigned ints various status things)
	ODAddUpdate(&AppOD, CO_KEY(0x1800, 0, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0x05), co_node_spec.Drv);
	ODAddUpdate(&AppOD, CO_KEY(0x1800, 1, CO_UNSIGNED32	| CO_OBJ_DN_R_), 0, (CO_DATA)(0x00000180 | CO_TPDO_COBID_REMOTE), co_node_spec.Drv);	//see evobldc EDS
	ODAddUpdate(&AppOD, CO_KEY(0x1800, 2, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0xFE), co_node_spec.Drv);
	ODAddUpdate(&AppOD, CO_KEY(0x1800, 3, CO_UNSIGNED16	| CO_OBJ_D__R_), 0, (CO_DATA)(0), co_node_spec.Drv);
				//subindex 4 reserved
	ODAddUpdate(&AppOD, CO_KEY(0x1800, 5, CO_UNSIGNED16	| CO_OBJ_D__R_), 0, (CO_DATA)(0), co_node_spec.Drv);

	ODAddUpdate(&AppOD, CO_KEY(0x3116, 0, CO_UNSIGNED8	| CO_OBJ_D__RW), 0, (CO_DATA)(0xAA), co_node_spec.Drv);	//status word
	ODAddUpdate(&AppOD, CO_KEY(0x6041, 0, CO_UNSIGNED16	| CO_OBJ_D__RW), 0, (CO_DATA)(0xAAAA), co_node_spec.Drv);	//status word
	
	ODAddUpdate(&AppOD, CO_KEY(0x6044, 0, CO_UNSIGNED32	| CO_OBJ____RW), 0, (CO_DATA)(&ERPMMeasurement), co_node_spec.Drv);	//TODO: replace with a type that handles the speed measurement on read

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

	ODAddUpdate(&AppOD, CO_KEY(0x6041, 0, CO_UNSIGNED16	| CO_OBJ_D__R_), 0, (CO_DATA)(0xAAAA), co_node_spec.Drv);	//status word
	
	ODAddUpdate(&AppOD, CO_KEY(0x6044, 0, CO_UNSIGNED32	| CO_OBJ____RW), 0, (CO_DATA)(&ERPMMeasurement), co_node_spec.Drv);	//TODO: replace with a type that handles the speed measurement on read
	ODAddUpdate(&AppOD, CO_KEY(0x6078, 0, CO_UNSIGNED16	| CO_OBJ____RW), 0, (CO_DATA)(&InputCurrentMeasurement), co_node_spec.Drv);	// Input current

	ODAddUpdate(&AppOD, CO_KEY(0x1A00, 0, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(3), co_node_spec.Drv);
	ODAddUpdate(&AppOD, CO_KEY(0x1A00, 1, CO_UNSIGNED32	| CO_OBJ_D__R_), 0, CO_LINK(0x6041, 0, 16), co_node_spec.Drv);
	ODAddUpdate(&AppOD, CO_KEY(0x1A00, 2, CO_UNSIGNED32	| CO_OBJ_D__RW), 0, CO_LINK(0x6078, 0, 16), co_node_spec.Drv);
	ODAddUpdate(&AppOD, CO_KEY(0x1A00, 3, CO_UNSIGNED32	| CO_OBJ_D__RW), 0, CO_LINK(0x6044, 0, 32), co_node_spec.Drv);

#endif
	// RPDO
	ODAddUpdate(&AppOD, CO_KEY(0x1400, 0, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0x02), co_node_spec.Drv);
	ODAddUpdate(&AppOD, CO_KEY(0x1400, 1, CO_UNSIGNED32	| CO_OBJ_DN_R_), 0, (CO_DATA)(0x00000200), co_node_spec.Drv);	//see evobldc EDS
	ODAddUpdate(&AppOD, CO_KEY(0x1400, 2, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0xFE), co_node_spec.Drv);

	ODAddUpdate(&AppOD, CO_KEY(0x6042, 0, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0x01), co_node_spec.Drv);
	ODAddUpdate(&AppOD, CO_KEY(0x6042, 1, CO_UNSIGNED32	| CO_OBJ____RW), CO_TERPM, (CO_DATA)(&ERPMFreq), co_node_spec.Drv);

	ODAddUpdate(&AppOD, CO_KEY(0x1600, 0, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0x01), co_node_spec.Drv);
	ODAddUpdate(&AppOD, CO_KEY(0x1600, 1, CO_UNSIGNED32	| CO_OBJ_D__RW), 0, CO_LINK(0x6042, 1, 32), co_node_spec.Drv);
	
	ODNvmToBuffer(co_node_spec.Drv, &AppOD);

	ODBufferToNvm(co_node_spec.Drv, &AppOD);

	ODClearBuffer();

	CORPdoInit(co_node.RPdo, &co_node);
	COTPdoInit(co_node.TPdo, &co_node);

	stop_now = false;
	chThdCreateStatic(cot_thread_wa, sizeof(cot_thread_wa),
			NORMALPRIO, cot_thread, NULL);

	// Terminal commands for the VESC Tool terminal can be registered.
	terminal_register_command_callback(
			"canopen_test_cmd",
			"Print the number d",
			"[d]",
			terminal_test);

	COTmrCreate(	&(co_node.Tmr),
			100,
			100,
			ERPM_TPdo_Callback,
			0);
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
	app_set_configuration(conf);
}

static THD_FUNCTION(cot_thread, arg) {
	(void)arg;

	chRegSetThreadName("App Canopen Test");

	is_running = true;

	//mcpwm_foc_beep(880.0f, 1.0f, 0.1f);

	//mcpwm_foc_beep(220, 100, 0.5);
	// Example of using the experiment plot
//	chThdSleepMilliseconds(8000);
//	commands_init_plot("Sample", "Voltage");
//	commands_plot_add_graph("Temp Fet");
//	commands_plot_add_graph("Input Voltage");
//	float samp = 0.0;
//
//	for(;;) {
//		commands_plot_set_graph(0);
//		commands_send_plot_points(samp, mc_interface_temp_fet_filtered());
//		commands_plot_set_graph(1);
//		commands_send_plot_points(samp, GET_INPUT_VOLTAGE());
//		samp++;
//		chThdSleepMilliseconds(10);
//	}

	for(;;) {
		// Check if it is time to stop.
		if (stop_now) {
			is_running = false;
			return;
		}

		// Run your logic here. A lot of functionality is available in mc_interface.h.
		
		chThdSleepMilliseconds(100);
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
#endif //USE_CANOPEN
