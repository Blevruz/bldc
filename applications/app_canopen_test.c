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
#include "canopen_driver.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

// User CANopen object type

uint32_t ERPMSize (CO_OBJ *obj, CO_NODE *node, uint32_t width) {return 4;}
CO_ERR   ERPMInit (CO_OBJ *obj, CO_NODE *node) {
	obj->Data = NULL;
	return CO_ERR_NONE;
}
CO_ERR   ERPMRead (CO_OBJ *obj, CO_NODE *node, void *buffer, uint32_t size) {
	if (size < 4)
		return CO_ERR_OBJ_SIZE;
	*(float*)buffer = *(float*)(obj->Data);
	return CO_ERR_NONE;
}
CO_ERR   ERPMWrite(CO_OBJ *obj, CO_NODE *node, void *buffer, uint32_t size) {
	if (size != 4)
		return CO_ERR_OBJ_SIZE;
	uint32_t o_data = 0;
	for (int i = 0; i < 4; i++)
		//*(uint32_t*)obj->Data += ((uint8_t*)buffer)[3-i] << i*8;
		o_data += ((uint8_t*)buffer)[3-i] << i*8;
//	mcpwm_foc_beep(*(float*)obj->Data, 1.0f, 0.5f);
	mc_interface_set_pid_speed(o_data);
	timeout_reset();
	*(uint32_t*)obj->Data = o_data;
	return CO_ERR_NONE;
}
CO_ERR   ERPMReset(CO_OBJ *obj, CO_NODE *node, uint32_t para) {
	*(float*)obj->Data = (float)para;
	return CO_ERR_NONE;
}

CO_OBJ_TYPE COTERPM = {
	ERPMSize,
	0,
	ERPMRead,
	ERPMWrite,
	ERPMReset
};

float ERPMFreq = 0;

#define CO_TERPM ((CO_OBJ_TYPE*)&COTERPM)

// Threads
static THD_FUNCTION(cot_thread, arg);
static THD_WORKING_AREA(cot_thread_wa, 1024);

// Private functions
static void pwm_callback(void);
static void terminal_test(int argc, const char **argv);

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;

// Called when the canopen_test application is started. Start our
// threads here and set up callbacks.
void app_canopen_test_start(void) {
	mc_interface_set_pwm_callback(pwm_callback);

	stop_now = false;
	chThdCreateStatic(cot_thread_wa, sizeof(cot_thread_wa),
			NORMALPRIO, cot_thread, NULL);

	// Terminal commands for the VESC Tool terminal can be registered.
	terminal_register_command_callback(
			"canopen_test_cmd",
			"Print the number d",
			"[d]",
			terminal_test);

	ODAddUpdate(&AppOD, CO_KEY(0x1400, 0, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0x02), co_node_spec.Drv);
	ODAddUpdate(&AppOD, CO_KEY(0x1400, 1, CO_UNSIGNED32	| CO_OBJ_D__R_), 0, (CO_DATA)(0x000000AD), co_node_spec.Drv);
	ODAddUpdate(&AppOD, CO_KEY(0x1400, 2, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0xFE), co_node_spec.Drv);

	ODAddUpdate(&AppOD, CO_KEY(0x3141, 0, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0x01), co_node_spec.Drv);
	ODAddUpdate(&AppOD, CO_KEY(0x3141, 1, CO_UNSIGNED32	| CO_OBJ____RW), CO_TERPM, (CO_DATA)(&ERPMFreq), co_node_spec.Drv);

	ODAddUpdate(&AppOD, CO_KEY(0x1600, 0, CO_UNSIGNED8	| CO_OBJ_D__R_), 0, (CO_DATA)(0x01), co_node_spec.Drv);
	ODAddUpdate(&AppOD, CO_KEY(0x1600, 1, CO_UNSIGNED32	| CO_OBJ_D__RW), 0, CO_LINK(0x3141, 1, 32), co_node_spec.Drv);
	
	ODNvmToBuffer(co_node_spec.Drv, &AppOD);

	ODBufferToNvm(co_node_spec.Drv, &AppOD);

	ODClearBuffer();

	CORPdoInit(co_node.RPdo, &co_node);
	COTPdoInit(co_node.TPdo, &co_node);
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

void app_canopen_test_configure(app_configuration *conf) {
	(void)conf;
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

		timeout_reset(); // Reset timeout if everything is OK.

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
#endif //USE_CANOPEN
