/*
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
/*	PDS (Power Drive System) Objects
 *	Place implementation of CiA 402 objects here
 *	TODO: make a TODO list
 */

#ifndef	CANOPEN_PDS_OBJECTS_H_
#define	CANOPEN_PDS_OBJECTS_H_

#include "co_core.h"
#include "mc_interface.h"
#include "canopen_pds_fsa.h"

#define CONTROL_WORD_COMMAND_MASK	0x8F	//keeps only the bits used in Table 27
#define CONTROL_WORD_SHUTDOWN_N		0x01	//bit 3 is X
#define CONTROL_WORD_SWITCH_ON_N	0x08
#define CONTROL_WORD_SO_EN		0x0F	//switch on + enable operation
#define CONTROL_WORD_DISABLE_VOLTAGE_N	0x02	//this command is chosen if input&this == 0
#define CONTROL_WORD_QUICK_STOP_N	0x04

#define CONTROL_WORD_FAULT_RESET	0x80	//note: must be rising edge

extern uint16_t ControlWord;	// XXX temporary workaround until proper FSA is done

extern const CO_OBJ_TYPE COTCONTROLWORD;
#define CO_T_CONTROL_WORD ((CO_OBJ_TYPE*)&COTCONTROLWORD)

extern const CO_OBJ_TYPE COTDUTYCOMMAND;
#define CO_T_DUTY_COMMAND ((CO_OBJ_TYPE*)&COTDUTYCOMMAND)

extern const CO_OBJ_TYPE COTCURRENTCOMMAND;
#define CO_T_CURRENT_COMMAND ((CO_OBJ_TYPE*)&COTCURRENTCOMMAND)

extern const CO_OBJ_TYPE COTRPMCOMMAND;
#define CO_T_RPM_COMMAND ((CO_OBJ_TYPE*)&COTRPMCOMMAND)

extern uint16_t StatusWord;	// XXX temporary workaround until proper FSA is done

extern const CO_OBJ_TYPE COTSTATUSWORD;
#define CO_T_STATUS_WORD ((CO_OBJ_TYPE*)&COTSTATUSWORD)

#endif	//CANOPEN_PDS_OBJECTS_H_
