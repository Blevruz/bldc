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

#include "co_nvm_chos.h"
#include "stm32f4xx_conf.h"
#include "mc_interface.h"
#include "timeout.h"
#include "utils_sys.h"

/******************************************************************************
* PRIVATE DEFINES
******************************************************************************/

//TODO: find proper size for Object dictionary (assuming 128KB)
#define NVM_CHOS_SIZE 1024

/******************************************************************************
* PRIVATE FUNCTIONS
******************************************************************************/

static void     DrvNvmInit  (void);
static uint32_t DrvNvmRead  (uint32_t start, uint8_t *buffer, uint32_t size);
static uint32_t DrvNvmWrite (uint32_t start, uint8_t *buffer, uint32_t size);


/******************************************************************************
* PUBLIC VARIABLE
******************************************************************************/

//uint8_t* __attribute__((__section__(".co_od"))) od_nvm_mem;

const CO_IF_NVM_DRV ChOSNvmDriver = {
    DrvNvmInit,
    DrvNvmRead,
    DrvNvmWrite
};

/******************************************************************************
* PRIVATE FUNCTIONS
******************************************************************************/

static void DrvNvmInit(void)
{
	//NVM init handled by ChibiOS
}

static uint32_t DrvNvmRead(uint32_t start, uint8_t *buffer, uint32_t size)
{
    uint32_t idx = 0;
    uint32_t pos;

    idx = 0;
    pos = start;
    while ((pos < NVM_CHOS_SIZE) && (idx < size)) {
        buffer[idx] = *(uint8_t*)(NVM_CHOS_ADDRESS + pos);//od_nvm_mem[pos];
        idx++;
        pos++;
    }

    return (idx);
}

static uint32_t DrvNvmWrite(uint32_t base, uint8_t *data, uint32_t len)
{
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
			FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

	mc_interface_ignore_input_both(5000);
	mc_interface_release_motor_override_both();

	if (!mc_interface_wait_for_motor_release_both(3.0)) {
		return 100;
	}

	utils_sys_lock_cnt();
	timeout_configure_IWDT_slowest();

	for (uint32_t i = 0;i < len;i++) {
		uint16_t res = FLASH_ProgramByte(base + i + NVM_CHOS_ADDRESS, data[i]);
		if (res != FLASH_COMPLETE) {
			FLASH_Lock();
			timeout_configure_IWDT();
			mc_interface_ignore_input_both(5000);
			utils_sys_unlock_cnt();
			return res;
		}
	}

	FLASH_Lock();
	timeout_configure_IWDT();
	mc_interface_ignore_input_both(100);
	utils_sys_unlock_cnt();

	return FLASH_COMPLETE;
/*
    uint32_t idx = 0;
    uint32_t pos;

    idx = 0;
    pos = start;
    while ((pos < NVM_CHOS_SIZE) && (idx < size)) {
        FLASH_ProgramByte(NVM_CHOS_ADDRESS + pos, buffer[idx]);
	idx++;
        pos++;
    }

    return (idx);
*/
}
