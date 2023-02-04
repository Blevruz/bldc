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

#include "co_nvm_vpkg.h"
#include "flash_helper.h"
#include "canopen_od_flash.h"
//#include "vesc_c_if.h"

/******************************************************************************
* PRIVATE DEFINES
******************************************************************************/

/******************************************************************************
* PRIVATE FUNCTIONS
******************************************************************************/

static void     DrvNvmInit  (void);
static uint32_t DrvNvmRead  (uint32_t start, uint8_t *buffer, uint32_t size);
static uint32_t DrvNvmWrite (uint32_t start, uint8_t *buffer, uint32_t size);


/******************************************************************************
* PUBLIC VARIABLE
******************************************************************************/

uint32_t VescPkgNvmDriver_offset = 0;

const CO_IF_NVM_DRV VescPkgNvmDriver = {
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
	//step 1: find beginning of OD
	int index = 0;
	int offset = 0;
	if (ODf_GetActiveOD	(&index, &offset) < 0)
		return -1;	//error somewhere
	
	//step 2: read from appropriate addr
        return flash_helper_read_nvm(buffer, size, offset + start) ? size : 0;
	//return VESC_IF->read_nvm(buffer, size, start + VescPkgNvmDriver_offset) ? size : 0;
}

static uint32_t DrvNvmWrite(uint32_t base, uint8_t *data, uint32_t len)
{
	//step 1: find beginning of OD
	int index = 0;
	int offset = 0;
	if (ODf_GetActiveOD	(&index, &offset) < 0)
		return -1;	//error somewhere
	
	//step 2: write to appropriate addr
	return flash_helper_write_nvm(data, len, offset + base) ? len : 0;
	//return VESC_IF->write_nvm(data, len, base + VescPkgNvmDriver_offset) ? len : 0;
}
