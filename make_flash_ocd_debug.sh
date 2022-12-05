#!/bin/bash -e
make fw_60_flash; openocd -f /usr/share/openocd/scripts/board/stm32f4discovery.cfg -c "stm32f4x.cpu configure -rtos auto" &
arm-none-eabi-gdb -ex "target remote localhost:3333" -ex "file build/60/60.elf"

