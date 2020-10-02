#!/bin/bash
#gdb-multiarch -x ./script2.gdb ./build/modbus.elf
~/src/stm32/gcc-arm-none-eabi-9-2020-q2-update/bin/arm-none-eabi-gdb-py -x ./script2.gdb ./build/smooservo.elf
