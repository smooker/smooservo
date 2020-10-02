#!/bin/bash
#gdb-multiarch -x ./script2.gdb ./build/modbus.elf
arm-none-eabi-gdb-py -x ./script2.gdb ./build/smooservo.elf
