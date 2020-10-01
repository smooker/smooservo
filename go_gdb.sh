#!/bin/bash
#gdb-multiarch -x ./script2.gdb ./build/modbus.elf
arm-eabi-gdb -x ./script2.gdb ./build/smooservo.elf
