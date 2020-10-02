set target-async on
set mem inaccessible-by-default off
target extended-remote /dev/ttyBmpGdb
monitor traceswo
monitor swdp_scan
#monitor traceswo
set debug remote 0
attach 1
stop
monitor traceswo

#file ./build/modbus.elf 
load ./build/smooservo.hex 
#compare-sections
hbreak main
#next
#watch huart
#watch data[0]
run

