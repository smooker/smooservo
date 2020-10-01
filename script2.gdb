set target-async on
set mem inaccessible-by-default off
target extended-remote /dev/ttyBmpGdb
monitor traceswo
monitor swdp_scan
#set debug remote 1
attach 1
#monitor traceswo
stop

#file ./build/modbus.elf 
load ./build/smooservo.hex 
compare-sections
#hbreak main
#next
#watch huart
#watch data[0]
run

