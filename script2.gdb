set target-async on
set mem inaccessible-by-default off
target extended-remote /dev/ttyBmpGdb
#monitor traceswo
monitor swdp_scan
monitor traceswo
set debug remote 1
attach 1
#stop
#monitor traceswo

#file ./build/modbus.elf 
load ./build/smooservo.hex 
compare-sections
#hbreak main
#hbreak Core/Src/main.c:258
#next
#watch huart1
#watch data[0]
run
#monitor hard_srst


