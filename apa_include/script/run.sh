#!/system/bin/sh
ip addr flush dev eth0
sleep 1
ip addr add 192.168.1.10/24 dev eth0
sleep 1
ip addr del 192.168.10.100/24 dev eth0
sleep 1
ip link set eth0 up
sleep 1
ip rule add to 192.168.1.0/24 lookup eth0
sleep 1
export LD_LIBRARY_PATH=./extlib:./lib:./:$LD_LIBRARY_PATH
./apa_pk_ndk
