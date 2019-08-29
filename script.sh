#!/bin/sh

#ip link add link wpan0 name lowpan0 type lowpan
iwpan dev wpan0 set pan_id 0x1111
iwpan dev wpan0 set short_addr 0x0001
ip link set wpan0 up
#ip link set lowpan0 up
wpan-ping -a 0x0001 -s 5 -c 100 -I 5
ip link set wpan0 down

#--count | -c number of packets
#--size | -s packet length

