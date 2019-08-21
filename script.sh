#!/bin/sh

iwpan dev wpan0 set pan_id 0x1111
iwpan dev wpan0 set short_addr 0x0001
ip link set wpan0 up
wpan-ping -a 0x0001  -s 5 -c5
