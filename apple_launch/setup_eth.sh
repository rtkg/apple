#!/bin/bash
ifconfig eth0 192.170.10.3 netmask 255.255.255.0 up
#ifconfig eth0 172.31.1.3 netmask 255.255.255.0 up
ifconfig eth0:0 10.0.0.68 netmask 255.255.0.0 up
