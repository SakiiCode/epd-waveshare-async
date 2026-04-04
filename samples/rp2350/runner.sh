#!/bin/sh
picotool load -u -v -x -t elf $1
socat /dev/ttyACM0,rawer,b115200 STDOUT | defmt-print -e $1