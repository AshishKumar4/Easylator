#!/bin/bash

pkill -f socat

if [[ ! $1 ]]; then
    sock="/dev/ttyACM0"
else 
    sock=$1
fi

if [[ ! $2 ]]; then
    port=54321
else 
    port=$2
fi
socat -d -d tcp-l:$port,reuseaddr,fork file:$sock,raw,nonblock,b57600,echo=0 & 
#,icanon=1,crnl
