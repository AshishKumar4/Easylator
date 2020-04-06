#!/bin/bash

pkill -f socat

if [[ ! $1 ]]; then
    ip=10.8.0.10
else 
    ip=$1
fi

if [[ ! $2 ]]; then
    sock="/dev/ttyACM0"
else 
    sock=$2
fi

if [[ ! $3 ]]; then
    port=54321
else 
    port=$3
fi

python3 -c "import os;exec(\"os.system('socat -d -d pty,link=$sock,waitslave tcp:$ip:$port');\");"
#chown ashishkumarsingh:dialout $sock
#chown ashishkumarsingh:dialout /dev/pts/7
