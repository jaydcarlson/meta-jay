#!/bin/sh

APP=/usr/local/intellifarm-app/IntellifarmApp.exe

[ -e ${APP} ] || exit 1

while [ 1 ] ; do 
	mono ${APP} /dev/lora_radio2 >> /var/log/intellifarm.log 2>&1
	sleep 10
done
