#!/bin/sh

APP=/usr/local/weather-app/weather-app.exe

[ -e ${APP} ] || exit 1

while [ 1 ] ; do 
	mono ${APP} T >> /var/log/weather-app.log 2>&1
	sleep 10
done
