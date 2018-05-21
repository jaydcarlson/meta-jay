#!/bin/sh

GPIO=/sys/class/gpio

echo 16 > $GPIO/export
echo out > $GPIO/gpio16/direction
echo 1 > $GPIO/gpio16/value
sleep 2

echo 45 > $GPIO/export
echo out > $GPIO/gpio45/direction

echo 32 > $GPIO/export
echo out > $GPIO/gpio32/direction

echo 39 > $GPIO/export
echo out > $GPIO/gpio39/direction

echo 1 > $GPIO/gpio32/value

sleep 1
echo 1 > $GPIO/gpio39/value
sleep 1
echo 0 > $GPIO/gpio39/value

stty -F /dev/ttymxc1 -crtscts
