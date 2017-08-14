#!/bin/bash
echo "Updating submodule..."
git submodule update --init

CORE=`perl -MConfig -e 'print $Config{archlib}'`/CORE
WIRINGPI=wiringPi/wiringPi

echo "Building against: $CORE"

gcc -fpic -c -Dbool=char -I$CORE WiringPi_wrap.c \
$WIRINGPI/wiringSerial.c \
$WIRINGPI/wiringShift.c \
$WIRINGPI/wiringPi.c \
$WIRINGPI/wiringPiSPI.c \
$WIRINGPI/softPwm.c \
$WIRINGPI/softTone.c \
-D_GNU_SOURCE

gcc -shared *.o -o lib/WiringPi.so
