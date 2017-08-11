#!/bin/bash
echo "Updating submodule..."
git submodule update --init

mkdir lib

echo "Generating bindings..."
swig -perl -const -outdir lib wiringpi.i

CORE=`perl -MConfig -e 'print $Config{archlib}'`/CORE
WIRINGPI=wiringPi/wiringPi

echo "Building against: $CORE"

gcc -fpic -c -Dbool=char -I$CORE wiringpi_wrap.c \
$WIRINGPI/wiringSerial.c \
$WIRINGPI/wiringShift.c \
$WIRINGPI/wiringPi.c \
$WIRINGPI/softPwm.c \
$WIRINGPI/softTone.c \
-D_GNU_SOURCE

gcc -shared *.o -o lib/wiringpi.so
