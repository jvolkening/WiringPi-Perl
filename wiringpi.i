%module wiringpi

%rename("wiringPiSetupPerl")     wiringPiSetup(void);
%rename("wiringPiSetupGpioPerl") wiringPiSetupGpio(void);
%rename("wiringPiSetupSysPerl")  wiringPiSetupSys(void);

#define INPUT            0
#define OUTPUT           1
#define PWM_OUTPUT       2
#define GPIO_CLOCK       3
#define SOFT_PWM_OUTPUT  4
#define SOFT_TONE_OUTPUT 5
#define PWM_TONE_OUTPUT  6

#define LOW  0
#define HIGH 1

#define PUD_OFF  0
#define PUD_DOWN 1
#define PUD_UP   2

#define PWM_MODE_MS  0
#define PWM_MODE_BAL 1

#define WPI_MODE_PINS           0
#define WPI_MODE_GPIO           1
#define WPI_MODE_GPIO_SYS       2
#define WPI_MODE_PHYS           3
#define WPI_MODE_PIFACE         4
#define WPI_MODE_UNINITIALISED -1

#define INT_EDGE_SETUP   0
#define INT_EDGE_FALLING 1
#define INT_EDGE_RISING  2
#define INT_EDGE_BOTH    3

%apply unsigned char { uint8_t };

extern int  wiringPiSetup     (void) ;
extern int  wiringPiSetupSys  (void) ;
extern int  wiringPiSetupGpio (void) ;

extern void pullUpDnControl   (int pin, int pud) ;
extern void pinMode           (int pin, int mode) ;
extern void digitalWrite      (int pin, int value) ;
extern void pwmWrite          (int pin, int value) ;
extern int  digitalRead       (int pin) ;
extern void shiftOut          (uint8_t dPin, uint8_t cPin, uint8_t order, uint8_t val);
extern uint8_t shiftIn        (uint8_t dPin, uint8_t cPin, uint8_t order);

extern void         delay             (unsigned int howLong) ;
extern void         delayMicroseconds (unsigned int howLong) ;
extern unsigned int millis            (void) ;

extern int  serialOpen      (char *device, int baud) ;
extern void serialClose     (int fd) ;
extern void serialPutchar   (int fd, uint8_t c) ;
extern void serialPuts      (int fd, char *s) ;
extern int  serialDataAvail (int fd) ;
extern int  serialGetchar   (int fd) ;
extern void serialPrintf    (int fd, char *message, ...) ;

extern int  softPwmCreate (int pin, int initialValue, int pwmRange) ;
extern void softPwmWrite  (int pin, int value) ;

extern int  wiringPiSPISetup  (int channel, int speed) ;
extern int  wiringPiSPIDataRW (int channel, uint8_t *data, int len) ;

extern int piHiPri (int priority) ;

extern int wpiPinToGpio (int wpiPin) ;
extern int wpiPhysToGpio (int physPin) ;

%{
#include "WiringPi/wiringPi/wiringPi.h"
#include "WiringPi/wiringPi/wiringShift.h"
#include "WiringPi/wiringPi/wiringSerial.h"
#include "WiringPi/wiringPi/piHiPri.c"
#include "WiringPi/wiringPi/softPwm.h"
#include "WiringPi/wiringPi/wiringPiSPI.h"
%}

%perlcode %{

require IO::Select;

our $wiringPiMode;
our @_children;

END {

    for (@_children) {
        kill -9, $_;
        waitpid($_, 0);
    }

}

my @const = qw/
    INPUT
    OUTPUT
    PWM_OUTPUT
    GPIO_CLOCK
    SOFT_PWM_OUTPUT
    SOFT_TONE_OUTPUT
    PWM_TONE_OUTPUT
    LOW
    HIGH
    PUD_OFF
    PUD_DOWN
    PUD_UP
    PWM_MODE_MS
    PWM_MODE_BAL
    WPI_MODE_PINS
    WPI_MODE_GPIO
    WPI_MODE_GPIO_SYS
    WPI_MODE_PHYS
    WPI_MODE_PIFACE
    WPI_MODE_UNINITIALISED
    INT_EDGE_SETUP
    INT_EDGE_FALLING
    INT_EDGE_RISING
    INT_EDGE_BOTH
/; 

my @func = qw/
    wiringPiSetup
    wiringPiSetupSys
    wiringPiSetupGpio
    PullUpDnControl
    pinMode
    digitalWrite
    pwmWrite
    digitalRead
    shiftOut
    shiftIn
    delay
    delayMicroseconds
    millis
    serialOpen
    serialClose
    serialPutchar
    serialPuts
    serialDataAvail
    serialGetchar
    serialPrintf
    softPwmCreate
    softPwmWrite
    wiringPiSPISetup
    wiringPiSPIDataRW
    wiringPiISR
    piHiPri
/;

@EXPORT_OK = (@const, @func);
%EXPORT_TAGS = (
 'constants' => [ @const ],
 'functions' => [ @func  ],
 'all'       => [ @const, @func ],
);

# This is done in pure Perl but uses the same sysfs-based mechanism as the C
# libs

sub wiringPiISR {

    my ($pin, $edge, $cb) = @_;

    $pin = $wiringpi::wiringPiMode == WPI_MODE_PINS ? wpiPinToGpio($pin)
         : $wiringpi::wiringPiMode == WPI_MODE_PHYS ? physPinToGpio($pin)
         : $wiringpi::wiringPiMode == WPI_MODE_GPIO ? $pin
         : die "Failed to determine pin mode";
         

    $edge = $edge == INT_EDGE_RISING  ? 'rising'
          : $edge == INT_EDGE_FALLING ? 'falling'
          : $edge == INT_EDGE_BOTH    ? 'both'
          : return();

    my $ret = system("gpio edge $pin $edge");
    die "gpio command failed: check that the binary is installed"
        if ($ret);

    my $fn = "/sys/class/gpio/gpio$pin/value";
    die "pin not set for interrupt" if (! -e $fn);

    my $pid = fork;
    if ($pid) {
        push @_children, $pid;
        return;
    }

    piHiPri(10);
    open my $in, '<', $fn or die "Error opening sysfs: $!\n";

    # this is necessary to prevent early firing
    my $val = <$in>;
    seek $in, 0, 0;

    my $poll = IO::Select->new($in);
    while ($poll->has_exception) {

        # fork again to keep lengthy callbacks from blocking new firings
        if (fork() == 0) {
            $cb->();
            exit;
        }

        # This is necessary to clear the event each time
        my $val = <$in>;
        seek $in, 0, 0;
    }

    waitpid(-1, 0);
    exit;

}

# These are redefined so that we can keep track of the pin mode

sub wiringPiSetup {
    $wiringPiMode = WPI_MODE_PINS;
    wiringPiSetupPerl();
}

sub wiringPiSetupGpio {
    $wiringPiMode = WPI_MODE_GPIO;
    wiringPiSetupGpioPerl();
}

sub wiringPiSetupSys {
    $wiringPiMode = WPI_MODE_PHYS;
    wiringPiSetupSys();
}

%}
