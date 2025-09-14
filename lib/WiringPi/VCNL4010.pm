#!/usr/bin/env perl

use strict;
use warnings;
use 5.012;

package WiringPi::VCNL4010;

use Exporter qw/import/;

our @EXPORT_OK = qw/
    VCNL4010_init
    read_ambient
    read_proximity
/;

our %EXPORT_TAGS = (
    all => \@EXPORT_OK,
);

use Time::HiRes qw/time sleep/;
use WiringPi qw/:all/;

# Common VCNL40xx constants:
use constant VCNL40xx_ADDRESS          => 0x13;
use constant VCNL40xx_COMMAND          => 0x80;
use constant VCNL40xx_PRODUCTID        => 0x81;
use constant VCNL40xx_IRLED            => 0x83;
use constant VCNL40xx_AMBIENTPARAMETER => 0x84;
use constant VCNL40xx_AMBIENTDATA      => 0x85;
use constant VCNL40xx_PROXIMITYDATA    => 0x87;
use constant VCNL40xx_PROXIMITYADJUST  => 0x8A;
use constant VCNL40xx_3M125            => 0;
use constant VCNL40xx_1M5625           => 1;
use constant VCNL40xx_781K25           => 2;
use constant VCNL40xx_390K625          => 3;
use constant VCNL40xx_MEASUREAMBIENT   => 0x10;
use constant VCNL40xx_MEASUREPROXIMITY => 0x08;
use constant VCNL40xx_AMBIENTREADY     => 0x40;
use constant VCNL40xx_PROXIMITYREADY   => 0x20;

# VCNL4010 constants:
use constant VCNL4010_PROXRATE         => 0x82;
use constant VCNL4010_INTCONTROL       => 0x89;
use constant VCNL4010_INTSTAT          => 0x8E;
use constant VCNL4010_MODTIMING        => 0x8F;
use constant VCNL4010_INT_PROX_READY   => 0x80;
use constant VCNL4010_INT_ALS_READY    => 0x40;

sub VCNL4010_init {

    die 1 if (wiringPiSetupGpio() == -1);

    my $fd = wiringPiI2CSetup( VCNL40xx_ADDRESS );
    die "Error initializing VCNL4010\n" if ($fd == -1);
    my $err = wiringPiI2CWriteReg8( $fd, VCNL4010_INTCONTROL,  0x08 );
    die "Error during init write: $err\n" if ($err);
    wiringPiI2CWriteReg8( $fd, VCNL40xx_IRLED, 10 );

    return $fd;

}

sub _clear_int {

    my ($fd, $int_bit) = @_;
    my $status = wiringPiI2CReadReg8( $fd, VCNL4010_INTSTAT & 0xff );
    $status &= ~$int_bit;
    wiringPiI2CWriteReg8( $fd, VCNL4010_INTSTAT & 0xff, $status );

}


sub _wait_response {

    my ($fd, $ready) = @_;

    while (1) {
        my $res = wiringPiI2CReadReg8( $fd, VCNL40xx_COMMAND );
        return if (($res & $ready) > 0);
        sleep 0.001;
    }

}

sub read_ambient {

    my ($fd) = @_;

    _clear_int($fd, VCNL4010_INT_ALS_READY );
    wiringPiI2CWriteReg8( $fd, VCNL40xx_COMMAND, VCNL40xx_MEASUREAMBIENT);
    _wait_response($fd, VCNL40xx_AMBIENTREADY );
    return _wiringPiI2CReadReg16BE( $fd, VCNL40xx_AMBIENTDATA );

}

sub read_proximity {

    my ($fd) = @_;

    _clear_int($fd, VCNL4010_INT_PROX_READY );
    wiringPiI2CWriteReg8( $fd, VCNL40xx_COMMAND & 0xff, VCNL40xx_MEASUREPROXIMITY & 0xff);
    _wait_response($fd, VCNL40xx_PROXIMITYREADY );
    return _wiringPiI2CReadReg16BE( $fd, VCNL40xx_PROXIMITYDATA );

}

sub _wiringPiI2CWriteReg16BE {

    my ($fd, $reg, $val) = @_;
    $val = unpack 'S', pack( 'n', $val );
    return wiringPiI2CWriteReg16( $fd, $reg, $val );

}

sub _wiringPiI2CReadReg16BE {

    my ($fd, $reg) = @_;
    my $val = wiringPiI2CReadReg16( $fd, $reg );
    return unpack 'n', pack( 'S', $val );

}
