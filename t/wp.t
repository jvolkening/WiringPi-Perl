#!/usr/bin/env perl

use strict;
use warnings;
use 5.012;

use Test::More;

use WiringPi qw/:all/;

require_ok( "WiringPi" );

ok( wiringPiSetup() != -1, "initialization" );

pinMode(8,INPUT);

ok( digitalRead(8) == 1, "read pin with hardware pull-up" );

done_testing();
exit;
