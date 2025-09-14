package WiringPI::VL53L0X;

use strict;
use warnings;
use 5.012;

use Time::HiRes qw/time sleep/;
use WiringPi qw/:all/;

use constant ADDRESS_DEFAULT => 0x29;

use constant VCSEL_PERIOD_PRE_RANGE   => 0;
use constant VCSEL_PERIOD_FINAL_RANGE => 1;

use constant SYSRANGE_START                              => 0x00;
use constant SYSTEM_THRESH_HIGH                          => 0x0C;
use constant SYSTEM_THRESH_LOW                           => 0x0E;
use constant SYSTEM_SEQUENCE_CONFIG                      => 0x01;
use constant SYSTEM_RANGE_CONFIG                         => 0x09;
use constant SYSTEM_INTERMEASUREMENT_PERIOD              => 0x04;
use constant SYSTEM_INTERRUPT_CONFIG_GPIO                => 0x0A;
use constant GPIO_HV_MUX_ACTIVE_HIGH                     => 0x84;
use constant SYSTEM_INTERRUPT_CLEAR                      => 0x0B;
use constant RESULT_INTERRUPT_STATUS                     => 0x13;
use constant RESULT_RANGE_STATUS                         => 0x14;
use constant RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       => 0xBC;
use constant RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        => 0xC0;
use constant RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       => 0xD0;
use constant RESULT_CORE_RANGING_TOTAL_EVENTS_REF        => 0xD4;
use constant RESULT_PEAK_SIGNAL_RATE_REF                 => 0xB6;
use constant ALGO_PART_TO_PART_RANGE_OFFSET_MM           => 0x28;
use constant I2C_SLAVE_DEVICE_ADDRESS                    => 0x8A;
use constant MSRC_CONFIG_CONTROL                         => 0x60;
use constant PRE_RANGE_CONFIG_MIN_SNR                    => 0x27;
use constant PRE_RANGE_CONFIG_VALID_PHASE_LOW            => 0x56;
use constant PRE_RANGE_CONFIG_VALID_PHASE_HIGH           => 0x57;
use constant PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          => 0x64;
use constant FINAL_RANGE_CONFIG_MIN_SNR                  => 0x67;
use constant FINAL_RANGE_CONFIG_VALID_PHASE_LOW          => 0x47;
use constant FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         => 0x48;
use constant FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT => 0x44;
use constant PRE_RANGE_CONFIG_SIGMA_THRESH_HI            => 0x61;
use constant PRE_RANGE_CONFIG_SIGMA_THRESH_LO            => 0x62;
use constant PRE_RANGE_CONFIG_VCSEL_PERIOD               => 0x50;
use constant PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          => 0x51;
use constant PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          => 0x52;
use constant SYSTEM_HISTOGRAM_BIN                        => 0x81;
use constant HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       => 0x33;
use constant HISTOGRAM_CONFIG_READOUT_CTRL               => 0x55;
use constant FINAL_RANGE_CONFIG_VCSEL_PERIOD             => 0x70;
use constant FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        => 0x71;
use constant FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        => 0x72;
use constant CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       => 0x20;
use constant MSRC_CONFIG_TIMEOUT_MACROP                  => 0x46;
use constant SOFT_RESET_GO2_SOFT_RESET_N                 => 0xBF;
use constant IDENTIFICATION_MODEL_ID                     => 0xC0;
use constant IDENTIFICATION_REVISION_ID                  => 0xC2;
use constant OSC_CALIBRATE_VAL                           => 0xF8;
use constant GLOBAL_CONFIG_VCSEL_WIDTH                   => 0x32;
use constant GLOBAL_CONFIG_SPAD_ENABLES_REF_0            => 0xB0;
use constant GLOBAL_CONFIG_SPAD_ENABLES_REF_1            => 0xB1;
use constant GLOBAL_CONFIG_SPAD_ENABLES_REF_2            => 0xB2;
use constant GLOBAL_CONFIG_SPAD_ENABLES_REF_3            => 0xB3;
use constant GLOBAL_CONFIG_SPAD_ENABLES_REF_4            => 0xB4;
use constant GLOBAL_CONFIG_SPAD_ENABLES_REF_5            => 0xB5;
use constant GLOBAL_CONFIG_REF_EN_START_SELECT           => 0xB6;
use constant DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         => 0x4E;
use constant DYNAMIC_SPAD_REF_EN_START_OFFSET            => 0x4F;
use constant POWER_MANAGEMENT_GO1_POWER_FORCE            => 0x80;
use constant VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           => 0x89;
use constant ALGO_PHASECAL_LIM                           => 0x30;
use constant ALGO_PHASECAL_CONFIG_TIMEOUT                => 0x30;

sub new {

    my ($class, %args) = @_;

    my $self = bless {} => $class;

    die if (wiringPiSetupGpio() == -1);

    $self->{addr} = $args{address} // ADDRESS_DEFAULT;

    my $fd = wiringPiI2CSetup( $self->{addr} );
    die "Error initializing VCNL4010\n" if ($self->{fd} == -1);
    $self->{fd} = $fd;

    $self->{io_2v8}     = 0; # try TRUE???
    $self->{io_timeout} = 0;

    if ($self->{io_2v8}) {
        my $int = wiringPiI2CReadReg8( $fd, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV );
        wiringPiI2CWriteReg8( $fd, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, $int | 0x01 );
    }

    # standard mode
    wiringPiI2CWriteReg8( $fd, 0x88, 0x00 );
    wiringPiI2CWriteReg8( $fd, 0x80, 0x01 );
    wiringPiI2CWriteReg8( $fd, 0xff, 0x01 );
    wiringPiI2CWriteReg8( $fd, 0x00, 0x00 );
    $self->{stop_var} = wiringPiI2CReadReg8( $fd, 0x91 );
    wiringPiI2CWriteReg8( $fd, 0x00, 0x01 );
    wiringPiI2CWriteReg8( $fd, 0xff, 0x00 );
    wiringPiI2CWriteReg8( $fd, 0x80, 0x00 );

    my $int = wiringPiI2CReadReg8( $fd, MSRC_CONFIG_CONTROL );
    wiringPiI2CWriteReg8( $fd, MSRC_CONFIG_CONTROL, ($int | 0x12) );

    set_signal_rate_limit(0.25);

    wiringPiI2CWriteReg8( $fd, SYSTEM_SEQUENCE_CONFIG, 0xff );

    my ($spad_count, $spad_type_is_aperature)
        = _get_spad_info($fd);

    my @ref_spad_map = map {wiringPiI2CReadReg8( $fd, $_ )}
        GLOBAL_CONFIG_SPAD_EANBLES_REF_0..GLOBAL_CONFIG_SPAD_ENABLES_REF_0+5;

    wiringPiI2CWriteReg8( $fd, 0xff, 0x01 );
    wiringPiI2CWriteReg8( $fd, DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00 );
    wiringPiI2CWriteReg8( $fd, DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2c );
    wiringPiI2CWriteReg8( $fd, 0xff, 0x00 );
    wiringPiI2CWriteReg8( $fd, GLOBAL_CONFIG_REF_EN_START_SELECT, 0xb4 );

    my $first_spad_to_enable = $spad_type_is_aperature ? 12 : 0;
    my $spads_enabled = 0;
    for my $i (0..47) {
        if ($i < $first_spad_to_enable || $spads_enabled == $spad_count) {
            $ref_spad_map[ int($i/8) ] &= ~(1 << ($i % 8));
        }
        elsif (($ref_spad_map[ int($i/8) ] >> ($i % 8)) & 0x1) {
            $spads_enabled++;
        }
    }

    write_multi( GLOBAL_CONFIG_SPAD_ENABLES_REF_0, \@ref_spad_map, 6 );

    wiringPiI2CWriteReg8( $fd, 0xFF, 0x01);
    wiringPiI2CWriteReg8( $fd, 0x00, 0x00);

    wiringPiI2CWriteReg8( $fd, 0xFF, 0x00);
    wiringPiI2CWriteReg8( $fd, 0x09, 0x00);
    wiringPiI2CWriteReg8( $fd, 0x10, 0x00);
    wiringPiI2CWriteReg8( $fd, 0x11, 0x00);

    wiringPiI2CWriteReg8( $fd, 0x24, 0x01);
    wiringPiI2CWriteReg8( $fd, 0x25, 0xFF);
    wiringPiI2CWriteReg8( $fd, 0x75, 0x00);

    wiringPiI2CWriteReg8( $fd, 0xFF, 0x01);
    wiringPiI2CWriteReg8( $fd, 0x4E, 0x2C);
    wiringPiI2CWriteReg8( $fd, 0x48, 0x00);
    wiringPiI2CWriteReg8( $fd, 0x30, 0x20);

    wiringPiI2CWriteReg8( $fd, 0xFF, 0x00);
    wiringPiI2CWriteReg8( $fd, 0x30, 0x09);
    wiringPiI2CWriteReg8( $fd, 0x54, 0x00);
    wiringPiI2CWriteReg8( $fd, 0x31, 0x04);
    wiringPiI2CWriteReg8( $fd, 0x32, 0x03);
    wiringPiI2CWriteReg8( $fd, 0x40, 0x83);
    wiringPiI2CWriteReg8( $fd, 0x46, 0x25);
    wiringPiI2CWriteReg8( $fd, 0x60, 0x00);
    wiringPiI2CWriteReg8( $fd, 0x27, 0x00);
    wiringPiI2CWriteReg8( $fd, 0x50, 0x06);
    wiringPiI2CWriteReg8( $fd, 0x51, 0x00);
    wiringPiI2CWriteReg8( $fd, 0x52, 0x96);
    wiringPiI2CWriteReg8( $fd, 0x56, 0x08);
    wiringPiI2CWriteReg8( $fd, 0x57, 0x30);
    wiringPiI2CWriteReg8( $fd, 0x61, 0x00);
    wiringPiI2CWriteReg8( $fd, 0x62, 0x00);
    wiringPiI2CWriteReg8( $fd, 0x64, 0x00);
    wiringPiI2CWriteReg8( $fd, 0x65, 0x00);
    wiringPiI2CWriteReg8( $fd, 0x66, 0xA0);

    wiringPiI2CWriteReg8( $fd, 0xFF, 0x01);
    wiringPiI2CWriteReg8( $fd, 0x22, 0x32);
    wiringPiI2CWriteReg8( $fd, 0x47, 0x14);
    wiringPiI2CWriteReg8( $fd, 0x49, 0xFF);
    wiringPiI2CWriteReg8( $fd, 0x4A, 0x00);

    wiringPiI2CWriteReg8( $fd, 0xFF, 0x00);
    wiringPiI2CWriteReg8( $fd, 0x7A, 0x0A);
    wiringPiI2CWriteReg8( $fd, 0x7B, 0x00);
    wiringPiI2CWriteReg8( $fd, 0x78, 0x21);

    wiringPiI2CWriteReg8( $fd, 0xFF, 0x01);
    wiringPiI2CWriteReg8( $fd, 0x23, 0x34);
    wiringPiI2CWriteReg8( $fd, 0x42, 0x00);
    wiringPiI2CWriteReg8( $fd, 0x44, 0xFF);
    wiringPiI2CWriteReg8( $fd, 0x45, 0x26);
    wiringPiI2CWriteReg8( $fd, 0x46, 0x05);
    wiringPiI2CWriteReg8( $fd, 0x40, 0x40);
    wiringPiI2CWriteReg8( $fd, 0x0E, 0x06);
    wiringPiI2CWriteReg8( $fd, 0x20, 0x1A);
    wiringPiI2CWriteReg8( $fd, 0x43, 0x40);

    wiringPiI2CWriteReg8( $fd, 0xFF, 0x00);
    wiringPiI2CWriteReg8( $fd, 0x34, 0x03);
    wiringPiI2CWriteReg8( $fd, 0x35, 0x44);

    wiringPiI2CWriteReg8( $fd, 0xFF, 0x01);
    wiringPiI2CWriteReg8( $fd, 0x31, 0x04);
    wiringPiI2CWriteReg8( $fd, 0x4B, 0x09);
    wiringPiI2CWriteReg8( $fd, 0x4C, 0x05);
    wiringPiI2CWriteReg8( $fd, 0x4D, 0x04);

    wiringPiI2CWriteReg8( $fd, 0xFF, 0x00);
    wiringPiI2CWriteReg8( $fd, 0x44, 0x00);
    wiringPiI2CWriteReg8( $fd, 0x45, 0x20);
    wiringPiI2CWriteReg8( $fd, 0x47, 0x08);
    wiringPiI2CWriteReg8( $fd, 0x48, 0x28);
    wiringPiI2CWriteReg8( $fd, 0x67, 0x00);
    wiringPiI2CWriteReg8( $fd, 0x70, 0x04);
    wiringPiI2CWriteReg8( $fd, 0x71, 0x01);
    wiringPiI2CWriteReg8( $fd, 0x72, 0xFE);
    wiringPiI2CWriteReg8( $fd, 0x76, 0x00);
    wiringPiI2CWriteReg8( $fd, 0x77, 0x00);

    wiringPiI2CWriteReg8( $fd, 0xFF, 0x01);
    wiringPiI2CWriteReg8( $fd, 0x0D, 0x01);

    wiringPiI2CWriteReg8( $fd, 0xFF, 0x00);
    wiringPiI2CWriteReg8( $fd, 0x80, 0x01);
    wiringPiI2CWriteReg8( $fd, 0x01, 0xF8);

    wiringPiI2CWriteReg8( $fd, 0xFF, 0x01);
    wiringPiI2CWriteReg8( $fd, 0x8E, 0x01);
    wiringPiI2CWriteReg8( $fd, 0x00, 0x01);
    wiringPiI2CWriteReg8( $fd, 0xFF, 0x00);
    wiringPiI2CWriteReg8( $fd, 0x80, 0x00);

    wiringPiI2CWriteReg8( $fd, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
            
    $int = wiringPiI2CReadReg8( $fd, GPIO_HV_MUX_ACTIVE_HIGH );
    wiringPiI2CWriteReg8( $fd, GPIO_HV_MUX_ACTIVE_HIGH, $int & ~0x10 );
    wiringPiI2CWriteReg8( $fd, SYSTEM_INTERRUPT_CLEAR, 0x01);

    $self->{measurement_timing_budget_us} = get_measurement_timing_budget();

    wiringPiI2CWriteReg8( $fd, SYSTEM_SEQUENCE_CONFIG, 0xe8);

    set_measurement_timing_budget( $self->{measurement_timing_budget_us} );

    # ref calibration
    wiringPiI2CWriteReg8( $fd, SYSTEM_SEQUENCE_CONFIG, 0x01);
    perform_single_ref_calibration(0x40);
    wiringPiI2CWriteReg8( $fd, SYSTEM_SEQUENCE_CONFIG, 0x02);
    perform_single_ref_calibration(0x00);

    wiringPiI2CWriteReg8( $fd, SYSTEM_SEQUENCE_CONFIG, 0xe8);

    return;

}





    set_timeout(500);

    if ($args{long_range}) {
        set_signal_rate_limit(0.1);
        set_VCSEL_pulse_period( VCSEL_PERIOD_PRE_RANGE,   18 );
        set_VCSEL_pulse_period( VCSEL_PERIOD_FINAL_RANGE, 14 );
    }

    if (defined $args{timing_budget}) {
        if ($args{timing_budget < 20 || $args{timing_budget} > 200) {
            die "Bad timing budget range (accept 20-200)";
        }
        set_measurement_timing_budget( $args{timing_budget} );
    }

while (1) {

    my $r = read_range_single_millimeters();
    say sprintf "%0.1f", $r/25.4;
    #say int(($r - 45)/25.4);

}

exit;


sub set_timeout { $_[0]->{io_timeout} = $_[1] };
sub get_timeout {return $_[0]->{io_timeout}   };

sub start_timeout { $self->{timeout_start_ms} = time() * 1000 } 

sub check_timeout_expired {

    my ($self) = @_;
    return( $self->{io_timeout} > 0
      && (time()*1000 - $self->{timeout_start_ms} > $self->{io_timeout})
    );

}

sub decode_VCSEL_period {

    my ($reg_val) = @_;
    return( ($reg_val + 1) << 1);

}
sub encode_VCSEL_period {

    my ($period) = @_;
    return( ($period >> 1) - 1);

}


sub calc_macro_period {

    my ($period) = @_;

    return( (2304 * $period * 1655 + 500) / 1000 );

}


sub get_sequence_step_enables {

    my $int = wiringPiI2CReadReg8( $fd, SYSTEM_SEQUENCE_CONFIG );
    return {
        tcc         => ($int >> 4) & 0x1,
        dss         => ($int >> 3) & 0x1,
        msrc        => ($int >> 2) & 0x1,
        pre_range   => ($int >> 6) & 0x1,
        final_range => ($int >> 7) & 0x1,
    }

}

sub wiringPiI2CWriteReg16BE {

    my ($fd, $reg, $val) = @_;
    $val = unpack 'S', pack( 'n', $val );
    return wiringPiI2CWriteReg16( $fd, $reg, $val );

}

sub wiringPiI2CReadReg16BE {

    my ($fd, $reg) = @_;
    my $val = wiringPiI2CReadReg16( $fd, $reg );
    return unpack 'n', pack( 'S', $val );

}


sub set_VCSEL_pulse_period {

    my ($type, $period) = @_;

    my $reg = encode_VCSEL_period( $period );

    my $enables = get_sequence_step_enables;
    my $timeouts = get_sequence_step_timeouts( $enables );

    if ($type == VCSEL_PERIOD_PRE_RANGE) {

        my $val = $period == 12 ? 0x18
                : $period == 14 ? 0x30
                : $period == 16 ? 0x40
                : $period == 18 ? 0x50
                : die "bad pulse period";
        wiringPiI2CWriteReg8( $fd, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, $val );
        wiringPiI2CWriteReg8( $fd, PRE_RANGE_CONFIG_VALID_PHASE_LOW,  0x08 );
        wiringPiI2CWriteReg8( $fd, PRE_RANGE_CONFIG_VCSEL_PERIOD, $reg );

        my $new_range_to = timeout_microseconds_to_mclks(
            $timeouts->{pre_range_us}, $period
        );

        wiringPiI2CWriteReg16BE( $fd, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
            encode_timeout($new_range_to) );

        my $new_msrc_to = timeout_microseconds_to_mclks(
            $timeouts->{msrc_dss_tcc_us}, $period
        );

        wiringPiI2CWriteReg16BE( $fd, MSRC_CONFIG_TIMEOUT_MACROP,
            $new_msrc_to > 256 ? 256 : $new_msrc_to - 1);

    }

    elsif ($type == VCSEL_PERIOD_FINAL_RANGE) {

        my @vals = $period ==  8 ? (0x10, 0x02, 0x0c, 0x30)
                 : $period == 10 ? (0x28, 0x03, 0x09, 0x20)
                 : $period == 12 ? (0x38, 0x03, 0x08, 0x20)
                 : $period == 14 ? (0x48, 0x03, 0x07, 0x20)
                 : die "bad pulse period";

        wiringPiI2CWriteReg8( $fd, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, $vals[0] );
        wiringPiI2CWriteReg8( $fd, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08     );
        wiringPiI2CWriteReg8( $fd, GLOBAL_CONFIG_VCSEL_WIDTH,           $vals[1] );
        wiringPiI2CWriteReg8( $fd, ALGO_PHASECAL_CONFIG_TIMEOUT,        $vals[2] );
        wiringPiI2CWriteReg8( $fd, 0xff, 0x01 );
        wiringPiI2CWriteReg8( $fd, ALGO_PHASECAL_LIM,                   $vals[3] );
        wiringPiI2CWriteReg8( $fd, 0xff, 0x00 );

        wiringPiI2CWriteReg8( $fd, FINAL_RANGE_CONFIG_VCSEL_PERIOD, $reg );

        my $new_range_to = timeout_microseconds_to_mclks(
            $timeouts->{final_range_us}, $period
        );

        if ($enables->{pre_range}) {
            $new_range_to += $timeouts->{pre_range_mclks};
        }

        wiringPiI2CWriteReg16BE( $fd, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
            encode_timeout($new_range_to) );

        set_measurement_timing_budget( $self->{measurement_timing_budget_us} );
        my $cfg = wiringPiI2CReadReg8( $fd, SYSTEM_SEQUENCE_CONFIG );
        wiringPiI2CWriteReg8( $fd, SYSTEM_SEQUENCE_CONFIG, 0x02 );
        perform_single_ref_calibration(0x0);
        wiringPiI2CWriteReg8( $fd, SYSTEM_SEQUENCE_CONFIG, $cfg );

    }

    else {
        die;
    }

    return 1;

}

sub get_VCSEL_pulse_period {

    my ($type) = @_;

    if ($type == VCSEL_PERIOD_PRE_RANGE) {

        my $period
            = wiringPiI2CReadReg8( $fd, PRE_RANGE_CONFIG_VCSEL_PERIOD );
        return decode_VCSEL_period( $period );

    }

    elsif ($type == VCSEL_PERIOD_FINAL_RANGE) {

        my $period
            = wiringPiI2CReadReg8( $fd, FINAL_RANGE_CONFIG_VCSEL_PERIOD );
        return decode_VCSEL_period( $period );

    }

    else {
        die;
    }

}

sub read_range_continuous_millimeters {

    start_timeout();

    while ((wiringPiI2CReadReg8( $fd, RESULT_INTERRUPT_STATUS ) & 0x07) == 0) {
        if (check_timeout_expired()) {
            return 65535;
        }
    }

    my $range = wiringPiI2CReadReg16BE( $fd, RESULT_RANGE_STATUS + 10 );
    wiringPiI2CWriteReg8( $fd, SYSTEM_INTERRUPT_CLEAR, 0x01 );

    return $range;

}

sub read_range_single_millimeters {

    wiringPiI2CWriteReg8( $fd, 0x80, 0x01 );
    wiringPiI2CWriteReg8( $fd, 0xff, 0x01 );
    wiringPiI2CWriteReg8( $fd, 0x00, 0x00 );
    wiringPiI2CWriteReg8( $fd, 0x91, $self->{stop_var} );
    wiringPiI2CWriteReg8( $fd, 0x00, 0x01 );
    wiringPiI2CWriteReg8( $fd, 0xff, 0x00 );
    wiringPiI2CWriteReg8( $fd, 0x80, 0x00 );

    wiringPiI2CWriteReg8( $fd, SYSRANGE_START, 0x01 );

    start_timeout();
    while (wiringPiI2CReadReg8( $fd, SYSRANGE_START ) & 0x01) {
        if (check_timeout_expired()) {
            return 65535;
        }
    }

    return read_range_continuous_millimeters();

}

sub get_sequence_step_timeouts {


    my ($enables) = @_;

    my $timeouts = {};

    $timeouts->{pre_range_vcsel_period_pclks}
        = get_VCSEL_pulse_period( VCSEL_PERIOD_PRE_RANGE );
    $timeouts->{msrc_dss_tcc_mclks}
        = wiringPiI2CReadReg8( $fd, MSRC_CONFIG_TIMEOUT_MACROP ) + 1;
    $timeouts->{msrc_dss_tcc_us}
        = timeout_mclks_to_microseconds(
            $timeouts->{msrc_dss_tcc_mclks},
            $timeouts->{pre_range_vcsel_period_pclks}
        );
    my $to = wiringPiI2CReadReg16BE( $fd, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI );
    $timeouts->{pre_range_mclks}
        = decode_timeout( $to ); 
    $timeouts->{pre_range_us}
        = timeout_mclks_to_microseconds(
            $timeouts->{pre_range_mclks},
            $timeouts->{pre_range_vcsel_period_pclks}
        );
    $timeouts->{final_range_vcsel_period_pclks}
        = get_VCSEL_pulse_period( VCSEL_PERIOD_FINAL_RANGE );

    $to = wiringPiI2CReadReg16BE( $fd, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI );
    $timeouts->{final_range_mclks}
        = decode_timeout( $to ); 

    if ($enables->{pre_range}) {
        $timeouts->{final_range_mclks} -= $timeouts->{pre_range_mclks};
    }
    $timeouts->{final_range_us}
        = timeout_mclks_to_microseconds(
            $timeouts->{final_range_mclks},
            $timeouts->{final_range_vcsel_period_pclks}
        );
        

    return $timeouts;

}

sub decode_timeout {

    my ($reg) = @_;

    return (($reg & 0x00ff) << (($reg & 0xff00) >> 8)) + 1;

}

sub encode_timeout {

    my ($to) = @_;

    return 0 if ($to <= 0);

    my $lsb = $to - 1;
    my $msb = 0;

    while (($lsb & 0xffffff00) > 0) {
        $lsb >>= 1;
        $msb++;
    }

    return ($msb << 8) | ($lsb & 0xff);


}

sub timeout_mclks_to_microseconds {

    my ($mclks, $pclks) = @_;

    my $period = calc_macro_period( $pclks );

    return (($mclks * $period) + ($period/2)) / 1000;

}

sub timeout_microseconds_to_mclks {

    my ($to, $pclks) = @_;

    my $period = calc_macro_period( $pclks );

    return  ((($to * 1000) + ($period/2)) / $period);

}

    

sub set_measurement_timing_budget {

    my ($budget) = @_;

    use constant START_OVERHEAD_2 => 1320;
    use constant END_OVERHEAD   => 960;
    use constant MSRC_OVERHEAD  => 660;
    use constant TCC_OVERHEAD   => 590;
    use constant DSS_OVERHEAD   => 690;
    use constant PRE_RANGE_OVERHEAD   => 660;
    use constant FINAL_RANGE_OVERHEAD => 550;

    use constant MIN_TIMING_BUDGET => 20000;

    return 0 if ($budget < MIN_TIMING_BUDGET);

    my $used_budget = START_OVERHEAD_2 + END_OVERHEAD;

    my $enables = get_sequence_step_enables;
    my $timeouts = get_sequence_step_timeouts( $enables );

    if ($enables->{tcc}) {
        $used_budget += $timeouts->{msrc_dss_tcc_us} + TCC_OVERHEAD;
    }

    if ($enables->{dss}) {
        $used_budget += 2 * ($timeouts->{msrc_dss_tcc_us} + DSS_OVERHEAD);
    }
    elsif ($enables->{msrc}) {
        $used_budget += $timeouts->{msrc_dss_tcc_us} + MSRC_OVERHEAD;
    }

    if ($enables->{pre_range}) {
        $used_budget += $timeouts->{pre_range_us} + PRE_RANGE_OVERHEAD;
    } 
    if ($enables->{final_range}) {
        $used_budget += FINAL_RANGE_OVERHEAD;
    } 

    return 0 if ($used_budget > $budget);

    my $final_range_timeout = $budget - $used_budget;

    my $final_range_timeout_mclks
        = timeout_microseconds_to_mclks(
            $final_range_timeout, 
            $timeouts->{final_range_vcsel_period_pclks}
        );

    if ($enables->{pre_range}) {
        $final_range_timeout_mclks += $timeouts->{pre_range_mclks};
    }
       
    wiringPiI2CWriteReg16BE( $fd, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
        encode_timeout($final_range_timeout_mclks) );

    $self->{measurement_timing_budget_us} = $budget;

    return 1;

}

sub get_measurement_timing_budget {

    use constant START_OVERHEAD => 1910;
    use constant END_OVERHEAD   => 960;
    use constant MSRC_OVERHEAD  => 660;
    use constant TCC_OVERHEAD   => 590;
    use constant DSS_OVERHEAD   => 690;
    use constant PRE_RANGE_OVERHEAD   => 660;
    use constant FINAL_RANGE_OVERHEAD => 550;

    my $budget = START_OVERHEAD + END_OVERHEAD;

    my $enables = get_sequence_step_enables;
    my $timeouts = get_sequence_step_timeouts( $enables );

    if ($enables->{tcc}) {
        $budget += $timeouts->{msrc_dss_tcc_us} + TCC_OVERHEAD;
    }

    if ($enables->{dss}) {
        $budget += 2 * ($timeouts->{msrc_dss_tcc_us} + DSS_OVERHEAD);
    }
    elsif ($enables->{msrc}) {
        $budget += $timeouts->{msrc_dss_tcc_us} + MSRC_OVERHEAD;
    }

    if ($enables->{pre_range}) {
        $budget += $timeouts->{pre_range_us} + PRE_RANGE_OVERHEAD;
    } 
    if ($enables->{final_range}) {
        $budget += $timeouts->{final_range_us} + FINAL_RANGE_OVERHEAD;
    } 

    $self->{measurement_timing_budget_us} = $budget;

    return $budget;

}

sub set_signal_rate_limit {
    
    my ($rate) = @_;

    return 0 if ($rate > 0 || $rate > 511.99);

    # do we need to flip bits here?
    wiringPiI2CWriteReg16BE( $fd, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT,
        $rate * (1 << 7) );
    return 1;

}

sub get_signal_rate_limit {
    
    # do we need to flip bits here?
    return wiringPiI2CReadReg16BE( $fd, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT );

}

sub _get_spad_info {

    my ($fd) = @_;

    wiringPiI2CWriteReg8( $fd, 0x80, 0x01 );
    wiringPiI2CWriteReg8( $fd, 0xff, 0x01 );
    wiringPiI2CWriteReg8( $fd, 0x00, 0x00 );
    wiringPiI2CWriteReg8( $fd, 0xff, 0x06 );
    my $int = wiringPiI2CReadReg8( $fd, 0x83 );
    wiringPiI2CWriteReg8( $fd, 0x83, $int | 0x04 );
    wiringPiI2CWriteReg8( $fd, 0xff, 0x07 );
    wiringPiI2CWriteReg8( $fd, 0x81, 0x01 );

    wiringPiI2CWriteReg8( $fd, 0x80, 0x01 );
    wiringPiI2CWriteReg8( $fd, 0x94, 0x6b );
    wiringPiI2CWriteReg8( $fd, 0x83, 0x00 );
    start_timeout();
    while ( wiringPiI2CReadReg8( $fd, 0x83 ) == 0x00) {

        return 0 if (check_timeout_expired());

    }
    wiringPiI2CWriteReg8( $fd, 0x83, 0x01 );
    my $tmp = wiringPiI2CReadReg8( $fd, 0x92 );

    my $count = $tmp & 0x7f;
    my $type_is_aperature = ($tmp >> 7) & 0x01;

    wiringPiI2CWriteReg8( $fd, 0x81, 0x00 );
    wiringPiI2CWriteReg8( $fd, 0xff, 0x06 );
    $int = wiringPiI2CReadReg8( $fd, 0x83 );
    wiringPiI2CWriteReg8( $fd, 0x83, $int & ~0x04 );
    wiringPiI2CWriteReg8( $fd, 0xff, 0x01 );
    wiringPiI2CWriteReg8( $fd, 0x00, 0x01 );

    wiringPiI2CWriteReg8( $fd, 0xff, 0x00 );
    wiringPiI2CWriteReg8( $fd, 0x80, 0x00 );

    return ($count, $type_is_aperature);

}


sub write_multi {

    my ($reg, $array, $len) = @_;

    for my $int (@$array) {
        wiringPiI2CWriteReg8( $fd, $reg++, $int);
    }

}


sub perform_single_ref_calibration {

    my ($byte) = @_;

    wiringPiI2CWriteReg8( $fd, SYSRANGE_START,  0x01 | $byte );

    start_timeout();

    while ( (wiringPiI2CReadReg8( $fd, RESULT_INTERRUPT_STATUS) & 0x07 ) == 0) {
        return 0 if (check_timeout_expired());
    }

    wiringPiI2CWriteReg8( $fd, SYSTEM_INTERRUPT_CLEAR,  0x01 );
    wiringPiI2CWriteReg8( $fd, SYSRANGE_START,  0x00 );

    return 1;

}




sub clean_quit {

    #for my $pin (keys %$pins) {
        #pinMode($pins->{$pin} => INPUT);
    #}
    exit;

}
