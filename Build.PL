use 5.006;
use strict;
use warnings FATAL => 'all';
use Module::Build;

my $builder = Module::Build->new(
    module_name         => 'wiringpi',
    license             => 'Artistic_2_0',
    dist_author         => q{Jeremy Volkening <jdv@base2bio.com>},
    dist_version        => 0.001,
    release_status      => 'stable',
    dist_abstract       => 'wiringpi',
    configure_requires => {
        'Module::Build' => 0,
    },
    build_requires => {
        'Test::More' => 0,
    },
    requires => {
    },
    add_to_cleanup     => [
        'wiringpi-*',
        '*.o',
        'wiringpi.pm',
        'wiringpi.so',
        'wiringpi_wrap.c',
    ],
);

my $ret = system('./build.sh');
die "Error building SWIG bindings: $!\n" if ($ret);
$builder->add_build_element('so');
$builder->create_build_script();
