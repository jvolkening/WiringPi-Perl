#!/bin/bash
echo "Updating submodule..."
git submodule update --init

mkdir -p lib

echo "Generating bindings..."
swig -perl -const -outdir lib WiringPi.i
