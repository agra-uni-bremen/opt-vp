#!/bin/bash
input_file=$1
margs=$2
if [ -z "$1" ] 
then
input_file=sw/basic-c/main
fi

time ./vp/build/bin/tiny32-vp --intercept-syscalls $input_file --output-file ./out/ -e $margs

