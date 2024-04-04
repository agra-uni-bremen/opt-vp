#!/bin/bash
input_file=$1
if [ -z "$1" ] 
then
input_file=sw/basic-c/main
fi

time ./vp/build/bin/tiny32-vp --intercept-syscalls $input_file --output-file ./out/ --dot --csv --seq
find ./out/ -type f -name "*.dot" -exec sh -c 'dot -Tpng "${0}" -o "${0%.*}_graph.png"' {} \;

