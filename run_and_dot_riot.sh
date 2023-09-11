#!/bin/bash
input_file=$1
if [ -z "$1" ] 
then
input_file=../RIOT/examples/hello-world/bin/hifive1/hello-world.elf 
fi

./vp/build/bin/hifive-vp $input_file --output-file ./out/
find ./out/ -type f -name "*.dot" -exec sh -c 'dot -Tpng "${0}" -o "${0%.*}_graph.png"' {} \;

