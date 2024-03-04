#!/bin/bash
# input_file=$1
# if [ -z "$1" ] 
# then
# input_file=sw/basic-c/main
# fi

# Define the directories and application name lists
directory1="sw"
directory2="../embench-iot/bd_O3/src"
app_list1=("asm-anti" "asm-two-branches" "basic-asm-test")
app_list2=("md5sum" "crc32" "aha-mont64" "edn" "huffbench" "matmult-int" "tarfind" "ud" "minver" "nettle-aes" "nettle-sha256" "nsichneu" "picojpeg" "primecount" "qrduino" "sglib-combined" "slre")

default_app="main"


run_application() {
  local input_file="$1"
  local command="./vp/build/bin/tiny32-vp --intercept-syscalls '$input_file' --output-file './out/' --seq"
  echo "Executing: $command"
  eval "$command"
}

for app in "${app_list1[@]}"; do
  input_file="$directory1/$app/${app:-$default_app}"
  run_application "$input_file"
done

for app in "${app_list2[@]}"; do
  input_file="$directory2/$app/${app:-$default_app}"
  run_application "$input_file"
done
