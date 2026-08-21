#!/bin/bash

directory2="../embench-iot/bd/src"
directory3="../tflite-micro/gen/riscv32_generic_x86_64_default_gcc/bin"

app_list2=(
  "md5sum"
  "crc32"
  "aha-mont64"
  "edn"
  "huffbench"
  "matmult-int"
  "tarfind"
  "ud"
  "minver"
  "nettle-aes"
  "nettle-sha256"
  "nsichneu"
  "picojpeg"
  "primecount"
  "qrduino"
  "sglib-combined"
  "slre"
  "depthconv"
)

app_list3=("image_classification_cat" "mlcommon-ad")

default_app="main"

run_application() {
  local input_file="$1"
  local app_name="$2"
  local out_dir="./out/test-set-extended/$app_name"
  shift 2

  mkdir -p "$out_dir"

  local command=(
    ./vp/build/bin/tiny32-vp
    --intercept-syscalls "$input_file"
    --output-file "$out_dir/"
    -e
  )

  if [ "$#" -gt 0 ]; then
    command+=("$@")
  fi

  echo "Executing: ${command[*]}"
  "${command[@]}"
}

for app in "${app_list2[@]}"; do
  input_file="$directory2/$app/${app:-$default_app}"
  run_application "$input_file" "$app" "$@"
done

for app in "${app_list3[@]}"; do
  input_file="$directory3/$app"
  run_application "$input_file" "$app" "$@"
done