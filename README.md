# Opt-VP 
 an extension to the RISC-V based Virtual Prototype (VP)

<p align="center">
  <img src="./img/riscv-vp_logo.png" alt="RISC-V based Virtual Prototype (VP)" width="250"/>
</p>


Most hardware in the area of IoT and embedded systems only ever runs a single application.  
To reduce the cost and increase performance the hardware can be tailored to this application.  
Unfortunately, identifying, designing, and evaluating application-specific optimizations is complex and requires significant effort.

In order to combine the advantages of high-level and lowlevel approaches we propose this Virtual Prototype to automatically identify promising hardware optimization candidates based on recurring patterns. 



For information about the base RISC-V VP please visit [the base repository](https://github.com/agra-uni-bremen/riscv-vp)

## :dart: Additional Features

* Tracing of any RISC-V binary compiled for RV32IMAC 
* Generation of bounded execution trees for every instruction
* Support for arbitrary scoring function for the analysis
* Ouput dot visualization of iternal trees
* export best sequences as json
* full csv export for trees
* reload scoring functions during runtime

## :rocket: Getting Started

### :wrench:Building the VP ### 
Check out all submodules
```console
$ git submodule update --init --recursive
```
Afterwards
```console
$ make
```
will build the VP

For dependencies and detailed instruction on how to build the Opt-Vp please follow the instructions from the base RISC-V VP

To uninstall/clean use
```console
$ make clean
```

### :computer: Usage

The **command line interface** of the Opt-VP can be used according to the following scheme:

```console
$ ./vp/build/bin/tiny32-vp --intercept-syscalls <executable> --output-file <output-directory>
```

You can find examples on how to run and analyze programs in the ./run_and_dot.sh script:
```
  ./vp/build/bin/tiny32-vp --intercept-syscalls $input_file --output-file ./out/ --dot
```

Additional arguments include 
* `--csv` to export all trees as csv files
* `--dot` to output dot files for the internal trees
* `--seq` to output the best sequences used in [opt-seq](https://github.com/agra-uni-bremen/opt-seq)
* `-i` to enter interactive mode after the simulation has finished


#### Acknowledgements:

The Opt-VP extension was supported in part by the German Federal Ministry of
Education and Research (BMBF) within projects Scale4Edge under grant no.
16ME0127, ECXL under grant no. 01IW22002 and VE-HEP under grant no. 16KIS1342.

