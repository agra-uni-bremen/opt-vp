# Opt-VP 
 an extension to the RISC-V based Virtual Prototype (VP)

<p align="center">
  <img src="./img/riscv-vp_logo.png" alt="RISC-V based Virtual Prototype (VP)" width="250"/>
</p>


Most hardware in the area of IoT and embedded systems only ever runs a single application.  
To reduce the cost and increase performance the hardware can be tailored to this application.  
Unfortunately, identifying, designing, and evaluating application-specific optimizations is complex and requires significant effort.

In order to combine the advantages of high-level and lowlevel approaches we propose this Virtual Prototype to automatically identify promising hardware optimization candidates based on recurring patterns. 

----

For information about the base RISC-V VP please visit [the base repository](https://github.com/agra-uni-bremen/riscv-vp)

If you are interested in this project please take a look at the related [publications](#publications) or feel free to reach out to one of the corresponding authors  

----

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
* `--trace-depth <n>` to shorten the traced sequences (default and maximum: the compiled tree depth)

### :package: Trace contents

Every node of an exported tree carries, next to its weight and dependencies:

* `register_sets`: per program counter the registers used at that pc (`rs1`, `rs2`, `rd`), how often
  it was reached (`count`) and, per predecessor pc, how often it was reached *from* that pc
  (`predecessors`). The predecessors make the pc path through a sequence provable later.
* `parameters`: per program counter the values the instruction was executed with, as
  `[[pc, [[value, count], ...]], ...]`. This is the shift amount for shifts, the target pc for taken
  branches and jumps, and the decoded immediate for every other instruction that carries one
  (`ADDI`, `ANDI`, `LUI`, load/store offsets, ...). Values are signed 64 bit.
* Branch and jump nodes additionally carry `BranchOutcomes`: per program counter the encoded
  `offset` and how often the branch was `taken`/`not_taken`. `JALR` reports its `I_imm`, which is
  relative to `rs1` rather than to the pc, and is therefore not included in the pc relative
  `Direction`/`offsets` summary.
* Load and store nodes additionally carry their accesses, offset sum and, if a registered peripheral
  was hit, the peripheral names.

### :hammer: Build options

The tree depth and the optional parts of the trace are fixed at compile time, because the nodes keep
static arrays for them:

```console
$ cmake -S vp -B vp/build -DINSTRUCTION_TREE_DEPTH=8
```

| Option | Effect |
| --- | --- |
| `-DINSTRUCTION_TREE_DEPTH=<n>` | maximum sequence length (default 20). `--trace-depth` can lower it at runtime |
| `-DNO_TRACE_PARAMETER_IMMEDIATES=ON` | do not record decoded immediates |
| `-DNO_TRACE_PREDECESSOR_PCS=ON` | do not record predecessor pcs |
| `-DNO_TRACE_BRANCH_OUTCOMES=ON` | do not record per pc branch outcomes |
| `-DTRACE_ROOT_PARAMETERS=ON` | also record parameters on the root node of each tree |

## Publications  
The concepts behind the Opt-VP are further described in the following publications:  
#### [Paper introducing the VP](https://ieeexplore.ieee.org/abstract/document/10272131)  
#### [Paper with latest results](https://ieeexplore.ieee.org/abstract/document/11539431/)  

   
#### Acknowledgements:

The Opt-VP extension was supported in part by the German Federal Ministry of Research, Technology and Space (BMFTR) within projects Scale4Edge under grant no.
16ME0127, ECXL under grant no. 01IW22002 and VE-HEP under grant no. 16KIS1342.

