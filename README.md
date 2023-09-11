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


## Building the VP ## 
To build the Opt-Vp please follow the instructions from the base RISC-V VP

##### Running examples

You can find examples on how to run and analyze programs in the ./run_and_dot.sh script:

```
  ./vp/build/bin/tiny32-vp --intercept-syscalls $input_file --output-file ./out/
```

#### Acknowledgements:

The Opt-VP extension was supported in part by the German Federal Ministry of
Education and Research (BMBF) within projects Scale4Edge under grant no.
16ME0127, ECXL under grant no. 01IW22002 and VE-HEP under grant no. 16KIS1342.
