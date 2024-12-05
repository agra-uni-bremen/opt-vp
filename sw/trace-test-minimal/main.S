.globl _start

.macro SYS_EXIT, exit_code
addi a7, zero, 93
addi a0,zero, 0
ecall
.endm


# program entry-point
_start:

addi a0,zero,50
addi a0,zero,100


end:

# call exit (SYS_EXIT=93) with exit code 0 (argument in a0)
SYS_EXIT 0
