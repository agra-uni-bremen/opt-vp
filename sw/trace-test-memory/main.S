.globl _start

.macro SYS_EXIT, exit_code
addi a7, zero, 93
addi a0,zero, 0
ecall
.endm


# program entry-point
_start:

addi a0,zero,50
addi a1,zero,200

call func1

j end

func1:
lui t2, 42
lw t1, test
addi t1, t1, 1
la s0, test
sw t1, (s0)
sw t2 , (s0)
lw t3, test
ret

ret






end:

# call exit (SYS_EXIT=93) with exit code 0 (argument in a0)
SYS_EXIT 0

.data
test: .word 42