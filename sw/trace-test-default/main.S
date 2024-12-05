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

call func2

call func3

j end

func1:
add a2, a0, a1
and a3, a2, a1
or a4, a3, a1
addi a5, a5, 1
addi a5, a6, 2
addi a5, a7, 3
addi a5, t1, 4
addi a5, t2, 5
ret

func2:
 bge a0, a1, func2_end
 addi a0, a0, 1
 xor a2, a0, a2
 j func2
func2_end: 
ret

func3:
addi a0,zero,0 
addi a1,zero,20
func3_loop:
 bge a0, a1, func3_end
 addi a0, a0, 1
 lw t4, test
 add t4, t4, t4
 la s0, test
 sw t4, (s0)
 j func3_loop
func3_end:
ret






end:

# call exit (SYS_EXIT=93) with exit code 0 (argument in a0)
SYS_EXIT 0

.data
test: .word 42