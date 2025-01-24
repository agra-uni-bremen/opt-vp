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
add a1, a0, a1
and a2, a2, a1
or a3, a3, a1
addi a5, a5, 1
xori a5, a6, 2
andi a5, a7, 3
ori a5, t1, 4
mul a5, t2, t2
xori a5, a1, 2
xori a5, a1, 2
xori a5, a1, 2
xori a5, a1, 2
xori a5, a1, 2
xori a5, a1, 2
xori a5, a1, 2
xori a5, a1, 2
xori a5, a1, 2
xori a5, a1, 2
xori a5, a1, 2
xori a5, a1, 2
xori a5, a1, 2
xori a5, a1, 2
add a5, a5, a5
ret






end:

# call exit (SYS_EXIT=93) with exit code 0 (argument in a0)
SYS_EXIT 0

.data
test: .word 42