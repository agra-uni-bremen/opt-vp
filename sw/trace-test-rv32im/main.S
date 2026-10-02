# Executes every RV32IM instruction that tests/trace/model.py describes, so the reference
# model can check the trace of each one. The loop runs three times: every branch is taken on
# some iterations and not taken on others at the same pc, and every window occurs more than
# once, which is what true_weight counts.
.globl _start

.macro SYS_EXIT, exit_code
addi a7, zero, 93
addi a0, zero, \exit_code
ecall
.endm

_start:
	li   s0, 0              # loop counter
	li   s1, 3              # iterations
	la   s2, buffer
	la   s3, compute        # called through a register other than ra
loop:
	jalr ra, 0(s3)
	call memory
	call branches
	addi s0, s0, 1
	blt  s0, s1, loop
	SYS_EXIT 0

compute:
	addi  t0, s0, 7
	slti  t1, t0, 9
	sltiu t2, t0, -1
	xori  t3, t0, 0x55
	ori   t4, t0, 0x100
	andi  t5, t0, 0x0f
	slli  t6, t0, 4
	srli  a0, t6, 2
	srai  a1, t6, 1
	add   a2, t0, t1
	sub   a3, t0, t1
	sll   a4, t0, t1
	slt   a5, t1, t0
	sltu  a6, t0, t1
	xor   a7, t0, t3
	srl   a2, t6, t1
	sra   a3, t6, t1
	or    a4, t0, t5
	and   a5, t0, t4
	lui   a6, 0x12345
	auipc a7, 0
	mul    t1, t0, t0
	mulh   t2, t0, t3
	mulhsu t3, t0, t1
	mulhu  t4, t0, t1
	div    t5, t1, t0
	divu   t6, t1, t0
	rem    a0, t1, t3
	remu   a1, t1, t3
	ret

memory:
	sw   s0, 0(s2)
	sh   s0, 4(s2)
	sb   s0, 6(s2)
	lw   t0, 0(s2)
	lh   t1, 4(s2)
	lhu  t2, 4(s2)
	lb   t3, 6(s2)
	lbu  t4, 6(s2)
	add  t5, t0, t1
	sw   t5, 8(s2)
	sw   t2, 8(s2)
	lw   t6, 8(s2)
	ret

branches:
	# s0 is 0, 1 and 2 on the three calls, so each branch below changes direction.
	li   t0, 1
	beq  s0, t0, 1f
	addi t1, t1, 1
1:	bne  s0, t0, 2f
	addi t1, t1, 1
2:	blt  s0, t0, 3f
	addi t1, t1, 1
3:	bge  s0, t0, 4f
	addi t1, t1, 1
4:	bltu s0, t0, 5f
	addi t1, t1, 1
5:	bgeu s0, t0, 6f
	addi t1, t1, 1
6:	j    7f
	addi t1, t1, 100        # never runs
7:	ret

.data
.align 4
buffer: .space 16
