# The same five instructions as trace-test-minimal, built for rv64 and run on tiny64-vp.
# It exists so the 32 bit and 64 bit instruction set simulators cannot drift apart without a
# test saying so. main.S is a symlink: there is one copy of the program, not two.
OBJECTS      = main.o
RISCV_PREFIX = riscv64-unknown-elf-
CFLAGS       = -march=rv64i -mabi=lp64
LDFLAGS      = -nostdlib -Wl,--no-relax
VP           = tiny64-vp
VP_FLAGS     = --intercept-syscalls --error-on-zero-traphandler=true

include ../Makefile.common
