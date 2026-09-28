OBJECTS  = main.o
CFLAGS   = -march=rv32im -mabi=ilp32
LDFLAGS  = -nostdlib -Wl,--no-relax
VP_FLAGS = --intercept-syscalls --error-on-zero-traphandler=true

include ../Makefile.common
