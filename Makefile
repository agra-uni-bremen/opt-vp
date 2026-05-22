MAKEFLAGS += --no-print-directory

# Whether to use a system-wide SystemC library instead of the vendored one.
USE_SYSTEM_SYSTEMC ?= OFF

BUILD_TYPE ?= Debug

# Enable gprof profiling flags when PG=ON (use: `make PG=ON ...`).
PG ?= OFF
ifeq ($(PG),ON)
PG_CMAKE_FLAGS := -DCMAKE_C_FLAGS="-pg" -DCMAKE_CXX_FLAGS="-pg" -DCMAKE_EXE_LINKER_FLAGS="-pg" -DCMAKE_SHARED_LINKER_FLAGS="-pg"
else
PG_CMAKE_FLAGS :=
endif

vps: vp/src/core/common/gdb-mc/libgdb/mpc/mpc.c vp/build/Makefile
	$(MAKE) install -C vp/build

vp/src/core/common/gdb-mc/libgdb/mpc/mpc.c:
	git submodule update --init vp/src/core/common/gdb-mc/libgdb/mpc

all: vps vp-display

vp/build/Makefile:
	mkdir -p vp/build
	cd vp/build && cmake -DCMAKE_BUILD_TYPE=$(BUILD_TYPE) -DUSE_SYSTEM_SYSTEMC=$(USE_SYSTEM_SYSTEMC) $(PG_CMAKE_FLAGS) ..

vp-eclipse:
	mkdir -p vp-eclipse
	cd vp-eclipse && cmake ../vp/ -G "Eclipse CDT4 - Unix Makefiles"

env/basic/vp-display/build/Makefile:
	mkdir -p env/basic/vp-display/build
	cd env/basic/vp-display/build && cmake $(PG_CMAKE_FLAGS) ..

vp-display: env/basic/vp-display/build/Makefile
	$(MAKE) -C env/basic/vp-display/build

scoring-functions:
	mkdir -p vp/build
	cd vp/build && cmake -DCMAKE_BUILD_TYPE=$(BUILD_TYPE) -DUSE_SYSTEM_SYSTEMC=$(USE_SYSTEM_SYSTEMC) $(PG_CMAKE_FLAGS) ..
	cd vp/src/scoring_functions && cmake .
	$(MAKE) -C vp/src/scoring_functions

# Phony targets
.PHONY: pg half essential

# Build everything with -pg
pg:
	$(MAKE) PG=ON half

# Minimal build: build a subset of vp targets and avoid installing virtual-breadboard, hwitl, test32, and the Qt display.
# This configures the build tree (if needed) then builds selected vp targets only.
MINIMAL_VP_TARGETS := riscv-vp linux-vp linux32-vp tiny32-vp tiny64-vp tiny32-mc tiny64-mc microrv32-vp

half: vp/build/Makefile
	@echo "Configuring vp build (if necessary)"
	@cd vp/build || exit 1
	@echo "Building vp target: minimal"
	cmake --build vp/build --target minimal --parallel $(shell nproc)
	@echo "Built minimal set (excluding vp-display/qt, virtual-breadboard, hwitl, test32)"

essential: vp/build/Makefile
	@cd vp/build || exit 1
	@echo "Building vp target: essential"
	cmake --build vp/build --target essential --parallel $(shell nproc)
	@echo "Built essential set of vps(riscv-vp, linux32-vp, tiny32-vp, microrv32-vp)"

vp-clean:
	rm -rf vp/build

qt-clean:
	rm -rf env/basic/vp-display/build

clean-all: vp-clean qt-clean

clean: vp-clean

codestyle:
	find . -type d \( -name .git -o -name dependencies \) -prune -o -name '*.h' -o -name '*.hpp' -o -name '*.cpp' -print | xargs clang-format -i -style=file
