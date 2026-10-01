MAKEFLAGS += --no-print-directory

# Whether to use a system-wide SystemC library instead of the vendored one.
USE_SYSTEM_SYSTEMC ?= OFF

BUILD_TYPE ?= Release

# Enable gprof profiling flags when PG=ON (use: `make PG=ON ...`).
# The flags are passed on every configure, empty included, so turning PG off again removes them.
PG ?= OFF
ifeq ($(PG),ON)
PG_FLAGS := -pg
else
PG_FLAGS :=
endif

VP_CMAKE_FLAGS := -DCMAKE_BUILD_TYPE=$(BUILD_TYPE) -DUSE_SYSTEM_SYSTEMC=$(USE_SYSTEM_SYSTEMC) \
	-DCMAKE_C_FLAGS="$(PG_FLAGS)" -DCMAKE_CXX_FLAGS="$(PG_FLAGS)" \
	-DCMAKE_EXE_LINKER_FLAGS="$(PG_FLAGS)" -DCMAKE_SHARED_LINKER_FLAGS="$(PG_FLAGS)"
# What the build tree was last configured with. Settings live in the CMake cache, so without
# comparing them a later `make PG=ON` or `make BUILD_TYPE=Debug` reused the first build's
# settings and said nothing.
VP_SETTINGS := BUILD_TYPE=$(BUILD_TYPE) USE_SYSTEM_SYSTEMC=$(USE_SYSTEM_SYSTEMC) PG=$(PG)

vps: vp/src/core/common/gdb-mc/libgdb/mpc/mpc.c vp-configure
	$(MAKE) install -C vp/build

vp/src/core/common/gdb-mc/libgdb/mpc/mpc.c:
	git submodule update --init vp/src/core/common/gdb-mc/libgdb/mpc

all: vps vp-display

# Configure vp/build, and reconfigure it whenever a setting above changed. This make file owns
# vp/build; configure a tree with other flags somewhere else, as vp/CMakeLists.txt describes.
vp-configure:
	@mkdir -p vp/build
	@if [ "`cat vp/build/.make-settings 2>/dev/null`" != "$(VP_SETTINGS)" ]; then \
		echo "Configuring vp/build for $(VP_SETTINGS)"; \
		cd vp/build && cmake $(VP_CMAKE_FLAGS) .. && echo "$(VP_SETTINGS)" > .make-settings; \
	fi

vp-eclipse:
	mkdir -p vp-eclipse
	cd vp-eclipse && cmake ../vp/ -G "Eclipse CDT4 - Unix Makefiles"

env/basic/vp-display/build/Makefile:
	mkdir -p env/basic/vp-display/build
	cd env/basic/vp-display/build && cmake -DCMAKE_C_FLAGS="$(PG_FLAGS)" -DCMAKE_CXX_FLAGS="$(PG_FLAGS)" ..

vp-display: env/basic/vp-display/build/Makefile
	$(MAKE) -C env/basic/vp-display/build

scoring-functions: vp-configure
	cd vp/src/scoring_functions && cmake .
	$(MAKE) -C vp/src/scoring_functions

# Phony targets
.PHONY: pg half essential vp-configure vps all vp-display scoring-functions

# Build everything with -pg
pg:
	$(MAKE) PG=ON half

# A subset of the platform binaries, without virtual-breadboard, hwitl, test32 and the Qt
# display. vp/CMakeLists.txt lists what `minimal` and `essential` contain.
half: vp-configure
	cmake --build vp/build --target minimal --parallel $(shell nproc)
	@echo "Built minimal set (excluding vp-display/qt, virtual-breadboard, hwitl, test32)"

essential: vp-configure
	cmake --build vp/build --target essential --parallel $(shell nproc)
	@echo "Built essential set of vps (riscv-vp, linux32-vp, tiny32-vp, tiny64-vp, microrv32-vp)"

vp-clean:
	rm -rf vp/build

qt-clean:
	rm -rf env/basic/vp-display/build

clean-all: vp-clean qt-clean

clean: vp-clean

# Format this repository's own sources. The name tests have to be grouped: with -print bound to
# the last -o branch only, the old form printed .cpp files and no header at all, and it descended
# into the vendored trees.
codestyle:
	find vp/src \( -path vp/src/vendor -o -path vp/src/lib \) -prune -o \( -name '*.h' -o -name '*.hpp' -o -name '*.cpp' \) -print | xargs clang-format -i -style=file
