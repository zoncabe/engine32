# --- engine32 -----------------------------------------------------------------
# Build of a game on the engine for the PlayStation. The toolchain is
# mipsel-none-elf, the runtime is nugget (crt0, linker scripts, hardware and
# kernel headers) and the library on top is PSYQo. Modeled on ultra64.mk, and
# on the Makefiles nugget's own examples use.
#
# Consumer contract, before including this file:
#   ENGINE_DIR     path to this repo.
#   PROJECT_NAME   name of the executable.
#   NUGGET_DIR     the nugget checkout (default: next to the engine).
#   src_extra      the game's sources outside its directory (optional).
#   cflags_extra   the game's own compiler flags, include paths and
#                  defines (optional).
#   collision_models  the models under assets/models that also collide,
#                  by name without extension (optional).
#
# Provides everything else: the game's own *.cpp in its directory are compiled
# and linked with the engine, and libpsyqo.a is built on demand. V=1 shows the
# commands.
#
# Unlike nugget's own common.mk, objects never land next to their sources:
# they go under $(BUILD_DIR), the way the rest of this engine's builds work.

BUILD_DIR  ?= build
NUGGET_DIR ?= $(ENGINE_DIR)/../librerias/nugget
PSYQO_DIR  ?= $(NUGGET_DIR)/psyqo

EXE = $(PROJECT_NAME).ps-exe
ELF = $(BUILD_DIR)/$(PROJECT_NAME).elf
MAP = $(BUILD_DIR)/$(PROJECT_NAME).map

.DEFAULT_GOAL := all

V ?= 0
ifeq ($(V),0)
  Q := @
endif

# --- toolchain ----------------------------------------------------------------

PREFIX  = mipsel-none-elf
CXX     = $(PREFIX)-g++
OBJCOPY = $(PREFIX)-objcopy

LIBPSYQO = $(PSYQO_DIR)/libpsyqo.a

# Release is what nugget's common.mk defaults to; it decides the -O level
# libpsyqo.a is built with, so it is passed down to that build too.
BUILD_TYPE ?= Release

# The linker scripts come from nugget: ps-exe.ld lays out a PS-EXE, and
# nooverlay.ld stands in when the game declares no overlays.
LDSCRIPT     = $(NUGGET_DIR)/ps-exe.ld
OVERLAYSCRIPT = $(NUGGET_DIR)/nooverlay.ld

# --- flags --------------------------------------------------------------------

# ARCHFLAGS is nugget's, with one change: mips1 for the R3000A, little
# endian, no PIC, no load-linked/store-conditional (the CPU has neither), and
# freestanding since there is no hosted C library underneath.
#
# The change is -msoft-float where nugget has -mfp32. The R3000A has no COP1,
# and -mfp32 lets the compiler emit FPU instructions that fault on real
# hardware: a float divide in the frame loop was enough to put 89 of them in
# the binary. PSYQo never notices because it is fixed point throughout; this
# engine is not, so every float has to go through libgcc's helpers.
ARCHFLAGS = -march=mips1 -mabi=32 -EL -fno-pic -mno-shared -mno-abicalls \
            -msoft-float -mno-llsc -fno-stack-protector -nostdlib -ffreestanding

INCLUDES = -I$(NUGGET_DIR) \
           -I$(NUGGET_DIR)/third_party/EASTL/include \
           -I$(NUGGET_DIR)/third_party/EABase/include/Common \
           -I$(ENGINE_DIR)/include -I$(ENGINE_DIR)/src -I.

# -O2, not nugget's -Os: the engine's own code goes a third faster with
# it (35 to 47 fps on the room scene), and -O3 gives no more. Link time
# optimization on top, so calls across files inline too.
COMMONFLAGS = -g -O2 -flto -ffunction-sections -fdata-sections -mno-gpopt \
              -fomit-frame-pointer -fno-builtin -fno-strict-aliasing \
              -Wno-attributes $(ARCHFLAGS) $(INCLUDES) $(cflags_extra)

CXXFLAGS = -std=c++20 -Wall -fno-exceptions -fno-rtti $(COMMONFLAGS)

LDFLAGS = -g -O2 -flto -nostdlib -static -Wl,--gc-sections \
          -Wl,-Map=$(MAP) -Wl,--oformat=elf32-littlemips \
          -T$(OVERLAYSCRIPT) -T$(LDSCRIPT) $(ARCHFLAGS)

# -nostdlib drops libgcc along with everything else, so the soft-float
# helpers (__addsf3, __divsf3, __fixsfsi and the rest) have to be asked for
# by name. gcc picks the soft-float multilib on its own.
LDLIBS = -lgcc

# --- sources ------------------------------------------------------------------

# The modules that are ported, in the order they link. The R3000A's
# instruction cache is 4 KB, so placement matters even more than it does on
# the VR4300: hot path first, cold code last.
#
# This is a whitelist, not a skip list: the port has just started, and most
# of src/ still speaks to libdragon and cannot be fed to this compiler at
# all. A module joins the build by being named here.
#
# An entry is a directory under src/, or a single .cpp for a directory that
# is only partly ported.
ENGINE_ORDER = \
	physics/math/e32_math_common.cpp physics/math/e32_trig.cpp \
	physics/math/e32_vector2.cpp physics/math/e32_vector3.cpp \
	physics/math/e32_matrix3.cpp physics/math/e32_quaternion.cpp \
	physics/math/e32_transform.cpp physics/math/e32_math_functions.cpp \
	physics/geometry physics/math/e32_frustum.cpp \
	render graphics/e32_mesh.cpp shaders/e32_mesh_deform.cpp animation \
	graphics/e32_texture.cpp \
	physics/shapes physics/collision physics/broadphase physics/body \
	physics/world physics/buoyancy physics/cloth physics/memory \
	character3d/e32_character3d_physics.cpp \
	character3d/e32_character3d_movement.cpp \
	character3d/e32_character3d.cpp character3d/e32_character3d_stubs.cpp \
	control/e32_character3d_control.cpp control/e32_player_control.cpp \
	player shaders/e32_water.cpp \
	scene3d camera \
	control/e32_controller.cpp control/e32_camera_control.cpp \
	entity/e32_entity3d.cpp game debug \
	system time viewport resource

engine_src = $(foreach m,$(ENGINE_ORDER),\
	$(if $(filter %.cpp,$(m)),$(m),\
	  $(patsubst $(ENGINE_DIR)/src/%,%,$(wildcard $(ENGINE_DIR)/src/$(m)/e32_*.cpp))))

# The game's sources: its directory, plus whatever it lists in src_extra
# (subdirectories are not walked).
src = $(wildcard *.cpp) $(src_extra)

objects = $(addsuffix .o,$(basename $(addprefix $(BUILD_DIR)/engine/,$(engine_src)))) \
          $(addsuffix .o,$(basename $(addprefix $(BUILD_DIR)/,$(src)))) \
          $(BUILD_DIR)/fs_data.o $(BUILD_DIR)/fs_table.o

# --- assets -------------------------------------------------------------------

# The game's models: every .obj under assets/models becomes a .model under
# filesystem/models through the engine's importer, a host tool built on
# demand. The importer assumes meters and applies the console's scale on
# its own; there is nothing to pass. OBJ keeps the quads the GPU draws
# natively, and its .mtl names the textures.
HOST_CXX ?= g++

MODEL_IMPORTER = $(ENGINE_DIR)/tools/psx_model_importer/obj_to_e32

assets_obj   = $(wildcard assets/models/*.obj)
assets_model = $(addprefix filesystem/models/,$(notdir $(assets_obj:%.obj=%.model)))

# Rebuilt when its sources or the file format change, so the models on
# disk never lag behind the header the engine reads them with.
$(MODEL_IMPORTER): $(ENGINE_DIR)/tools/psx_model_importer/main.cpp \
                   $(ENGINE_DIR)/include/graphics/e32_model_format.h
	@echo "    [HOST] $@"
	$(Q)$(MAKE) -C $(ENGINE_DIR)/tools/psx_model_importer CXX=$(HOST_CXX)

filesystem/models/%.model: assets/models/%.obj assets/models/%.mtl $(MODEL_IMPORTER)
	@mkdir -p $(dir $@)
	@echo "    [MODEL] $@"
	$(Q)$(MODEL_IMPORTER) $< $@

# Collision meshes: the models the game names in collision_models also go
# through the collision importer, from the same OBJ, to a .collision under
# filesystem/collision. What is drawn is what is walked on.
COLLISION_IMPORTER = $(ENGINE_DIR)/tools/psx_collision_importer/obj_to_collision

assets_collision = $(addprefix filesystem/collision/,$(addsuffix .collision,$(collision_models)))

$(COLLISION_IMPORTER): $(ENGINE_DIR)/tools/psx_collision_importer/main.cpp \
                       $(ENGINE_DIR)/include/physics/collision/e32_collision_mesh.h
	@echo "    [HOST] $@"
	$(Q)$(MAKE) -C $(ENGINE_DIR)/tools/psx_collision_importer CXX=$(HOST_CXX)

filesystem/collision/%.collision: assets/models/%.obj $(COLLISION_IMPORTER)
	@mkdir -p $(dir $@)
	@echo "    [COLLISION] $@"
	$(Q)$(COLLISION_IMPORTER) $< $@

# --- filesystem ---------------------------------------------------------------

# Everything under filesystem/ is linked into the executable, each file as
# a block of read only data, and a generated table maps "rom:/<path>" to
# it. Two generated sources: the assembly that pulls the bytes in, and the
# table the resource module walks.
fs_files = $(sort $(assets_model) $(assets_collision) $(shell find filesystem -type f 2>/dev/null))

fs_symbol = fs_$(subst -,_,$(subst .,_,$(subst /,_,$(1))))

define fs_data_entry
	.section .rodata
	.balign 4
	.global $(call fs_symbol,$(1))_start
	.global $(call fs_symbol,$(1))_end
$(call fs_symbol,$(1))_start:
	.incbin "$(1)"
$(call fs_symbol,$(1))_end:

endef

define fs_table_entry
	{ "rom:/$(patsubst filesystem/%,%,$(1))", $(call fs_symbol,$(1))_start, $(call fs_symbol,$(1))_end },

endef

$(BUILD_DIR)/fs_data.s: $(fs_files)
	@mkdir -p $(dir $@)
	@echo "    [FS] $@"
	$(file >$@,$(foreach f,$(fs_files),$(call fs_data_entry,$(f))))

$(BUILD_DIR)/fs_table.cpp: $(fs_files)
	@mkdir -p $(dir $@)
	@echo "    [FS] $@"
	$(file >$@,#include "resource/e32_resource.h")
	$(file >>$@,extern "C" {)
	$(file >>$@,$(foreach f,$(fs_files),extern const uint8_t $(call fs_symbol,$(f))_start[]; extern const uint8_t $(call fs_symbol,$(f))_end[];))
	$(file >>$@,})
	$(file >>$@,const EmbeddedFile embedded_file[] = {)
	$(file >>$@,$(foreach f,$(fs_files),$(call fs_table_entry,$(f))))
	$(file >>$@,	{ nullptr, nullptr, nullptr })
	$(file >>$@,};)
	$(file >>$@,const unsigned embedded_file_count = $(words $(fs_files));)

$(BUILD_DIR)/fs_data.o: $(BUILD_DIR)/fs_data.s $(fs_files)
	@echo "    [AS] $<"
	$(Q)$(CXX) -c $(ARCHFLAGS) -o $@ $<

$(BUILD_DIR)/fs_table.o: $(BUILD_DIR)/fs_table.cpp
	@echo "    [CXX] $<"
	$(Q)$(CXX) -c $(CXXFLAGS) -o $@ $<

# --- rules --------------------------------------------------------------------

all: $(EXE)

$(BUILD_DIR)/engine/%.o: $(ENGINE_DIR)/src/%.cpp
	@mkdir -p $(dir $@)
	@echo "    [CXX] $<"
	$(Q)$(CXX) -c $(CXXFLAGS) -MMD -MF $(@:.o=.d) -o $@ $<

$(BUILD_DIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	@echo "    [CXX] $<"
	$(Q)$(CXX) -c $(CXXFLAGS) -MMD -MF $(@:.o=.d) -o $@ $<

# PSYQo builds itself, in its own tree, with its own Makefile: the same call
# nugget's psyqo.mk makes, with ARCHFLAGS overridden so the whole library
# agrees with the engine on soft float. Without it the linker warns on every
# object that mixes the two ABIs. Always asked for, so it rebuilds when its
# sources change.
$(LIBPSYQO):
	@echo "    [PSYQO] $@"
	$(Q)$(MAKE) -C $(PSYQO_DIR) BUILD=$(BUILD_TYPE) ARCHFLAGS="$(ARCHFLAGS)"

$(ELF): $(objects) $(LIBPSYQO)
	@mkdir -p $(dir $@)
	@echo "    [LD] $@"
	$(Q)$(CXX) -g -o $@ $(objects) $(LDFLAGS) $(LIBPSYQO) $(LDLIBS)

# The PS-EXE is the raw image the console loads: the ELF stripped of
# everything but its loadable contents.
$(EXE): $(ELF)
	@echo "    [EXE] $@"
	$(Q)$(OBJCOPY) -O binary $< $@

# --- disc image ---------------------------------------------------------------

# A CD image around the executable, for emulators that boot from disc:
# the system file that names the executable, and the executable itself,
# through mkpsxiso. 'make cd' builds it; the run scripts for those
# emulators do too.
MKPSXISO ?= mkpsxiso

CUE = $(PROJECT_NAME).cue
BIN = $(PROJECT_NAME).bin

$(BUILD_DIR)/system.cnf:
	@mkdir -p $(dir $@)
	$(file >$@,BOOT=cdrom:\PSX.EXE;1)
	$(file >>$@,TCB=4)
	$(file >>$@,EVENT=10)
	$(file >>$@,STACK=801FFFF0)

# mkpsxiso resolves every path against the XML's own directory, hence the
# absolute ones.
$(BUILD_DIR)/cd.xml: $(BUILD_DIR)/system.cnf
	$(file >$@,<?xml version="1.0" encoding="UTF-8"?>)
	$(file >>$@,<iso_project image_name="$(abspath $(BIN))" cue_sheet="$(abspath $(CUE))">)
	$(file >>$@,	<track type="data">)
	$(file >>$@,		<identifiers system="PLAYSTATION" application="PLAYSTATION" volume="$(PROJECT_NAME)" publisher="ENGINE32"/>)
	$(file >>$@,		<directory_tree>)
	$(file >>$@,			<file name="SYSTEM.CNF" type="data" source="$(abspath $(BUILD_DIR)/system.cnf)"/>)
	$(file >>$@,			<file name="PSX.EXE" type="data" source="$(abspath $(EXE))"/>)
	$(file >>$@,		</directory_tree>)
	$(file >>$@,	</track>)
	$(file >>$@,</iso_project>)

$(CUE) $(BIN): $(EXE) $(BUILD_DIR)/cd.xml
	@echo "    [CD] $(CUE)"
	$(Q)$(MKPSXISO) -y -q $(BUILD_DIR)/cd.xml

cd: $(CUE)

clean:
	rm -rf $(BUILD_DIR) $(EXE) $(CUE) $(BIN) filesystem/models filesystem/collision

# Also drops libpsyqo.a and its objects, which live in the nugget checkout.
deepclean: clean
	$(Q)$(MAKE) -C $(PSYQO_DIR) clean

-include $(objects:%.o=%.d)

.PHONY: all cd clean deepclean $(LIBPSYQO)
