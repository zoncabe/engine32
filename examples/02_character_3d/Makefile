ENGINE_DIR   = ../..
PROJECT_NAME = 02_character_3d

# The physics, the character and the player seat: the modules this example is
# about, switched on for the engine's scene and entity code.
cflags_extra = -DENGINE_32_PHYSICS -DENGINE_32_CHARACTER -DENGINE_32_PLAYER

# The prefabs, one per file; what the scene runs with; what drives it.
src_extra = $(wildcard prefabs/*.cpp) $(wildcard scene/*.cpp) $(wildcard controls/*.cpp)

# The room is the only model that also needs a collision mesh: it is what the
# character stands on.
collision_models = room

include $(ENGINE_DIR)/engine32.mk
