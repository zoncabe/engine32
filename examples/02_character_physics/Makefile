ENGINE_DIR   = ../..
PROJECT_NAME = 02_character_physics

# The physics, the character, the player seat and the water: the modules
# this example is about, switched on for the engine's scene and entity code.
cflags_extra = -DENGINE_32_PHYSICS -DENGINE_32_CHARACTER -DENGINE_32_PLAYER -DENGINE_32_WATER

# The prefabs, one per file.
src_extra = $(wildcard prefabs/*.cpp)

# Models that also need a collision mesh: the room is what the body walks
# on, the water plane is what the waves run on.
collision_models = room water

include $(ENGINE_DIR)/engine32.mk
