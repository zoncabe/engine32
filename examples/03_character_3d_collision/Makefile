ENGINE_DIR   = ../..
PROJECT_NAME = 03_character_3d_collision

# The physics, the character and the player seat: the modules this example is
# about, switched on for the engine's scene and entity code.
cflags_extra = -DENGINE_32_PHYSICS -DENGINE_32_CHARACTER -DENGINE_32_PLAYER

# The prefabs, one per file; what the scene runs with; what drives it.
src_extra = $(wildcard prefabs/*.cpp) $(wildcard scene/*.cpp) $(wildcard controls/*.cpp)

# Models that also need a collision mesh: the room is what the body walks on,
# and the three still bodies collide as the shapes their prefabs declare.
collision_models = room

include $(ENGINE_DIR)/engine32.mk
