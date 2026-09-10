# Host tool: OBJ to the engine's model file. lodepng is the copy vendored
# in the N64 importer next door.

LIB      = ../model_importer/src/lib
CXXFLAGS += -O2 -std=c++20 -I$(LIB) -I../../include

all: obj_to_e32

obj_to_e32: main.cpp $(LIB)/lodepng.cpp $(LIB)/lodepng.h ../../include/graphics/e32_model_format.h
	$(CXX) $(CXXFLAGS) -o $@ main.cpp $(LIB)/lodepng.cpp

clean:
	rm -f obj_to_e32 gltf_to_e32

.PHONY: all clean
