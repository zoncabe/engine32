# Host tool: OBJ to the engine's collision file, for the PlayStation.

CXXFLAGS += -O2 -std=c++20 -I../../include

all: obj_to_collision

obj_to_collision: main.cpp ../../include/physics/collision/e32_collision_mesh.h
	$(CXX) $(CXXFLAGS) -o $@ main.cpp

clean:
	rm -f obj_to_collision

.PHONY: all clean
