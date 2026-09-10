/*
	Low-level malloc/free wrappers + PTR_ADD helper. Shared by PhysicsStack /
	PhysicsHeap / PhysicsPagedAllocator.
*/
#ifndef ENGINE_32_PHYSICS_MEMORY_H
#define ENGINE_32_PHYSICS_MEMORY_H

#include <stdint.h>

#include "psyqo/alloc.h"


static inline void *physics_alloc(int32_t bytes) { return psyqo_malloc((size_t)bytes); }
static inline void  physics_free(void *memory)   { psyqo_free(memory); }

#define PHYSICS_PTR_ADD(P, BYTES) ((void*)(((uint8_t*)(P)) + (BYTES)))


#endif
