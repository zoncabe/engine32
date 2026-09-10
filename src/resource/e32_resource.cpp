#include "resource/e32_resource.h"


static bool same(const char *a, const char *b)
{
	while (*a && *a == *b) { a++; b++; }
	return *a == *b;
}


const uint8_t *Resource::find(const char *path, size_t *size)
{
	for (unsigned i = 0; i < embedded_file_count; i++) {
		if (!same(embedded_file[i].path, path)) continue;
		if (size) *size = (size_t)(embedded_file[i].end - embedded_file[i].start);
		return embedded_file[i].start;
	}
	if (size) *size = 0;
	return nullptr;
}
