/*
	OBJ to the engine's model file, for the console.

	Reads a Wavefront .obj the way Blender exports it: objects with
	positions, normals, texture coordinates and, optionally, vertex colors
	on the position lines; faces of three or four vertices, which the file
	keeps as they are since the GPU draws quads natively, and bigger ones,
	cut into a quad and a fan of triangles; materials from
	the .mtl next to it, with a diffuse color and a PNG texture. The scene
	is expected in meters with Y up, which is what Blender exports by
	default; the importer turns it Z up and scales the positions to the
	GTE's integers on its own.

	Usage: obj_to_e32 <in.obj> <out.model> [--verbose]
*/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <array>
#include <string>
#include <vector>
#include <map>

#include "lodepng.h"

#include "graphics/e32_model_format.h"


static bool verbose = false;


/* --- what gets written ---------------------------------------------------- */

struct Material {
	std::string name;
	uint16_t width = 0, height = 0;
	std::vector<uint16_t> pixels;
	uint8_t color[4] = {255, 255, 255, 255};

	/* map_Ke: the layer the GPU adds over the base texture in a second
	   pass. Has to match the base texture's size. */
	uint16_t glow_width = 0, glow_height = 0;
	std::vector<uint16_t> glow_pixels;
};

struct Object {
	std::string name;
	uint16_t material = 0;
	std::vector<ModelFileVertex> vertices;
	std::vector<ModelFilePosition> positions;
	std::vector<ModelFileShade> shades;
	std::vector<ModelFileFace> faces;
	int16_t aabb_min[3], aabb_max[3];
};


/* --- helpers -------------------------------------------------------------- */

static int16_t clamp16(float v)
{
	if (v >  32767.0f) return  32767;
	if (v < -32768.0f) return -32768;
	return (int16_t)lrintf(v);
}

static uint8_t clamp8(float v)
{
	if (v > 255.0f) return 255;
	if (v < 0.0f)   return 0;
	return (uint8_t)lrintf(v);
}

/* 8 bit RGBA to the GPU's 15 bit. Black is transparent to the GPU unless
   the mask bit is set, so opaque black gets it; transparent pixels are 0. */
static uint16_t toPsxColor(uint8_t r, uint8_t g, uint8_t b, uint8_t a)
{
	if (a < 128) return 0;
	uint16_t c = (r >> 3) | ((g >> 3) << 5) | ((b >> 3) << 10);
	return c == 0 ? 0x8000 : c;
}

/* OBJ is Y up; the engine is Z up. */
static void toZUp(float v[3])
{
	float y = v[1], z = v[2];
	v[1] = -z;
	v[2] = y;
}

static std::string directoryOf(const std::string &path)
{
	size_t slash = path.find_last_of('/');
	return slash == std::string::npos ? "" : path.substr(0, slash + 1);
}

static std::string trim(const std::string &s)
{
	size_t a = s.find_first_not_of(" \t\r\n");
	size_t b = s.find_last_not_of(" \t\r\n");
	return a == std::string::npos ? "" : s.substr(a, b - a + 1);
}


/* --- materials ------------------------------------------------------------ */

/* One PNG to the GPU's pixels, into whichever layer of the material asked. */
static void decodeTexture(const std::string &path, const std::string &name,
                          uint16_t *width, uint16_t *height, std::vector<uint16_t> *pixels)
{
	std::vector<uint8_t> rgba;
	unsigned w, h;
	unsigned err = lodepng::decode(rgba, w, h, path);
	if (err) { fprintf(stderr, "texture %s: %s\n", path.c_str(), lodepng_error_text(err)); return; }
	if (w > 256 || h > 256) {
		fprintf(stderr, "texture of %s is %ux%u: the GPU addresses 256x256 per page\n", name.c_str(), w, h);
		return;
	}
	/* The GPU repeats a texture through a mask on the coordinates, so
	   only powers of two, from 8 up, repeat. */
	if (w < 8 || h < 8 || (w & (w - 1)) || (h & (h - 1))) {
		fprintf(stderr, "texture of %s is %ux%u: sides must be powers of two, 8 to 256\n", name.c_str(), w, h);
		return;
	}

	*width  = (uint16_t)w;
	*height = (uint16_t)h;
	pixels->resize(w * h);
	for (unsigned i = 0; i < w * h; i++)
		(*pixels)[i] = toPsxColor(rgba[i*4], rgba[i*4+1], rgba[i*4+2], rgba[i*4+3]);
}

/* A texture path of the .mtl, relative to the .mtl itself. */
static std::string texturePath(const std::string &dir, const std::string &tex)
{
	if (tex.empty() || tex[0] == '/') return tex;
	return dir + tex;
}

/* Every material of a .mtl, in file order. */
static void readMtl(const std::string &path, std::vector<Material> *materials)
{
	FILE *f = fopen(path.c_str(), "r");
	if (!f) { fprintf(stderr, "cannot open %s\n", path.c_str()); return; }

	std::string dir = directoryOf(path);
	Material *mat = nullptr;
	char line[1024];

	while (fgets(line, sizeof(line), f)) {
		std::string s = trim(line);
		if (s.compare(0, 7, "newmtl ") == 0) {
			materials->push_back(Material());
			mat = &materials->back();
			mat->name = trim(s.substr(7));
		} else if (!mat) {
			continue;
		} else if (s.compare(0, 3, "Kd ") == 0) {
			float r, g, b;
			if (sscanf(s.c_str() + 3, "%f %f %f", &r, &g, &b) == 3) {
				mat->color[0] = clamp8(r * 255.0f);
				mat->color[1] = clamp8(g * 255.0f);
				mat->color[2] = clamp8(b * 255.0f);
			}
		} else if (s.compare(0, 2, "d ") == 0) {
			float d;
			if (sscanf(s.c_str() + 2, "%f", &d) == 1) mat->color[3] = clamp8(d * 255.0f);
		} else if (s.compare(0, 7, "map_Kd ") == 0) {
			std::string tex = texturePath(dir, trim(s.substr(7)));
			if (tex.empty()) continue;
			decodeTexture(tex, mat->name, &mat->width, &mat->height, &mat->pixels);
		} else if (s.compare(0, 7, "map_Ke ") == 0) {
			std::string tex = texturePath(dir, trim(s.substr(7)));
			if (tex.empty()) continue;
			decodeTexture(tex, mat->name, &mat->glow_width, &mat->glow_height, &mat->glow_pixels);
		}
	}

	fclose(f);
}


/* --- objects -------------------------------------------------------------- */

struct ObjCorner { int v, t, n; };

/* One corner of a face as the file writes it: v, v/t, v//n or v/t/n,
   one based, negative from the end. */
static bool parseCorner(const char *s, int nv, int nt, int nn, ObjCorner *out)
{
	int v = 0, t = 0, n = 0;
	if (sscanf(s, "%d/%d/%d", &v, &t, &n) == 3) {
	} else if (sscanf(s, "%d//%d", &v, &n) == 2) {
		t = 0;
	} else if (sscanf(s, "%d/%d", &v, &t) == 2) {
		n = 0;
	} else if (sscanf(s, "%d", &v) == 1) {
		t = n = 0;
	} else {
		return false;
	}
	out->v = v < 0 ? nv + v : v - 1;
	out->t = t < 0 ? nt + t : t - 1;
	out->n = n < 0 ? nn + n : n - 1;
	return out->v >= 0 && out->v < nv;
}

struct ObjFile {
	std::vector<std::array<float, 3>> positions;
	std::vector<std::array<float, 3>> colors;   /* per position, when the file has them */
	std::vector<std::array<float, 2>> uvs;
	std::vector<std::array<float, 3>> normals;
	std::vector<Material> materials;
	std::vector<Object> objects;
};

/* The three tables a corner indexes. Each key is the quantized value the
   console reads, not the OBJ index: two OBJ entries that land on the same
   integers are the same entry to the GTE, and the point of the tables is
   that the coprocessor answers once per entry.

   A position is deduplicated by its 10.6 triple, a shade by its 4.12
   normal plus the color that modulates it. A vertex is one distinct
   combination of position, shade and texel, which is the same set of
   corners the single table used to hold. Texels are assigned per face, so
   the key holds the texel, not the OBJ texture index. */
struct PositionKey { int16_t x, y, z; bool operator<(const PositionKey &o) const { return memcmp(this, &o, sizeof(*this)) < 0; } };
struct ShadeKey { int16_t nx, ny, nz; uint8_t r, g, b, a; bool operator<(const ShadeKey &o) const { return memcmp(this, &o, sizeof(*this)) < 0; } };
struct Slot { uint16_t p, s; uint8_t u, t; bool operator<(const Slot &o) const { return memcmp(this, &o, sizeof(*this)) < 0; } };

static void finishObject(Object *obj)
{
	for (int i = 0; i < 3; i++) { obj->aabb_min[i] = 32767; obj->aabb_max[i] = -32768; }
	for (const ModelFilePosition &p : obj->positions) {
		const int16_t *pv = &p.x;
		for (int k = 0; k < 3; k++) {
			if (pv[k] < obj->aabb_min[k]) obj->aabb_min[k] = pv[k];
			if (pv[k] > obj->aabb_max[k]) obj->aabb_max[k] = pv[k];
		}
	}
}

static bool readObj(const std::string &path, ObjFile *file)
{
	FILE *f = fopen(path.c_str(), "r");
	if (!f) { fprintf(stderr, "cannot open %s\n", path.c_str()); return false; }

	std::string dir = directoryOf(path);
	std::map<std::string, uint16_t> material_index;
	uint16_t default_material = 0xFFFF;

	Object *obj = nullptr;
	uint16_t material = 0xFFFF;
	std::map<Slot, uint16_t> slots;
	std::map<PositionKey, uint16_t> position_slots;
	std::map<ShadeKey, uint16_t> shade_slots;
	char line[4096];

	auto beginObject = [&](const std::string &name) {
		file->objects.push_back(Object());
		obj = &file->objects.back();
		obj->name = name;
		obj->material = material;
		slots.clear();
		position_slots.clear();
		shade_slots.clear();
	};

	while (fgets(line, sizeof(line), f)) {
		std::string s = trim(line);
		if (s.empty() || s[0] == '#') continue;

		if (s.compare(0, 7, "mtllib ") == 0) {
			std::string mtl = trim(s.substr(7));
			if (mtl[0] != '/') mtl = dir + mtl;
			readMtl(mtl, &file->materials);
			for (size_t i = 0; i < file->materials.size(); i++)
				material_index[file->materials[i].name] = (uint16_t)i;
		} else if (s.compare(0, 2, "v ") == 0) {
			float p[3], c[3];
			int n = sscanf(s.c_str() + 2, "%f %f %f %f %f %f", &p[0], &p[1], &p[2], &c[0], &c[1], &c[2]);
			if (n < 3) continue;
			toZUp(p);
			file->positions.push_back({p[0], p[1], p[2]});
			file->colors.push_back(n == 6 ? std::array<float, 3>{c[0], c[1], c[2]} : std::array<float, 3>{1, 1, 1});
		} else if (s.compare(0, 3, "vt ") == 0) {
			float t[2] = {0, 0};
			sscanf(s.c_str() + 3, "%f %f", &t[0], &t[1]);
			/* OBJ has v growing upward; texels grow downward. */
			file->uvs.push_back({t[0], 1.0f - t[1]});
		} else if (s.compare(0, 3, "vn ") == 0) {
			float n[3] = {0, 0, 1};
			sscanf(s.c_str() + 3, "%f %f %f", &n[0], &n[1], &n[2]);
			toZUp(n);
			file->normals.push_back({n[0], n[1], n[2]});
		} else if (s.compare(0, 2, "o ") == 0 || s.compare(0, 2, "g ") == 0) {
			beginObject(trim(s.substr(2)));
		} else if (s.compare(0, 7, "usemtl ") == 0) {
			std::string name = trim(s.substr(7));
			auto it = material_index.find(name);
			if (it == material_index.end()) {
				fprintf(stderr, "%s: material %s not in the mtl\n", path.c_str(), name.c_str());
				continue;
			}
			material = it->second;
			/* A material change inside an object splits it: an object
			   draws with one material. */
			if (obj && obj->material != material && !obj->faces.empty())
				beginObject(obj->name + "." + name);
			else if (obj)
				obj->material = material;
		} else if (s.compare(0, 2, "f ") == 0) {
			if (!obj) beginObject("object");
			if (obj->material == 0xFFFF) {
				if (default_material == 0xFFFF) {
					default_material = (uint16_t)file->materials.size();
					Material white;
					white.name = "default";
					file->materials.push_back(white);
				}
				obj->material = default_material;
			}
			const Material *mat = &file->materials[obj->material];

			/* the corners */
			ObjCorner corner[16];
			int count = 0;
			char *save = nullptr;
			char *tok = strtok_r(line + 2, " \t\r\n", &save);
			while (tok && count < 16) {
				if (!parseCorner(tok, (int)file->positions.size(), (int)file->uvs.size(), (int)file->normals.size(), &corner[count]))
					break;
				count++;
				tok = strtok_r(nullptr, " \t\r\n", &save);
			}
			if (count < 3) continue;

			/* Texels, per face. OBJ repeats the texture past 1.0; the GPU
			   repeats it through its texture window, but a vertex holds a
			   byte, 0..255 texels. So each face is moved by whole repeats
			   until its lowest corner sits inside the first one, and the
			   other corners land where they fall, which the window wraps. */
			float base[2] = {0, 0};
			if (mat->width) {
				for (int a = 0; a < 2; a++) {
					float lo = 0;
					bool any = false;
					for (int k = 0; k < count; k++) {
						if (corner[k].t < 0 || corner[k].t >= (int)file->uvs.size()) continue;
						float u = file->uvs[corner[k].t][a];
						if (!any || u < lo) lo = u;
						any = true;
					}
					base[a] = floorf(lo);
				}
			}

			/* One output face from a pick of the corners. Called once for a
			   triangle or a quad; a bigger face is cut into a quad and a fan
			   of triangles from its first corner, the same fan the collision
			   importer makes, so what is drawn is still what is walked on. */
			auto emitFace = [&](const int *pick, int n) {
				ModelFileFace face;
				for (int k = 0; k < 4; k++) face.v[k] = MODEL_NO_VERTEX;

				for (int k = 0; k < n; k++) {
					const ObjCorner &c = corner[pick[k]];

					uint8_t u = 0, t = 0;
					if (mat->width && c.t >= 0 && c.t < (int)file->uvs.size()) {
						float s0 = (file->uvs[c.t][0] - base[0]) * mat->width;
						float q0 = (file->uvs[c.t][1] - base[1]) * mat->height;
						if (s0 > 255.0f || q0 > 255.0f)
							fprintf(stderr, "%s: a face spans more than 255 texels; the texture window cannot repeat it\n", obj->name.c_str());
						u = clamp8(s0);
						t = clamp8(q0);
					}

					const std::array<float, 3> &p = file->positions[c.v];
					PositionKey pkey;
					memset(&pkey, 0, sizeof(pkey));
					pkey.x = clamp16(p[0] * MODEL_UNITS_PER_METER);
					pkey.y = clamp16(p[1] * MODEL_UNITS_PER_METER);
					pkey.z = clamp16(p[2] * MODEL_UNITS_PER_METER);

					float n[3] = {0, 0, 1};
					if (c.n >= 0 && c.n < (int)file->normals.size())
						for (int r = 0; r < 3; r++) n[r] = file->normals[c.n][r];
					float len = sqrtf(n[0]*n[0] + n[1]*n[1] + n[2]*n[2]);
					if (len > 0.0f) for (int r = 0; r < 3; r++) n[r] /= len;

					const std::array<float, 3> &col = file->colors[c.v];
					ShadeKey skey;
					memset(&skey, 0, sizeof(skey));
					/* 4.12, and 1.0 is 4096: it fits, but the GTE's limiter
					   treats exactly 4096 as saturation, so stop one short. */
					skey.nx = clamp16(n[0] * 4095.0f);
					skey.ny = clamp16(n[1] * 4095.0f);
					skey.nz = clamp16(n[2] * 4095.0f);
					skey.r  = clamp8(col[0] * 255.0f);
					skey.g  = clamp8(col[1] * 255.0f);
					skey.b  = clamp8(col[2] * 255.0f);
					skey.a  = 255;

					uint16_t pi, si;
					auto pit = position_slots.find(pkey);
					if (pit != position_slots.end()) {
						pi = pit->second;
					} else {
						ModelFilePosition mp;
						memset(&mp, 0, sizeof(mp));
						mp.x = pkey.x;
						mp.y = pkey.y;
						mp.z = pkey.z;
						pi = (uint16_t)obj->positions.size();
						obj->positions.push_back(mp);
						position_slots[pkey] = pi;
					}

					auto sit = shade_slots.find(skey);
					if (sit != shade_slots.end()) {
						si = sit->second;
					} else {
						ModelFileShade ms;
						memset(&ms, 0, sizeof(ms));
						ms.nx = skey.nx;
						ms.ny = skey.ny;
						ms.nz = skey.nz;
						ms.r  = skey.r;
						ms.g  = skey.g;
						ms.b  = skey.b;
						ms.a  = skey.a;
						si = (uint16_t)obj->shades.size();
						obj->shades.push_back(ms);
						shade_slots[skey] = si;
					}

					Slot key;
					memset(&key, 0, sizeof(key));
					key.p = pi;
					key.s = si;
					key.u = u;
					key.t = t;
					auto it = slots.find(key);
					uint16_t index;
					if (it != slots.end()) {
						index = it->second;
					} else {
						ModelFileVertex v;
						memset(&v, 0, sizeof(v));
						v.position = pi;
						v.shade    = si;
						v.u = u;
						v.v = t;

						index = (uint16_t)obj->vertices.size();
						obj->vertices.push_back(v);
						slots[key] = index;
					}
					face.v[k] = index;
				}

				obj->faces.push_back(face);
			};

			if (count <= 4) {
				const int all[4] = { 0, 1, 2, 3 };
				emitFace(all, count);
			} else {
				const int quad[4] = { 0, 1, 2, 3 };
				emitFace(quad, 4);
				for (int k = 3; k + 1 < count; k++) {
					const int tri[3] = { 0, k, k + 1 };
					emitFace(tri, 3);
				}
			}
		}
	}

	fclose(f);

	for (Object &o : file->objects) finishObject(&o);

	/* objects that never got a face */
	std::vector<Object> kept;
	for (Object &o : file->objects)
		if (!o.faces.empty()) kept.push_back(o);
	file->objects.swap(kept);

	return true;
}


/* --- output --------------------------------------------------------------- */

struct Writer {
	std::vector<uint8_t> data;

	uint32_t tell() const { return (uint32_t)data.size(); }
	void align() { while (data.size() % 4) data.push_back(0); }
	uint32_t put(const void *p, size_t n) { uint32_t at = tell(); data.insert(data.end(), (const uint8_t *)p, (const uint8_t *)p + n); return at; }
	void patch(uint32_t at, const void *p, size_t n) { memcpy(&data[at], p, n); }
};

static void writeModel(const char *path, const std::vector<Material> &materials, const std::vector<Object> &objects)
{
	Writer w;

	ModelFileHeader header;
	memset(&header, 0, sizeof(header));
	memcpy(header.magic, MODEL_MAGIC, 4);
	header.version        = MODEL_VERSION;
	header.object_count   = (uint16_t)objects.size();
	header.material_count = (uint16_t)materials.size();
	for (int i = 0; i < 3; i++) { header.aabb_min[i] = 32767; header.aabb_max[i] = -32768; }
	for (const Object &o : objects)
		for (int i = 0; i < 3; i++) {
			if (o.aabb_min[i] < header.aabb_min[i]) header.aabb_min[i] = o.aabb_min[i];
			if (o.aabb_max[i] > header.aabb_max[i]) header.aabb_max[i] = o.aabb_max[i];
		}
	uint32_t header_at = w.put(&header, sizeof(header));

	/* strings */
	w.align();
	header.strings = w.tell();
	std::vector<uint32_t> material_names, object_names;
	for (const Material &m : materials) { material_names.push_back(w.tell()); w.put(m.name.c_str(), m.name.size() + 1); }
	for (const Object   &o : objects)   { object_names.push_back(w.tell());   w.put(o.name.c_str(), o.name.size() + 1); }

	/* materials, pixels after the table */
	w.align();
	header.materials = w.tell();
	std::vector<uint32_t> material_at;
	std::vector<bool>     glow_ok;
	for (size_t i = 0; i < materials.size(); i++) {
		const Material &src = materials[i];

		/* The vertices carry one set of coordinates, in texels of the base
		   texture: an emission map of another size would read them wrong. */
		bool glow = src.glow_width && src.glow_height;
		if (glow && (src.glow_width != src.width || src.glow_height != src.height)) {
			fprintf(stderr, "material %s: map_Ke is %ux%u, map_Kd is %ux%u; the emission map must match and is dropped\n",
			        src.name.c_str(), src.glow_width, src.glow_height, src.width, src.height);
			glow = false;
		}
		glow_ok.push_back(glow);

		ModelFileMaterial m;
		memset(&m, 0, sizeof(m));
		m.name   = material_names[i];
		m.width  = src.width;
		m.height = src.height;
		memcpy(m.color, src.color, 4);
		if (glow) {
			m.glow_width  = src.glow_width;
			m.glow_height = src.glow_height;
		}
		material_at.push_back(w.put(&m, sizeof(m)));
	}
	for (size_t i = 0; i < materials.size(); i++) {
		if (!materials[i].pixels.empty()) {
			w.align();
			uint32_t at = w.put(materials[i].pixels.data(), materials[i].pixels.size() * 2);
			w.patch(material_at[i] + offsetof(ModelFileMaterial, pixels), &at, 4);
		}
		if (glow_ok[i]) {
			w.align();
			uint32_t at = w.put(materials[i].glow_pixels.data(), materials[i].glow_pixels.size() * 2);
			w.patch(material_at[i] + offsetof(ModelFileMaterial, glow_pixels), &at, 4);
		}
	}

	/* objects, geometry after the table */
	w.align();
	header.objects = w.tell();
	std::vector<uint32_t> object_at;
	for (size_t i = 0; i < objects.size(); i++) {
		ModelFileObject o;
		memset(&o, 0, sizeof(o));
		o.name         = object_names[i];
		o.material     = objects[i].material;
		o.vertex_count   = (uint16_t)objects[i].vertices.size();
		o.face_count     = (uint16_t)objects[i].faces.size();
		o.position_count = (uint16_t)objects[i].positions.size();
		o.shade_count    = (uint16_t)objects[i].shades.size();
		memcpy(o.aabb_min, objects[i].aabb_min, sizeof(o.aabb_min));
		memcpy(o.aabb_max, objects[i].aabb_max, sizeof(o.aabb_max));
		object_at.push_back(w.put(&o, sizeof(o)));
	}
	for (size_t i = 0; i < objects.size(); i++) {
		w.align();
		uint32_t v = w.put(objects[i].vertices.data(), objects[i].vertices.size() * sizeof(ModelFileVertex));
		w.align();
		uint32_t p = w.put(objects[i].positions.data(), objects[i].positions.size() * sizeof(ModelFilePosition));
		w.align();
		uint32_t s = w.put(objects[i].shades.data(), objects[i].shades.size() * sizeof(ModelFileShade));
		w.align();
		uint32_t x = w.put(objects[i].faces.data(), objects[i].faces.size() * sizeof(ModelFileFace));
		w.patch(object_at[i] + offsetof(ModelFileObject, vertices),  &v, 4);
		w.patch(object_at[i] + offsetof(ModelFileObject, positions), &p, 4);
		w.patch(object_at[i] + offsetof(ModelFileObject, shades),    &s, 4);
		w.patch(object_at[i] + offsetof(ModelFileObject, faces),     &x, 4);
	}
	w.align();

	w.patch(header_at, &header, sizeof(header));

	FILE *f = fopen(path, "wb");
	if (!f) { fprintf(stderr, "cannot write %s\n", path); exit(1); }
	fwrite(w.data.data(), 1, w.data.size(), f);
	fclose(f);

	if (verbose) {
		printf("%s: %u bytes, %zu objects, %zu materials\n", path, w.tell(), objects.size(), materials.size());
		for (const Object &o : objects) {
			size_t quads = 0;
			for (const ModelFileFace &face : o.faces) if (face.v[3] != MODEL_NO_VERTEX) quads++;
			printf("  %s: %zu vertices (%zu positions, %zu shades), %zu faces (%zu quads), material %u\n",
			       o.name.c_str(), o.vertices.size(), o.positions.size(), o.shades.size(),
			       o.faces.size(), quads, o.material);
		}
		for (const Material &m : materials)
			printf("  %s: %ux%u\n", m.name.c_str(), m.width, m.height);
	}
}


/* --- main ----------------------------------------------------------------- */

int main(int argc, char **argv)
{
	if (argc < 3) {
		printf("Usage: %s <in.obj> <out.model> [--verbose]\n", argv[0]);
		return 1;
	}
	for (int i = 3; i < argc; i++)
		if (strcmp(argv[i], "--verbose") == 0) verbose = true;

	ObjFile file;
	if (!readObj(argv[1], &file)) return 1;

	writeModel(argv[2], file.materials, file.objects);
	return 0;
}
