#include <stddef.h>

#include "psyqo/alloc.h"

#include "physics/cloth/e32_cloth.h"
#include "physics/math/e32_math_common.h"


/*
	Edge extraction: one constraint per unique mesh edge. On a triangulated
	grid that yields the structural edges plus the diagonal of every quad, and
	the diagonal is what resists shearing, so it comes free with the
	triangulation.
*/

#define EDGE_EMPTY 0xFFFFFFFFu

static uint32_t cloth_edgeKey(uint16_t a, uint16_t b)
{
	return a < b ? ((uint32_t)a << 16) | b
	             : ((uint32_t)b << 16) | a;
}

/* Returns true when the edge had not been seen yet. */
static bool cloth_edgeIsNew(uint32_t *table, uint32_t mask, uint32_t key)
{
	uint32_t i = (key * 2654435761u) & mask;   /* Knuth multiplicative hash */

	while (table[i] != EDGE_EMPTY) {
		if (table[i] == key) return false;
		i = (i + 1) & mask;
	}

	table[i] = key;
	return true;
}


static void cloth_updateNormals(Cloth *cloth);


/* The edge held in place: everything at or left of the def's X threshold. */
static bool cloth_atPinnedEdge(const Vector3 &position, void *user)
{
	return position.x <= *(const Fixed *)user;
}


bool Cloth::create(const CollisionMesh *mesh, const ClothDef *def)
{
	*this = {};
	iterations = 4;   /* Jakobsen: 3-4 relaxation passes */

	if (mesh == NULL || mesh->vertex_count == 0 || mesh->triangle_count == 0) return false;

	particle_count  = mesh->vertex_count;
	position        = (Vector3 *)psyqo_malloc(sizeof(Vector3) * particle_count);
	previous        = (Vector3 *)psyqo_malloc(sizeof(Vector3) * particle_count);
	force           = (Vector3 *)psyqo_malloc(sizeof(Vector3) * particle_count);
	normal          = (Vector3 *)psyqo_malloc(sizeof(Vector3) * particle_count);
	render_position = (Vector3 *)psyqo_malloc(sizeof(Vector3) * particle_count);
	pinned          = (uint8_t *)psyqo_malloc(particle_count);
	if (pinned) __builtin_memset(pinned, 0, particle_count);

	triangle_count = mesh->triangle_count;
	triangle = (uint16_t *)psyqo_malloc(sizeof(uint16_t) * 3 * triangle_count);
	if (triangle)
		__builtin_memcpy(triangle, mesh->indices, sizeof(uint16_t) * 3 * triangle_count);

	/* At rest both Verlet slots hold the same position, so velocity is zero. */
	if (position && previous && render_position) {
		__builtin_memcpy(position, mesh->vertices, sizeof(Vector3) * particle_count);
		__builtin_memcpy(previous, mesh->vertices, sizeof(Vector3) * particle_count);
		__builtin_memcpy(render_position, mesh->vertices, sizeof(Vector3) * particle_count);
	}

	uint32_t edge_max = (uint32_t)mesh->triangle_count * 3;

	uint32_t capacity = 16;
	while (capacity < edge_max * 2) capacity *= 2;

	uint32_t *table = (uint32_t *)psyqo_malloc(sizeof(uint32_t) * capacity);
	constraint = (ClothConstraint *)psyqo_malloc(sizeof(ClothConstraint) * edge_max);

	if (!position || !previous || !force || !pinned
	    || !render_position || !triangle || !table || !constraint) {
		psyqo_free(table);
		free();
		return false;
	}

	__builtin_memset(table, 0xFF, sizeof(uint32_t) * capacity);

	for (uint16_t t = 0; t < mesh->triangle_count; t++)
	{
		const uint16_t *tri = &mesh->indices[t * 3];

		for (int e = 0; e < 3; e++)
		{
			uint16_t a = tri[e];
			uint16_t b = tri[(e + 1) % 3];
			if (a == b) continue;

			if (!cloth_edgeIsNew(table, capacity - 1, cloth_edgeKey(a, b))) continue;

			Fixed rest = (position[a] - position[b]).squaredMagnitude();

			/* A zero-length edge would divide by zero in the projection. */
			if (rest <= 0) continue;

			constraint[constraint_count++] = { a, b, rest };
		}
	}

	psyqo_free(table);

	if (def) {
		damping = def->damping;
		if (def->iterations) iterations = def->iterations;

		Fixed pin_max_x = def->pin_max_x;
		pinWhere(cloth_atPinnedEdge, &pin_max_x);
	}

	/* The rest normals have to exist before anything binds to the cloth: that
	   is what lets the binding tell whether the mesh's own normals agree with
	   the winding of this triangle list. */
	cloth_updateNormals(this);

	return true;
}


void Cloth::pinWhere(bool (*predicate)(const Vector3 &position, void *user), void *user)
{
	if (pinned == NULL || predicate == NULL) return;

	for (uint16_t i = 0; i < particle_count; i++)
		pinned[i] = predicate(position[i], user) ? 1 : 0;
}


/*
	Integration: Jakobsen's step, with the previous position standing in for
	velocity:

		x += x - oldx + a * dt * dt;
		oldx = temp;

	Damping scales the implied velocity term instead of adding a force, which
	keeps the step to one multiply-add per axis.
*/
/*
	Wind acts on the surface, not on the points. For each triangle the raw
	cross product gives a normal whose length is twice the area, and the unit
	normal says how squarely the face meets the wind:

		force = normal * dot(normalize(normal), wind)

	That product is what makes a cloth turn into the wind on its own: a face
	edge-on has dot near zero and catches nothing, so the faces still exposed
	drag it around until the whole sheet lines up. A uniform push per particle
	cannot do that, because it applies no torque.

	Method from Mosegaard's cloth tutorial (addWindForcesForTriangle).
*/
static void cloth_applyWind(Cloth *cloth)
{
	__builtin_memset(cloth->force, 0, sizeof(Vector3) * cloth->particle_count);

	if (cloth->wind.x == 0 && cloth->wind.y == 0 && cloth->wind.z == 0) return;

	for (uint16_t t = 0; t < cloth->triangle_count; t++)
	{
		const uint16_t *tri = &cloth->triangle[t * 3];

		const Vector3 &a = cloth->position[tri[0]];
		Vector3 edge1 = cloth->position[tri[1]] - a;
		Vector3 edge2 = cloth->position[tri[2]] - a;

		Vector3 normal = edge1.cross(edge2);

		Fixed len_sq = normal.squaredMagnitude();
		if (len_sq <= 0) continue;

		/* dot(normal, wind) / |normal|: the unit normal without building it. */
		Fixed facing = normal.dot(cloth->wind) * inverseSqrt(len_sq);

		Vector3 force = normal * facing;

		for (int v = 0; v < 3; v++)
			cloth->force[tri[v]] += force;
	}
}


/* Area-weighted vertex normals: the raw cross product is already twice the
   face area, so accumulating it unnormalised weights each face by its size,
   which is what keeps a stretched triangle from swinging the result. */
static void cloth_updateNormals(Cloth *cloth)
{
	if (cloth->normal == NULL) return;

	__builtin_memset(cloth->normal, 0, sizeof(Vector3) * cloth->particle_count);

	for (uint16_t t = 0; t < cloth->triangle_count; t++)
	{
		const uint16_t *tri = &cloth->triangle[t * 3];

		const Vector3 &a = cloth->position[tri[0]];
		Vector3 edge1 = cloth->position[tri[1]] - a;
		Vector3 edge2 = cloth->position[tri[2]] - a;

		Vector3 face = edge1.cross(edge2);

		for (int v = 0; v < 3; v++)
			cloth->normal[tri[v]] += face;
	}

	for (uint16_t i = 0; i < cloth->particle_count; i++) {
		Vector3 *n = &cloth->normal[i];
		Fixed len_sq = n->squaredMagnitude();

		if (len_sq <= 0) { *n = { 0.0_fp, 0.0_fp, 1.0_fp }; continue; }

		*n *= inverseSqrt(len_sq);
	}
}


static void cloth_integrate(Cloth *cloth, Fixed dt)
{
	Fixed dt2    = dt * dt;
	Fixed retain = 1.0_fp - cloth->damping;

	for (uint16_t i = 0; i < cloth->particle_count; i++)
	{
		if (cloth->pinned[i]) continue;

		Vector3 current = cloth->position[i];

		Vector3 accel = cloth->gravity + cloth->force[i];

		cloth->position[i] += (current - cloth->previous[i]) * retain + accel * dt2;

		cloth->previous[i] = current;
	}
}


/*
	Constraint relaxation. The exact projection needs the current distance, and
	therefore a root:

		deltalength = sqrt(delta * delta);
		diff        = (deltalength - restlength) / deltalength;
		x1 -= delta * 0.5 * diff;
		x2 += delta * 0.5 * diff;

	Jakobsen's approximation replaces it with the first-order Taylor expansion
	around the rest length, which is exact where it matters most because a
	satisfied constraint sits precisely there:

		delta *= restlength * restlength / (delta * delta + restlength * restlength) - 0.5;
		x1 -= delta;
		x2 += delta;

	One division per constraint, no roots. The factor already carries the half
	share, so a particle facing a pinned neighbour takes twice as much.
*/
static void cloth_satisfy(Cloth *cloth)
{
	for (uint8_t pass = 0; pass < cloth->iterations; pass++)
	{
		for (uint16_t i = 0; i < cloth->constraint_count; i++)
		{
			const ClothConstraint *c = &cloth->constraint[i];

			bool pin_a = cloth->pinned[c->a];
			bool pin_b = cloth->pinned[c->b];
			if (pin_a && pin_b) continue;

			Vector3 *x1 = &cloth->position[c->a];
			Vector3 *x2 = &cloth->position[c->b];

			Vector3 delta = *x2 - *x1;

			Fixed dist_sq = delta.squaredMagnitude();
			Fixed scale   = c->rest_length_sq / (dist_sq + c->rest_length_sq) - 0.5_fp;

			if (pin_a) {
				*x2 += delta * (scale * 2);
			}
			else if (pin_b) {
				*x1 -= delta * (scale * 2);
			}
			else {
				*x1 -= delta * scale;
				*x2 += delta * scale;
			}
		}
	}
}


void Cloth::step(Fixed dt)
{
	if (position == NULL) return;

	cloth_applyWind(this);
	cloth_integrate(this, dt);
	cloth_satisfy(this);

	/* After the solver, so the normals describe the pose the mesh will show.
	   Reusing the cross products from cloth_applyWind would save the pass but
	   would describe the pose of the previous frame. */
	cloth_updateNormals(this);
}


void Cloth::blendRenderState(Fixed t)
{
	if (render_position == NULL) return;

	for (uint16_t i = 0; i < particle_count; i++)
		render_position[i] = previous[i] + (position[i] - previous[i]) * t;
}


void Cloth::free()
{
	psyqo_free(position);
	psyqo_free(previous);
	psyqo_free(force);
	psyqo_free(normal);
	psyqo_free(render_position);
	psyqo_free(pinned);
	psyqo_free(triangle);
	psyqo_free(constraint);

	*this = {};
}
