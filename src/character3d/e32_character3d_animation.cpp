#include <assert.h>
#include <math.h>
#include <fmath.h>
#include <malloc.h>
#include <string.h>
#include <stdio.h>
#include <libdragon.h>
#include "time/e32_time.h"
#include "entity/e32_entity3d.h"
#include "viewport/e32_viewport.h"
#include "physics/math/e32_math_common.h"


/* Open clips hold a FILE, and libc caps those at 64 (lock pool) — fmemopen
   streams included, they take a lock like any file. Clips open on first use
   and close after a while untouched, so only the graph's working set holds
   files. The delay keeps blend-boundary flicker from churning open/close,
   and the hard cap bounds the open set no matter the input. */
#define ANIMATION_CLIP_CLOSE_DELAY 60   /* frames untouched before closing */
#define ANIMATION_CLIP_MAX_OPEN    24   /* per character */


/* --- layers ----------------------------------------------------------------- */

void character3dAnimation_addLayer(Character3DAnimationBuffer *buffer, const Armature *skel, float weight)
{
	/* Never write past the stack: a dropped layer is a pose glitch, an
	   overflow is garbage quaternions frames later. */
	assert(buffer->count < ANIMATION_MAX_LAYERS);
	if (buffer->count >= ANIMATION_MAX_LAYERS) return;

	buffer->layer[buffer->count] = skel;
	buffer->weight[buffer->count] = weight;
	buffer->count++;
}

/* A layer at this weight or above replaces the pose outright: the first
   corner of a grid at full weight always lands here through the dilution in
   evaluateGraph, and the nlerp it would get costs a sqrt and a divide per bone
   to hand back the layer's own quaternion. What the copy leaves out of the
   blend is a hundredth of the pose underneath, below what the eye sees. */
#define ANIMATION_LAYER_COPY_WEIGHT 0.99f

void character3dAnimation_blendLayers(const Armature *main, const Character3DAnimationBuffer *buffer)
{
	/* A copying layer discards everything blended before it, so the stack is
	   applied from the last one that copies: the layers below it would cost a
	   nlerp per bone each and change nothing. Weights are per layer, so this
	   is settled once, not per bone. */
	int first = 0;
	for (int j = buffer->count - 1; j > 0; j--) {
		if (buffer->weight[j] >= ANIMATION_LAYER_COPY_WEIGHT) { first = j; break; }
	}

	for (int i = 0; i < main->skeletonRef->boneCount; i++)
	{
		Bone *bone = &main->bones[i];
		bone->hasChanged = true;

		for (int j = first; j < buffer->count; j++)
		{
			Bone *layer = &buffer->layer[j]->bones[i];

			if (buffer->weight[j] >= ANIMATION_LAYER_COPY_WEIGHT) {
				bone->rotation = layer->rotation;
				bone->position = layer->position;
				bone->scale    = layer->scale;
				continue;
			}

			bone->rotation = quaternion_nlerp(&bone->rotation, &layer->rotation, buffer->weight[j]);
			bone->position = vector3_lerp(&bone->position, &layer->position, buffer->weight[j]);
			bone->scale = vector3_lerp(&bone->scale, &layer->scale, buffer->weight[j]);
		}
	}
}


/* --- clip cache ------------------------------------------------------------- */

static Armature *character3dAnimation_clipBuffer(const Character3DAnimationDef *def, Character3DAnimation *animation, uint8_t clip)
{
	uint8_t buffer = def->clip[clip].buffer;
	return (buffer == ANIMATION_SLOT_MAIN) ? &animation->main : &animation->buffer[buffer];
}

static void character3dAnimation_closeClip(Character3DAnimation *animation, uint8_t index)
{
	animation_destroy(&animation->clip[index]);
	memset(&animation->clip[index], 0, sizeof(animation->clip[index]));

	/* RAM-resident keyframes: destroy already closed the memory stream. */
	if (animation->clip_data[index]) {
		free(animation->clip_data[index]);
		animation->clip_data[index] = NULL;
	}
}

static void character3dAnimation_openClip(Character3DAnimation *animation, uint8_t index)
{
	Animation *clip = &animation->clip[index];
	const Character3DAnimationClipDef *clip_def = &animation->def->clip[index];

	*clip = animation_create(animation->model, clip_def->name);

	/* RAM-resident keyframes: load the whole .sdata once and swap the clip's
	   stream for a memory one. t3d keeps fread()ing as always, just without
	   the cartridge DMA underneath; rewinds on loop become free. Remove this
	   block (and the frees in closeClip / character3d_delete) to fall back to
	   cartridge streaming. */
	{
		int size = 0;
		void *data = asset_load(clip->animRef->filePath, &size);
		FILE *mem  = data ? fmemopen(data, (size_t)size, "rb") : NULL;
		if (mem) {
			long pos = ftell(clip->file);
			fclose(clip->file);
			fseek(mem, pos, SEEK_SET);
			clip->file = mem;
			animation->clip_data[index] = data;
		} else if (data) {
			free(data);
		}
	}

	animation_attach(clip, character3dAnimation_clipBuffer(animation->def, animation, index));
	animation_setLooping(clip, clip_def->is_looping);
	animation_setPlaying(clip, clip_def->is_looping);
}

static Animation *character3dAnimation_clip(Character3DAnimation *animation, uint8_t index)
{
	Animation *clip = &animation->clip[index];
	animation->clip_cooldown[index] = 0;
	if (clip->animRef != NULL) return clip;

	/* At the cap, evict the least recently touched clip. Clips touched this
	   frame have cooldown 0 and are never evicted: live pointers stay valid. */
	int open  = 0;
	int evict = -1;
	for (int i = 0; i < animation->def->clip_count; i++) {
		if (animation->clip[i].animRef == NULL) continue;
		open++;
		if (evict < 0 || animation->clip_cooldown[i] > animation->clip_cooldown[evict]) evict = i;
	}
	if (open >= ANIMATION_CLIP_MAX_OPEN && evict >= 0 && animation->clip_cooldown[evict] > 0)
		character3dAnimation_closeClip(animation, (uint8_t)evict);

	character3dAnimation_openClip(animation, index);

	return clip;
}

/* Once per frame: close what the graph stopped touching. */
static void character3dAnimation_closeIdleClips(Character3DAnimation *animation)
{
	for (int i = 0; i < animation->def->clip_count; i++) {
		if (animation->clip[i].animRef == NULL) continue;

		if (animation->clip_cooldown[i] < ANIMATION_CLIP_CLOSE_DELAY) {
			animation->clip_cooldown[i]++;
			continue;
		}

		character3dAnimation_closeClip(animation, (uint8_t)i);
	}
}


/* --- shared helpers --------------------------------------------------------- */

static uint8_t character3dAnimation_blendSegment(float weight, uint8_t count, float *t)
{
	if (count < 2) { *t = 0.0f; return 0; }
	if (weight < 0.0f) weight = 0.0f;
	if (weight > 1.0f) weight = 1.0f;

	float s = weight * (count - 1);
	uint8_t i = (uint8_t)s;
	if (i > count - 2) i = count - 2;
	*t = s - i;
	return i;
}

/* gait axis: gait i sits at i / (count - 1) */
static float character3dAnimation_getGaitParam(float speed, const Character3DMovementSettings *movement)
{
	uint8_t last = movement->gait_count - 1;
	if (last == 0 || speed <= movement->gait[0].target_speed) return 0.0f;

	for (uint8_t i = 0; i < last; i++) {
		float lo = movement->gait[i].target_speed;
		float hi = movement->gait[i + 1].target_speed;
		if (speed > hi) continue;
		return (i + (speed - lo) / (hi - lo)) / last;
	}
	return 1.0f;
}

/* grid weight over the idle: covers 0 to the first gait */
static float character3dAnimation_getWalkWeight(float speed, const Character3DMovementSettings *movement)
{
	float first = movement->gait[0].target_speed;
	if (first <= 0.0f) return (speed > 0.0f) ? 1.0f : 0.0f;
	if (speed >= first) return 1.0f;
	return speed / first;
}

/* In the air, or about to be: the crouch that starts a jump runs on the ground
   but already belongs to the air layer. */
static bool character3dAnimation_isAerial(const Character3D *character)
{
	return character->movement.current == MOVEMENT_STATE_FALLING
	    || character->movement.data.jump_timer > 0.0f;
}

/* The 1 to 4 clips a grid blends at the given axis values: the base corner,
   then the next column, the next row, and the far corner, each only when
   its share is not zero. */
static uint8_t character3dAnimation_getGridClips(const Character3DAnimationNode *node, float cols_value, float rows_value, uint8_t clip[4])
{
	float tx, ty;
	uint8_t col = character3dAnimation_blendSegment(cols_value, node->cols, &tx);
	uint8_t row = character3dAnimation_blendSegment(rows_value, node->rows, &ty);
	uint8_t count = 0;

	clip[count++] = node->animation[row * node->cols + col];
	if (tx > 0.0f)              clip[count++] = node->animation[row * node->cols + col + 1];
	if (ty > 0.0f)              clip[count++] = node->animation[(row + 1) * node->cols + col];
	if (tx > 0.0f && ty > 0.0f) clip[count++] = node->animation[(row + 1) * node->cols + col + 1];

	return count;
}

/* phase carry: clips entering the grid start where the ones leaving it were */
static void character3dAnimation_syncGridClips(Character3DAnimation *animation, const Character3DAnimationNode *node, float cols_value, float rows_value)
{
	uint8_t prev[4], curr[4];
	uint8_t prev_count = character3dAnimation_getGridClips(node, animation->param[node->param_cols], animation->param[node->param_rows], prev);
	uint8_t curr_count = character3dAnimation_getGridClips(node, cols_value, rows_value, curr);

	/* phase reference: a corner still taking part, or the previous base if
	   none is. Among those, the centre column wins: it holds the clip the
	   grids share (walk, run, sprint), the one most likely to have kept
	   advancing under another grid. A side corner may have been frozen the
	   whole time, and copying its phase puts the newcomer out of step. */
	uint8_t centre = node->cols / 2;
	int16_t ref_clip = -1;
	for (uint8_t m = 0; m < curr_count; m++) {
		bool carried = false;
		for (uint8_t p = 0; p < prev_count; p++)
			if (curr[m] == prev[p]) carried = true;
		if (!carried) continue;

		bool centred = false;
		for (uint8_t r = 0; r < node->rows; r++)
			if (curr[m] == node->animation[r * node->cols + centre]) centred = true;

		if (ref_clip < 0 || centred) ref_clip = curr[m];
		if (centred) break;
	}
	Animation *ref = character3dAnimation_clip(animation, ref_clip >= 0 ? (uint8_t)ref_clip : prev[0]);

	float ref_length = animation_getLength(ref);
	if (ref_length <= 0.0f) return;
	float phase = ref->time / ref_length;

	for (uint8_t m = 0; m < curr_count; m++) {
		bool carried = false;
		for (uint8_t p = 0; p < prev_count; p++)
			if (curr[m] == prev[p]) carried = true;
		if (carried) continue;

		Animation *dst = character3dAnimation_clip(animation, curr[m]);
		animation_setTime(dst, phase * animation_getLength(dst));
	}
}

/* axis: back 0 | left 1/4 | fwd 2/4 | right 3/4 | back 1
   Shared by every camera-locked grid; at a standstill the direction holds
   whatever its param last carried. */
static float character3dAnimation_getLockedDirectionWeight(const Character3D *character, uint8_t dir_param)
{
	const KinematicBody *body = &character->body;

	if (body->velocity.x == 0.0f && body->velocity.y == 0.0f)
		return character->animation.param[dir_param];

	float velocity_yaw = rad_to_deg(fm_atan2f(-body->velocity.x, -body->velocity.y));
	float rel = angle_wrap_relative(velocity_yaw, body->rotation.z) - body->rotation.z;

	return (rel + 180.0f) / 360.0f;
}

/* carries a clip's phase into every clip of a grid */
static void character3dAnimation_snapGridFromClip(Character3DAnimation *animation, uint8_t src_clip, uint8_t dst_node_idx)
{
	const Character3DAnimationNode *node = &animation->def->node[dst_node_idx];

	Animation *src = character3dAnimation_clip(animation, src_clip);
	float src_length = animation_getLength(src);
	if (src_length <= 0.0f) return;
	float phase = src->time / src_length;

	for (int c = 0; c < node->cols * node->rows; c++) {
		Animation *dst = character3dAnimation_clip(animation, node->animation[c]);
		animation_setTime(dst, phase * animation_getLength(dst));
	}
}

/* hands a grid's phase back to the locomotion clips on the way out */
static void character3dAnimation_snapLocomotionFromGrid(Character3DAnimation *animation, uint8_t src_node_idx, float src_dir)
{
	const Character3DAnimationDef *def = animation->def;
	const Character3DAnimationNode *node = &def->node[src_node_idx];

	/* source: the walk-row clip closest to the current weight */
	uint8_t src_col = (uint8_t)(src_dir * (node->cols - 1) + 0.5f);
	if (src_col > node->cols - 1) src_col = node->cols - 1;

	Animation *src = character3dAnimation_clip(animation, node->animation[src_col]);
	float src_length = animation_getLength(src);
	if (src_length <= 0.0f) return;
	float phase = src->time / src_length;

	const uint8_t target[] = {
		def->walk_animation, def->run_animation, def->sprint_animation,
		def->turn_walk_animation, (uint8_t)(def->turn_walk_animation + 1),
		def->turn_run_animation,  (uint8_t)(def->turn_run_animation + 1),
	};

	for (unsigned t = 0; t < sizeof(target); t++) {
		Animation *dst = character3dAnimation_clip(animation, target[t]);
		animation_setTime(dst, phase * animation_getLength(dst));
	}
}

/* The walk row keeps step with the walk clip, the run row with the run clip:
   every clip of a row gets the speed that makes its cycle last as long as
   that clip's, its own length times the clip's cycle rate. Copying the
   clip's speed as it was would only match cycles between clips of equal
   length; a side clip of another length would drift away from the centre
   column while it played, and any phase carried between the two would land
   out of step. */
static void character3dAnimation_setGridRowSpeeds(Character3DAnimation *animation, const Character3DAnimationNode *node)
{
	const Animation *walk = character3dAnimation_clip(animation, animation->def->walk_animation);
	const Animation *run  = character3dAnimation_clip(animation, animation->def->run_animation);

	float walk_length = animation_getLength(walk);
	float run_length  = animation_getLength(run);
	if (walk_length <= 0.0f || run_length <= 0.0f) return;

	float walk_rate = walk->speed / walk_length;
	float run_rate  = run->speed  / run_length;

	for (uint8_t r = 0; r < node->rows; r++) {
		float rate = (r == 0) ? walk_rate : run_rate;
		for (uint8_t c = 0; c < node->cols; c++) {
			Animation *clip = character3dAnimation_clip(animation, node->animation[r * node->cols + c]);
			animation_setSpeed(clip, animation_getLength(clip) * rate);
		}
	}
}

/* carries the phase of a grid's dominant column (walk row) to every clip of another */
static void character3dAnimation_snapGridFromGrid(Character3DAnimation *animation, uint8_t src_node_idx, float src_dir, uint8_t dst_node_idx)
{
	const Character3DAnimationNode *src_node = &animation->def->node[src_node_idx];
	const Character3DAnimationNode *dst_node = &animation->def->node[dst_node_idx];

	uint8_t src_col = (uint8_t)(src_dir * (src_node->cols - 1) + 0.5f);
	if (src_col > src_node->cols - 1) src_col = src_node->cols - 1;

	Animation *src = character3dAnimation_clip(animation, src_node->animation[src_col]);
	float src_length = animation_getLength(src);
	if (src_length <= 0.0f) return;
	float phase = src->time / src_length;

	for (int c = 0; c < dst_node->cols * dst_node->rows; c++) {
		Animation *dst = character3dAnimation_clip(animation, dst_node->animation[c]);
		animation_setTime(dst, phase * animation_getLength(dst));
	}
}


/* --- idle ------------------------------------------------------------------- */

/* the idle profile is the footing itself: it holds because the footing does */
static void character3dAnimation_setIdleRightParam(Character3DAnimation *animation)
{
	animation->param[ANIMATION_PARAM_IDLE_RIGHT] =
		animation->def->settings->action_idle_max_blending_ratio * animation->footing;
}


/* --- walk ------------------------------------------------------------------- */

/* Footing wave: 0 on the left plant, 1 on the right one, eased through the
   cycle. The plant phases come from the asset's settings. */
static float character3dAnimation_getLocomotionPhase(const Character3DAnimationSettings *settings, float clip_time, float clip_length)
{
	float left  = settings->footing_left;
	float right = settings->footing_right;

	/* rises left plant -> right plant, and falls at ONE rate across the wrap
	   back to the left plant: no anchor at the cycle seam, so asymmetric
	   plants keep the wave speed continuous */
	float phase = clip_time / clip_length;
	float f;
	if      (phase <= left)  f = (left - phase) / (1.0f - right + left);
	else if (phase <= right) f = (phase - left) / (right - left);
	else                     f = 1.0f - (phase - right) / (1.0f - right + left);
	if (f > 0.9999999f) f = 0.9999999f;
	if (f < 0.0000001f) f = 0.0000001f;
	return f;
}

static float character3dAnimation_getTurningAvg(Character3DAnimation *animation, const Character3DAnimationSettings *settings, float current_yaw, float previous_yaw)
{
	float delta_yaw = current_yaw - previous_yaw;
	if (delta_yaw >  180.0f) delta_yaw -= 360.0f;
	if (delta_yaw <= -180.0f) delta_yaw += 360.0f;

	/* Left as written on purpose: libdragon builds with -ffast-math, so the
	   divide by the constant count is already a multiply, and the modulo by a
	   constant compiles to shifts and adds, not a DIV (checked in the
	   assembly). Nothing to gain by hand. */
	animation->turn_avg[animation->turn_avg_idx] = delta_yaw;
	animation->turn_avg_idx = (animation->turn_avg_idx + 1) % ANIMATION_TURN_AVG_COUNT;

	float sum = 0.0f;
	for (int i = 0; i < ANIMATION_TURN_AVG_COUNT; i++) sum += animation->turn_avg[i];
	float avg_delta_yaw = sum / ANIMATION_TURN_AVG_COUNT;

	float r = avg_delta_yaw / settings->turn_max_angle;

	if (r >  1.0f) r =  1.0f;
	if (r < -1.0f) r = -1.0f;
	if (fabsf(r) < 0.001f) r = 0.0f;
	return r * settings->turn_max_weight;
}

/* The gait axis freezes while the idle takes over: braking would sweep the
   raw value through every gait with the grid still visible. On resuming the
   walk it lerps back to the live value at the gait's own response rate.

   Moving, the axis heads for the higher of the gait the speed is at and the
   gait the stick asks for. Setting off, the stick wins and a run does not
   pass through the walk while the body picks up; braking, the speed wins
   and the axis comes down with the body, never ahead of it. The clips are
   paced on the real speed either way, so the feet hold.

   Returned, not stored: ANIMATION_PARAM_WALK_GAIT has to keep last frame's
   value until the locomotion phase has re-phased its grid on it. */
static float character3dAnimation_getGaitAxis(const Character3D *character, float delta)
{
	const Character3DMovementSettings *movement = character->movement.settings;
	float speed = character->movement.data.horizontal_speed;

	float raw_gait  = character3dAnimation_getGaitParam(speed, movement);
	float prev_gait = character->animation.param[ANIMATION_PARAM_WALK_GAIT];

	uint8_t state = character->movement.current;
	if (!character3dMovement_isLocomotion(state)) state = character->movement.locomotion;

	if (character3dAnimation_getWalkWeight(speed, movement) == 0.0f)
		return raw_gait;

	if (state == MOVEMENT_STATE_IDLE)
		return prev_gait;

	uint8_t last = movement->gait_count - 1;
	uint8_t gait = character->movement.data.gait;
	if (gait > last) gait = last;

	float asked_gait  = (last > 0) ? (float)gait / last : 0.0f;
	float target_gait = (asked_gait > raw_gait) ? asked_gait : raw_gait;

	float factor = fm_expf(-movement->gait[gait].response_rate * delta);
	return prev_gait * factor + target_gait * (1.0f - factor);
}

/* the footing is read from the clip that is actually running: the center
   column of the row the gait sits on. The row comes from the previous
   frame's value, because the clips of a row the axis just reached are only
   brought into phase further down — read now they still hold the time they
   were left at, and the footing jumps for one frame. */
static void character3dAnimation_setFooting(Character3D *character)
{
	Character3DAnimation *animation = &character->animation;
	const Character3DAnimationDef *def = animation->def;
	float prev_gait = animation->param[ANIMATION_PARAM_WALK_GAIT];

	const Character3DAnimationNode *locomotion = &def->node[def->locomotion_node];
	float row_t;
	uint8_t row = character3dAnimation_blendSegment(prev_gait, locomotion->rows, &row_t);
	if (row_t > 0.5f) row++;

	Animation *base = character3dAnimation_clip(animation, locomotion->animation[row * locomotion->cols + locomotion->cols / 2]);

	/* The footing follows the clip only while the body moves: stopped, the
	   clip's phase is whatever it froze at, and the idle already settled on
	   the foot it had. Everyone downstream reads the held value. */
	if (character->movement.data.horizontal_speed > 0.0f)
		animation->footing = character3dAnimation_getLocomotionPhase(def->settings, base->time, animation_getLength(base));
}

static void character3dAnimation_setJumpFootingSpeed(Character3D *character)
{
	if (!character3dAnimation_isAerial(character)) return;

	Character3DAnimation *animation = &character->animation;
	const Character3DAnimationDef *def = animation->def;

	float jump   = animation->param[ANIMATION_PARAM_JUMP_L] + animation->param[ANIMATION_PARAM_JUMP_R];
	float factor = def->settings->jump_footing_speed * (1.0f - jump);
	if (factor < 0.0f) factor = 0.0f;

	character3dAnimation_clip(animation, def->walk_animation)->speed          *= factor;
	character3dAnimation_clip(animation, def->run_animation)->speed           *= factor;
	character3dAnimation_clip(animation, def->sprint_animation)->speed        *= factor;
	character3dAnimation_clip(animation, def->turn_walk_animation)->speed     *= factor;
	character3dAnimation_clip(animation, def->turn_walk_animation + 1)->speed *= factor;
	character3dAnimation_clip(animation, def->turn_run_animation)->speed      *= factor;
	character3dAnimation_clip(animation, def->turn_run_animation + 1)->speed  *= factor;
}

/* the grid runs at the cycle rate the current gait asks for: real speed over
   target speed, and every clip gets the speed that makes its cycle last that long */
static void character3dAnimation_setLocomotionSpeed(Character3D *character, float gait)
{
	Character3DAnimation *animation = &character->animation;
	const Character3DAnimationDef *def = animation->def;
	const Character3DAnimationNode *node = &def->node[def->locomotion_node];
	const Character3DMovementSettings *movement = character->movement.settings;
	uint8_t center = node->cols / 2;

	float t;
	uint8_t row = character3dAnimation_blendSegment(gait, node->rows, &t);

	float low  = animation_getLength(character3dAnimation_clip(animation, node->animation[row * node->cols + center]));
	float high = animation_getLength(character3dAnimation_clip(animation, node->animation[(row + 1) * node->cols + center]));
	float length = low + t * (high - low);

	float target = movement->gait[row].target_speed
	             + t * (movement->gait[row + 1].target_speed - movement->gait[row].target_speed);

	if (length <= 0.0f || target <= 0.0f) return;

	float scale = (character->movement.data.horizontal_speed / target) / length;

	for (int i = 0; i < node->cols * node->rows; i++) {
		Animation *clip = character3dAnimation_clip(animation, node->animation[i]);
		animation_setSpeed(clip, animation_getLength(clip) * scale);
	}

	character3dAnimation_setJumpFootingSpeed(character);
}

static void character3dAnimation_setLocomotionParam(Character3D *character, float gait)
{
	Character3DAnimation *animation = &character->animation;
	const Character3DAnimationDef *def = animation->def;
	const Character3DAnimationNode *node = &def->node[def->locomotion_node];
	const Character3DMovementSettings *movement = character->movement.settings;
	float speed = character->movement.data.horizontal_speed;

	float turning = character3dAnimation_getTurningAvg(animation, def->settings, character->body.rotation.z, character->movement.data.previous_yaw);

	/* turn axis: 0 left, 0.5 straight, 1 right */
	float turn = (turning + 1.0f) * 0.5f;

	character3dAnimation_syncGridClips(animation, node, turn, gait);

	animation->param[ANIMATION_PARAM_WALK]      = character3dAnimation_getWalkWeight(speed, movement);
	animation->param[ANIMATION_PARAM_WALK_GAIT] = gait;
	animation->param[ANIMATION_PARAM_WALK_TURN] = turn;
}


/* --- strafe ----------------------------------------------------------------- */

static float character3dAnimation_getStrafeDirectionWeight(Character3D *character, float delta)
{
	Character3DAnimation *animation = &character->animation;
	const Character3DAnimationSettings *s = animation->def->settings;
	const KinematicBody *body = &character->body;

	if (body->velocity.x == 0.0f && body->velocity.y == 0.0f)
		return animation->param[ANIMATION_PARAM_STRAFE_DIR];

	float velocity_yaw = rad_to_deg(fm_atan2f(-body->velocity.x, -body->velocity.y));
	float rel = angle_wrap_relative(velocity_yaw, body->rotation.z) - body->rotation.z;

	/* axis:    back 0 | back_l 1/6 | strafe_l 2/6 | fwd 3/6 | strafe_r 4/6 | back_r 5/6 | back 1
	   anchors: fwd 0º, strafe ±90º, back_l/back_r ±90º on the back side, back ±180º
	   the 1/6-2/6 and 4/6-5/6 stretches are never reached by direction: they are the hip turn */
	float raw;
	if      (rel < -90.0f) raw = (rel + 180.0f) / 90.0f          * (1.0f / 6.0f);
	else if (rel <   0.0f) raw = (2.0f + (rel + 90.0f) / 90.0f)  * (1.0f / 6.0f);
	else if (rel <  90.0f) raw = (3.0f + rel / 90.0f)            * (1.0f / 6.0f);
	else                   raw = (5.0f + (rel - 90.0f) / 90.0f)  * (1.0f / 6.0f);

	if (animation->param[ANIMATION_PARAM_STRAFE] == 0.0f) return raw;

	float out = animation->param[ANIMATION_PARAM_STRAFE_DIR];

	bool front_raw = (raw >= 2.0f / 6.0f && raw <= 4.0f / 6.0f);
	bool front_out = (out >= 2.0f / 6.0f && out <= 4.0f / 6.0f);

	if (!animation->strafe_turning) {
		if (front_raw == front_out) return raw;
		animation->strafe_turning = true;
	}

	/* ends 0 and 1 of the axis are the same clip: if the target is more than
	   half the axis away, the shortest path crosses the seam */
	if (raw - out > 0.5f) raw -= 1.0f;
	if (out - raw > 0.5f) raw += 1.0f;

	/* exponential lerp toward the live weight, released once it lands */
	float factor = fm_expf(-s->strafe_turn_rate * delta);
	out = out * factor + raw * (1.0f - factor);

	if (fabsf(out - raw) < 0.001f) {
		out = raw;
		animation->strafe_turning = false;
	}

	if (out < 0.0f) out += 1.0f;
	if (out > 1.0f) out -= 1.0f;

	return out;
}

static void character3dAnimation_snapStrafeEntry(Character3DAnimation *animation)
{
	const Character3DAnimationDef *def = animation->def;
	const Character3DAnimationNode *node = &def->node[def->strafe_node];

	Animation *src = character3dAnimation_clip(animation, def->walk_animation);
	float src_length = animation_getLength(src);
	if (src_length <= 0.0f) return;
	float phase = src->time / src_length;

	for (int c = 0; c < node->cols * node->rows; c++) {
		Animation *dst = character3dAnimation_clip(animation, node->animation[c]);
		if (dst == src) continue;
		animation_setTime(dst, phase * animation_getLength(dst));
	}
}

static void character3dAnimation_snapStrafeExit(Character3DAnimation *animation)
{
	const Character3DAnimationDef *def = animation->def;
	const Character3DAnimationNode *node = &def->node[def->strafe_node];

	/* source: the walk-row clip closest to the current weight, already active */
	float out = animation->param[ANIMATION_PARAM_STRAFE_DIR];
	uint8_t src_col = (uint8_t)(out * (node->cols - 1) + 0.5f);
	if (src_col > node->cols - 1) src_col = node->cols - 1;

	Animation *src = character3dAnimation_clip(animation, node->animation[src_col]);
	float src_length = animation_getLength(src);
	if (src_length <= 0.0f) return;
	float phase = src->time / src_length;

	const uint8_t target[] = {
		def->walk_animation, def->run_animation, def->sprint_animation,
		def->turn_walk_animation, (uint8_t)(def->turn_walk_animation + 1),
		def->turn_run_animation,  (uint8_t)(def->turn_run_animation + 1),
	};

	for (unsigned t = 0; t < sizeof(target); t++) {
		Animation *dst = character3dAnimation_clip(animation, target[t]);
		animation_setTime(dst, phase * animation_getLength(dst));
	}
}

static void character3dAnimation_setStrafeParams(Character3D *character, float delta)
{
	Character3DAnimation *animation = &character->animation;
	const Character3DAnimationDef *def = animation->def;
	const Character3DMovementData *data = &character->movement.data;
	const Character3DMovementSettings *movement = character->movement.settings;

	/* Aiming owns the pose while either of its flags is up: the free strafe
	   bows out entirely, so its exit can never hand the locomotion a phase
	   gone stale while its grid sat frozen underneath. */
	/* The air keeps the grid up: the jump and landing layers sit on top of
	   it, and the landing comes back down onto the same strafe it left. */
	uint8_t current = character->movement.current;
	bool strafing = data->strafe
		&& !data->aiming && !data->charging_shoot
		&& (character3dMovement_isLocomotion(current) || current == MOVEMENT_STATE_FALLING)
		&& data->horizontal_speed > 0.0f;

	/* the strafe grid takes over the locomotion one gradually; the ramp is
	   only worth computing while it is up or on its way */
	float prev_blend = animation->strafe_blend;
	float blend = 0.0f;
	if (strafing || prev_blend > 0.0f) {
		float factor = fm_expf(-def->settings->strafe_blend_rate * delta);
		blend = strafing ? 1.0f - (1.0f - prev_blend) * factor : prev_blend * factor;
		if (blend > 0.999f) blend = 1.0f;
		if (blend < 0.001f) blend = 0.0f;
	}

	if (blend == 0.0f) {
		if (prev_blend > 0.0f) character3dAnimation_snapStrafeExit(animation);
		animation->strafe_blend = 0.0f;
		animation->param[ANIMATION_PARAM_STRAFE] = 0.0f;
		animation->strafe_turning = false;
		return;
	}

	if (prev_blend == 0.0f)
		character3dAnimation_snapStrafeEntry(animation);

	const Character3DAnimationNode *node = &def->node[def->strafe_node];
	character3dAnimation_setGridRowSpeeds(animation, node);

	float dir = character3dAnimation_getStrafeDirectionWeight(character, delta);

	float gait = (data->horizontal_speed - movement->gait[0].target_speed)
	           / (movement->gait[1].target_speed - movement->gait[0].target_speed);
	if (gait < 0.0f) gait = 0.0f;
	if (gait > 1.0f) gait = 1.0f;

	character3dAnimation_syncGridClips(animation, node, dir, gait);

	float weight = character3dAnimation_getWalkWeight(data->horizontal_speed, movement);

	animation->strafe_blend = blend;
	animation->param[ANIMATION_PARAM_STRAFE]      = weight * blend;
	animation->param[ANIMATION_PARAM_STRAFE_DIR]  = dir;
	animation->param[ANIMATION_PARAM_STRAFE_GAIT] = gait;

	animation->param[ANIMATION_PARAM_WALK] = weight * (1.0f - blend);
}

static void character3dAnimation_setStrafeLockedParams(Character3D *character, float delta)
{
	Character3DAnimation *animation = &character->animation;
	const Character3DAnimationDef *def = animation->def;
	const Character3DMovementData *data = &character->movement.data;
	const Character3DMovementSettings *movement = character->movement.settings;

	bool locked = data->strafe_locked
		&& character3dMovement_isLocomotion(character->movement.current)
		&& data->horizontal_speed > 0.0f;

	float prev_blend = animation->strafe_locked_blend;
	float blend = 0.0f;
	if (locked || prev_blend > 0.0f) {
		float factor = fm_expf(-def->settings->strafe_locked_blend_rate * delta);
		blend = locked ? 1.0f - (1.0f - prev_blend) * factor : prev_blend * factor;
		if (blend > 0.999f) blend = 1.0f;
		if (blend < 0.001f) blend = 0.0f;
	}

	if (blend == 0.0f) {
		if (prev_blend > 0.0f)
			character3dAnimation_snapLocomotionFromGrid(animation, def->strafe_locked_node,
			                                          animation->param[ANIMATION_PARAM_STRAFE_LOCKED_DIR]);
		animation->strafe_locked_blend = 0.0f;
		animation->param[ANIMATION_PARAM_STRAFE_LOCKED] = 0.0f;
		return;
	}

	if (prev_blend == 0.0f)
		character3dAnimation_snapGridFromClip(animation, def->walk_animation, def->strafe_locked_node);

	const Character3DAnimationNode *node = &def->node[def->strafe_locked_node];
	character3dAnimation_setGridRowSpeeds(animation, node);

	float dir = character3dAnimation_getLockedDirectionWeight(character, ANIMATION_PARAM_STRAFE_LOCKED_DIR);

	float gait = (data->horizontal_speed - movement->gait[0].target_speed)
	           / (movement->gait[1].target_speed - movement->gait[0].target_speed);
	if (gait < 0.0f) gait = 0.0f;
	if (gait > 1.0f) gait = 1.0f;

	character3dAnimation_syncGridClips(animation, node, dir, gait);

	float weight = character3dAnimation_getWalkWeight(data->horizontal_speed, movement);

	animation->strafe_locked_blend = blend;
	animation->param[ANIMATION_PARAM_STRAFE_LOCKED]      = weight * blend;
	animation->param[ANIMATION_PARAM_STRAFE_LOCKED_DIR]  = dir;
	animation->param[ANIMATION_PARAM_STRAFE_LOCKED_GAIT] = gait;

	animation->param[ANIMATION_PARAM_WALK]   *= (1.0f - blend);
	animation->param[ANIMATION_PARAM_STRAFE] *= (1.0f - blend);
}


/* --- jump ------------------------------------------------------------------- */

static void character3dAnimation_syncLandToJump(Character3DAnimation *animation)
{
	const Character3DAnimationDef *def = animation->def;
	const Character3DAnimationSettings *j = def->settings;
	Animation *jump_l   = character3dAnimation_clip(animation, def->jump_animation);
	Animation *jump_r   = character3dAnimation_clip(animation, def->jump_animation + 1);
	float    land_t   = character3dAnimation_clip(animation, def->land_animation)->time;
	float    jump_t;

	if (land_t < j->land_anim_crouch)
		jump_t = (land_t / j->land_anim_crouch) * j->jump_anim_crouch;
	else
		jump_t = (1.0f - (land_t - j->land_anim_crouch) / (j->land_anim_stand - j->land_anim_crouch)) * j->jump_anim_crouch;

	if (jump_t < 0.0f)              jump_t = 0.0f;
	if (jump_t > j->jump_anim_crouch) jump_t = j->jump_anim_crouch;

	animation_setTime(jump_l, jump_t);
	animation_setTime(jump_r, jump_t);
}

static void character3dAnimation_snapToJump(Character3DAnimation *animation)
{
	const Character3DAnimationDef *def = animation->def;
	Animation *jump_l   = character3dAnimation_clip(animation, def->jump_animation);
	Animation *jump_r   = character3dAnimation_clip(animation, def->jump_animation + 1);
	Animation *land_animation = character3dAnimation_clip(animation, def->land_animation);

	Animation *fall_l = character3dAnimation_clip(animation, def->fall_animation);
	Animation *fall_r = character3dAnimation_clip(animation, def->fall_animation + 1);

	animation_setPlaying(jump_l, true);
	animation_setPlaying(jump_r, true);

	animation_setTime(fall_l, 0.0f);
	animation_setTime(fall_r, 0.0f);

	/* Jumping straight out of a landing: the take-off starts at the crouch
	   depth the landing is already holding, so the pose does not jump. With no
	   landing running there is nothing to match and it starts from the top. */
	if (land_animation->isPlaying) {
		character3dAnimation_syncLandToJump(animation);
	} else {
		animation_setTime(jump_l, 0.0f);
		animation_setTime(jump_r, 0.0f);
	}

	animation->param[ANIMATION_PARAM_JUMP_L] = 0.0f;
	animation->param[ANIMATION_PARAM_JUMP_R] = 0.0f;
}

static void character3dAnimation_snapToLand(Character3DAnimation *animation)
{
	const Character3DAnimationDef *def = animation->def;
	Animation *land_l = character3dAnimation_clip(animation, def->land_animation);
	Animation *land_r = character3dAnimation_clip(animation, def->land_animation + 1);
	animation_setTime(land_l, 0.0f);
	animation_setTime(land_r, 0.0f);
	animation_setPlaying(land_l, true);
	animation_setPlaying(land_r, true);
	animation->param[ANIMATION_PARAM_LAND_L] = 0.0f;
	animation->param[ANIMATION_PARAM_LAND_R] = 0.0f;
}

/* Falling with no crouch behind it — off a ledge, or a roll that ran out of
   ground. The take-off clip never played, so the sequence is sent straight to
   the falling one by marking it done. */
static void character3dAnimation_snapToFall(Character3DAnimation *animation)
{
	const Character3DAnimationDef *def = animation->def;
	Animation *jump_l = character3dAnimation_clip(animation, def->jump_animation);
	Animation *jump_r = character3dAnimation_clip(animation, def->jump_animation + 1);
	Animation *fall_l = character3dAnimation_clip(animation, def->fall_animation);
	Animation *fall_r = character3dAnimation_clip(animation, def->fall_animation + 1);

	animation_setPlaying(jump_l, false);
	animation_setPlaying(jump_r, false);
	animation_setTime(fall_l, 0.0f);
	animation_setTime(fall_r, 0.0f);
}

static void character3dAnimation_setJumpParams(Character3D *character, float delta)
{
	Character3DAnimation *animation = &character->animation;
	const Character3DAnimationDef *def = animation->def;
	const Character3DAnimationSettings *j = def->settings;
	Animation *land_animation = character3dAnimation_clip(animation, def->land_animation);
	uint8_t  cur     = character->movement.current;
	uint8_t *as      = &animation->action_state;
	float    footing = animation->footing;

	float jump = animation->param[ANIMATION_PARAM_JUMP_L] + animation->param[ANIMATION_PARAM_JUMP_R];
	float land = animation->param[ANIMATION_PARAM_LAND_L] + animation->param[ANIMATION_PARAM_LAND_R];

	/* One owner for the air layer: the crouch on the ground opens it and the
	   fall keeps it. Entering with a crouch plays the take-off clip; entering
	   without one starts on the falling clip. */
	bool aerial = character3dAnimation_isAerial(character);

	if (aerial && *as != MOVEMENT_STATE_FALLING) {
		if (character->movement.data.jump_timer > 0.0f)
			character3dAnimation_snapToJump(animation);
		else
			character3dAnimation_snapToFall(animation);

		jump = 0.0f;
		*as  = MOVEMENT_STATE_FALLING;
	}

	if (*as == MOVEMENT_STATE_FALLING && !aerial)
		*as = cur;

	/* The landing starts one clip-to-contact away from the floor, measured by
	   the fall probe: the foot meets the ground on the frame the clip has it
	   touching, whatever the drop was. */
	float floor_distance = character->movement.data.floor_distance;
	float fall_speed     = -character->body.velocity.z;

	if (aerial && !land_animation->isPlaying && floor_distance >= 0.0f && fall_speed > 0.0f
	    && floor_distance <= fall_speed * j->land_anim_ground) {
		character3dAnimation_snapToLand(animation);
		land = 0.0f;
	}

	/* The landing weight is full on the contact frame, so the pose at touchdown
	   is the landing clip alone. Until then the falling layer drains at the same
	   rate, otherwise it keeps its full weight against the landing all the way
	   down. */
	if (land_animation->isPlaying) {
		if (land_animation->time < j->land_anim_ground) {
			float ground_rate = j->jump_max_blending_ratio * delta / j->land_anim_ground;
			land += ground_rate;
			if (land > j->jump_max_blending_ratio) land = j->jump_max_blending_ratio;
			jump -= ground_rate;
			if (jump < 0.0f) jump = 0.0f;
		} else if (land_animation->time < j->land_anim_crouch) {
			land = j->jump_max_blending_ratio;
		} else {
			float stand_rate = j->jump_max_blending_ratio * delta / (j->land_anim_length - j->land_anim_crouch);
			land -= stand_rate;
			if (land < 0.0f) {
				land = 0.0f;
				animation_setPlaying(land_animation, false);
				animation_setPlaying(character3dAnimation_clip(animation, def->land_animation + 1), false);
			}
		}

	}

	/* The air layer only grows while no landing is running: once the landing
	   has started it owns the drain above. */
	if (aerial && !land_animation->isPlaying) {
		jump += j->jump_max_blending_ratio * delta / j->jump_anim_crouch;
		if (jump > j->jump_max_blending_ratio) jump = j->jump_max_blending_ratio;
	}
	/* Back on the ground the air layer drains on its own. Tied to the landing
	   clip it left a remnant, because that clip had already run most of its
	   length during the drop and ended before the weight was gone. */
	else if (!aerial && jump > 0.0f) {
		jump -= j->jump_max_blending_ratio * delta / j->land_anim_crouch;
		if (jump < 0.0f) jump = 0.0f;
	}

	animation->param[ANIMATION_PARAM_JUMP_L] = jump * (1.0f - footing);
	animation->param[ANIMATION_PARAM_JUMP_R] = jump * footing;
	animation->param[ANIMATION_PARAM_LAND_L] = land * (1.0f - footing);
	animation->param[ANIMATION_PARAM_LAND_R] = land * footing;
}


/* --- roll ------------------------------------------------------------------- */

/* Weight the roll sheds per second on its way out: the exit ramp runs from the
   stand pose to the end of the clip. */
static float character3dAnimation_rollExitRate(const Character3DAnimationSettings *r)
{
	return 1.0f / (r->run_to_rolling_anim_length - r->run_to_rolling_anim_stand);
}

static void character3dAnimation_snapRollToLocomotion(Character3DAnimation *animation, bool left)
{
	const Character3DAnimationDef *def = animation->def;
	const Character3DAnimationSettings *settings = def->settings;
	const Character3DAnimationNode *node = &def->node[def->locomotion_node];

	/* the plant phases are where the footing wave peaks: footing_left is
	   footing 0, footing_right is footing 1. The exit lands short of the
	   plant so the leg is still reaching for it. */
	float phase = left ? settings->footing_right : settings->footing_left;
	phase -= settings->run_to_rolling_anim_lead;
	if (phase < 0.0f) phase += 1.0f;

	for (int i = 0; i < node->cols * node->rows; i++) {
		Animation *clip = character3dAnimation_clip(animation, node->animation[i]);
		animation_setTime(clip, phase * animation_getLength(clip));
	}
}

static void character3dAnimation_setRollParam(Character3D *character, float delta)
{
	Character3DAnimation *animation = &character->animation;
	const Character3DAnimationDef *def = animation->def;
	const Character3DAnimationSettings *r = def->settings;
	uint8_t  cur = character->movement.current;
	uint8_t *as  = &animation->action_state;

	if (cur != MOVEMENT_STATE_ROLLING) {
		if (*as == MOVEMENT_STATE_ROLLING) *as = cur;

		/* Cut short by a ledge: the clip never reached its own exit ramp, so
		   the weight is drained here at that same rate instead of dropping
		   the pose in one frame. */
		float ratio = animation->param[ANIMATION_PARAM_ROLL_RUN];
		if (ratio > 0.0f) {
			ratio -= delta * character3dAnimation_rollExitRate(r);
			if (ratio < 0.0f) ratio = 0.0f;
		}

		animation->param[ANIMATION_PARAM_ROLL_RUN] = ratio;
		if (ratio == 0.0f) animation->param[ANIMATION_PARAM_ROLL_DIR] = 0.0f;
		return;
	}

	if (*as != MOVEMENT_STATE_ROLLING) {
		uint8_t base = def->roll_animation;
		Animation *roll_l = character3dAnimation_clip(animation, base);
		Animation *roll_r = character3dAnimation_clip(animation, base + 1);

		animation_setPlaying(roll_l, true);
		animation_setTime   (roll_l, 0.0f);
		animation_setPlaying(roll_r, true);
		animation_setTime   (roll_r, 0.0f);

		float dir = (animation->footing < 0.5f) ? -1.0f : 1.0f;
		animation->param[ANIMATION_PARAM_ROLL_RUN] = 0.0f;
		animation->param[ANIMATION_PARAM_ROLL_DIR] = dir;
		*as = MOVEMENT_STATE_ROLLING;
	}

	float dir   = animation->param[ANIMATION_PARAM_ROLL_DIR];
	bool  left  = dir < 0.0f;

	uint8_t base      = def->roll_animation;
	uint8_t roll_idx  = left ? base : base + 1;
	float   roll_time = character3dAnimation_clip(animation, roll_idx)->time;
	float   ratio     = animation->param[ANIMATION_PARAM_ROLL_RUN];

	if (roll_time < r->run_to_rolling_anim_ground && ratio <= 1.0f)
		ratio += delta / r->run_to_rolling_anim_ground;

	if (roll_time > r->run_to_rolling_anim_stand && ratio > 0.0f)
		ratio -= delta * character3dAnimation_rollExitRate(r);

	if (ratio > 1.0f) {
		ratio = 1.0f;
		character3dAnimation_snapRollToLocomotion(animation, left);
	}

	if (ratio < 0.0f) ratio = 0.0f;

	animation->param[ANIMATION_PARAM_ROLL_RUN] = ratio;
}


/* --- swim ------------------------------------------------------------------- */

/* The swim clips run at their own native lengths; blending two strokes of
   different period desyncs the arms mid-blend. Same cure as the locomotion
   grid: the cycle length is interpolated at the blend point and every clip
   gets the speed that makes its cycle last exactly that long. */
static void character3dAnimation_setSwimSpeed(Character3DAnimation *animation,
                                            const Character3DAnimationNode *node, float gait)
{
	float t;
	uint8_t col = character3dAnimation_blendSegment(gait, node->cols, &t);

	float low    = animation_getLength(character3dAnimation_clip(animation, node->animation[col]));
	float high   = animation_getLength(character3dAnimation_clip(animation, node->animation[col + 1]));
	float length = low + t * (high - low);
	if (length <= 0.0f) return;

	/* one divide for the grid: gcc leaves a div.s per iteration otherwise */
	float rate = 1.0f / length;

	for (int i = 0; i < node->cols * node->rows; i++) {
		Animation *clip = character3dAnimation_clip(animation, node->animation[i]);
		animation_setSpeed(clip, animation_getLength(clip) * rate);
	}
}

/* Swim grid: weight is a timed ramp gated by the SWIMMING state, never the
   raw submersion — the waves oscillate the submerged fraction and would
   jitter the blend. The gait axis crosses idle -> slow -> fast strokes by
   the horizontal speed. While the ramp is up, every land-borne param fades
   with it: the water owns the pose. */
static void character3dAnimation_setSwimParams(Character3D *character, float delta)
{
	Character3DAnimation *animation = &character->animation;
	const Character3DAnimationDef *def = animation->def;
	const Character3DMovementData *data = &character->movement.data;
	const Character3DMovementSettings *movement = character->movement.settings;

	bool swimming = character->movement.current == MOVEMENT_STATE_SWIMMING;

	float prev_blend = animation->swim_blend;
	float blend = 0.0f;
	if (swimming || prev_blend > 0.0f) {
		float factor = fm_expf(-def->settings->swim_blend_rate * delta);
		blend = swimming ? 1.0f - (1.0f - prev_blend) * factor : prev_blend * factor;
		if (blend > 0.999f) blend = 1.0f;
		if (blend < 0.001f) blend = 0.0f;
	}

	animation->swim_blend = blend;

	if (blend == 0.0f) {
		animation->param[ANIMATION_PARAM_SWIM] = 0.0f;
		return;
	}

	const Character3DAnimationNode *node = &def->node[def->swim_node];

	/* Fading in from nothing: restart the strokes so they enter in phase. */
	if (prev_blend == 0.0f)
		for (int c = 0; c < node->cols * node->rows; c++)
			animation_setTime(character3dAnimation_clip(animation, node->animation[c]), 0.0f);

	float speed = data->horizontal_speed;
	float gait;
	if (speed <= movement->swim_slow_speed)
		gait = 0.5f * speed / movement->swim_slow_speed;
	else
		gait = 0.5f + 0.5f * (speed - movement->swim_slow_speed)
		            / (movement->swim_fast_speed - movement->swim_slow_speed);
	if (gait < 0.0f) gait = 0.0f;
	if (gait > 1.0f) gait = 1.0f;

	character3dAnimation_setSwimSpeed(animation, node, gait);
	character3dAnimation_syncGridClips(animation, node, gait, 0.0f);

	animation->param[ANIMATION_PARAM_SWIM]      = blend;
	animation->param[ANIMATION_PARAM_SWIM_GAIT] = gait;

	animation->param[ANIMATION_PARAM_WALK]          *= (1.0f - blend);
	animation->param[ANIMATION_PARAM_STRAFE]        *= (1.0f - blend);
	animation->param[ANIMATION_PARAM_STRAFE_LOCKED] *= (1.0f - blend);
	animation->param[ANIMATION_PARAM_AIMING]      *= (1.0f - blend);
	animation->param[ANIMATION_PARAM_AIMING_IDLE] *= (1.0f - blend);
	animation->param[ANIMATION_PARAM_CHARGING_SHOOT]       *= (1.0f - blend);
	animation->param[ANIMATION_PARAM_CHARGING_SHOOT_IDLE]  *= (1.0f - blend);
	animation->param[ANIMATION_PARAM_JUMP_L]        *= (1.0f - blend);
	animation->param[ANIMATION_PARAM_JUMP_R]        *= (1.0f - blend);
	animation->param[ANIMATION_PARAM_LAND_L]        *= (1.0f - blend);
	animation->param[ANIMATION_PARAM_LAND_R]        *= (1.0f - blend);
}


/* --- climb ------------------------------------------------------------------ */

/* Climb layer: same timed ramp as the swim, gated by the CLIMBING state.

   The cycle is timed against distance, not the clock — the hands only land
   on the rungs if one cycle of the clip lasts exactly one rung spacing, so
   the clip speed is the climb speed measured in cycles-worth-of-height per
   second. Stopped on the ladder that speed is zero and the pose freezes
   mid-grip, which is what hanging there looks like.

   The direction only picks which clip the select plays and holds its last
   non-zero value: at a standstill the arms must stay where the last move
   left them rather than snap to a default. */
static void character3dAnimation_setClimbParams(Character3D *character, float delta)
{
	Character3DAnimation *animation = &character->animation;
	const Character3DAnimationDef *def = animation->def;
	const Character3DMovementSettings *movement = character->movement.settings;

	bool climbing = character->movement.current == MOVEMENT_STATE_CLIMBING;

	float prev_blend = animation->climb_blend;
	float blend = 0.0f;
	if (climbing || prev_blend > 0.0f) {
		float factor = fm_expf(-def->settings->climb_blend_rate * delta);
		blend = climbing ? 1.0f - (1.0f - prev_blend) * factor : prev_blend * factor;
		if (blend > 0.999f) blend = 1.0f;
		if (blend < 0.001f) blend = 0.0f;
	}

	animation->climb_blend = blend;

	/* The direction gates the select as well as picking its clip: zeroed off
	   the ladder, the pair stops being stepped every frame for a layer that
	   is contributing nothing. The held direction lives outside the param. */
	if (blend == 0.0f) {
		animation->param[ANIMATION_PARAM_CLIMB]     = 0.0f;
		animation->param[ANIMATION_PARAM_CLIMB_DIR] = 0.0f;
		return;
	}

	const Character3DAnimationNode *node = &def->node[def->climb_node];

	float velocity = character->body.velocity.z;

	if (velocity >  LOCOMOTION_MIN_SPEED) animation->climb_dir =  1.0f;
	if (velocity < -LOCOMOTION_MIN_SPEED) animation->climb_dir = -1.0f;
	if (animation->climb_dir == 0.0f)     animation->climb_dir =  1.0f;

	/* The cycle runs on how fast the body is actually moving as a fraction
	   of the speed the climb tops out at: full tilt lands on the clip's own
	   pace and nothing plays it faster, while accelerating into the climb
	   and easing out of it slow the cycle to match. Stopped on the ladder
	   it is zero and the pose holds mid-grip, which is what hanging there
	   looks like.

	   Both clips share the slot, so the one that is not playing has to be
	   kept fed with the same speed: the select hands the time over on a
	   direction change and a stale speed would jump the cycle. */
	float speed = (movement->climb_speed > 0.0f)
		? fabsf(velocity) / movement->climb_speed : 0.0f;

	for (uint8_t i = 0; i < node->cols * node->rows; i++)
		animation_setSpeed(character3dAnimation_clip(animation, node->animation[i]), speed);

	animation->param[ANIMATION_PARAM_CLIMB]     = blend;
	animation->param[ANIMATION_PARAM_CLIMB_DIR] = animation->climb_dir;

	animation->param[ANIMATION_PARAM_WALK]          *= (1.0f - blend);
	animation->param[ANIMATION_PARAM_STRAFE]        *= (1.0f - blend);
	animation->param[ANIMATION_PARAM_STRAFE_LOCKED] *= (1.0f - blend);
	animation->param[ANIMATION_PARAM_AIMING]      *= (1.0f - blend);
	animation->param[ANIMATION_PARAM_AIMING_IDLE] *= (1.0f - blend);
	animation->param[ANIMATION_PARAM_CHARGING_SHOOT]       *= (1.0f - blend);
	animation->param[ANIMATION_PARAM_CHARGING_SHOOT_IDLE]  *= (1.0f - blend);
	animation->param[ANIMATION_PARAM_SWIM]          *= (1.0f - blend);
	animation->param[ANIMATION_PARAM_JUMP_L]        *= (1.0f - blend);
	animation->param[ANIMATION_PARAM_JUMP_R]        *= (1.0f - blend);
	animation->param[ANIMATION_PARAM_LAND_L]        *= (1.0f - blend);
	animation->param[ANIMATION_PARAM_LAND_R]        *= (1.0f - blend);
}


/* --- aim -------------------------------------------------------------------- */

/* Aiming modes: the locked grid twice over, plus an idle of their own per
   mode, so a standstill keeps the weapon up instead of dropping to the bare
   idle. The charge owns the pose while both flags are on: the ready pose
   fades under it and comes back when the shot is let go.

   Entering from plain locomotion the grids inherit the walk cycle's phase;
   hopping between the modes they hand it to each other, and the way out
   returns it, so the feet never skip. */
static void character3dAnimation_setAimingParams(Character3D *character, float delta)
{
	Character3DAnimation *animation = &character->animation;
	const Character3DAnimationDef *def = animation->def;
	const Character3DMovementData *data = &character->movement.data;
	const Character3DMovementSettings *movement = character->movement.settings;

	/* Node 0 is the base idle clip: a def without the module leaves these
	   fields zeroed and the whole thing stays out of the graph. */
	if (def->aiming_node == 0) return;

	bool locomotion = character3dMovement_isLocomotion(character->movement.current);
	bool charging = data->charging_shoot && locomotion;
	bool ready    = data->aiming && locomotion && !charging;

	float prev_ready    = animation->aiming_blend;
	float prev_charging = animation->charging_shoot_blend;

	float ready_blend    = 0.0f;
	float charging_blend = 0.0f;

	if (ready || prev_ready > 0.0f) {
		float factor = fm_expf(-def->settings->aiming_blend_rate * delta);
		ready_blend = ready ? 1.0f - (1.0f - prev_ready) * factor : prev_ready * factor;
		if (ready_blend > 0.999f) ready_blend = 1.0f;
		if (ready_blend < 0.001f) ready_blend = 0.0f;
	}

	if (charging || prev_charging > 0.0f) {
		float factor = fm_expf(-def->settings->charging_shoot_blend_rate * delta);
		charging_blend = charging ? 1.0f - (1.0f - prev_charging) * factor : prev_charging * factor;
		if (charging_blend > 0.999f) charging_blend = 1.0f;
		if (charging_blend < 0.001f) charging_blend = 0.0f;
	}

	animation->aiming_blend         = ready_blend;
	animation->charging_shoot_blend = charging_blend;

	if (ready_blend == 0.0f && charging_blend == 0.0f) {
		animation->param[ANIMATION_PARAM_AIMING]      = 0.0f;
		animation->param[ANIMATION_PARAM_AIMING_IDLE] = 0.0f;
		animation->param[ANIMATION_PARAM_CHARGING_SHOOT]       = 0.0f;
		animation->param[ANIMATION_PARAM_CHARGING_SHOOT_IDLE]  = 0.0f;
		return;
	}

	/* Entries: the cycle comes from whoever carried it last, and the idle
	   restarts so it never wakes mid-breath. */
	if (prev_ready == 0.0f && ready_blend > 0.0f) {
		if (prev_charging > 0.0f)
			character3dAnimation_snapGridFromGrid(animation, def->charging_shoot_node,
				animation->param[ANIMATION_PARAM_CHARGING_SHOOT_DIR], def->aiming_node);
		else
			character3dAnimation_snapGridFromClip(animation, def->walk_animation, def->aiming_node);
		animation_setTime(character3dAnimation_clip(animation, def->node[def->aiming_idle_node].animation[0]), 0.0f);
	}
	if (prev_charging == 0.0f && charging_blend > 0.0f) {
		if (prev_ready > 0.0f)
			character3dAnimation_snapGridFromGrid(animation, def->aiming_node,
				animation->param[ANIMATION_PARAM_AIMING_DIR], def->charging_shoot_node);
		else
			character3dAnimation_snapGridFromClip(animation, def->walk_animation, def->charging_shoot_node);
		animation_setTime(character3dAnimation_clip(animation, def->node[def->charging_shoot_idle_node].animation[0]), 0.0f);
	}

	float gait = (data->horizontal_speed - movement->gait[0].target_speed)
	           / (movement->gait[1].target_speed - movement->gait[0].target_speed);
	if (gait < 0.0f) gait = 0.0f;
	if (gait > 1.0f) gait = 1.0f;

	/* Splits each mode between its grid and its idle; only the grids of a
	   mode that weighs something get touched, so the other one's clips can
	   close behind it. */
	float weight = character3dAnimation_getWalkWeight(data->horizontal_speed, movement);

	if (ready_blend > 0.0f) {
		const Character3DAnimationNode *node = &def->node[def->aiming_node];
		float dir = character3dAnimation_getLockedDirectionWeight(character, ANIMATION_PARAM_AIMING_DIR);
		character3dAnimation_setGridRowSpeeds(animation, node);
		character3dAnimation_syncGridClips(animation, node, dir, gait);
		animation->param[ANIMATION_PARAM_AIMING]      = weight * ready_blend;
		animation->param[ANIMATION_PARAM_AIMING_IDLE] = (1.0f - weight) * ready_blend;
		animation->param[ANIMATION_PARAM_AIMING_DIR]  = dir;
		animation->param[ANIMATION_PARAM_AIMING_GAIT] = gait;
	} else {
		animation->param[ANIMATION_PARAM_AIMING]      = 0.0f;
		animation->param[ANIMATION_PARAM_AIMING_IDLE] = 0.0f;
	}

	if (charging_blend > 0.0f) {
		const Character3DAnimationNode *node = &def->node[def->charging_shoot_node];
		float dir = character3dAnimation_getLockedDirectionWeight(character, ANIMATION_PARAM_CHARGING_SHOOT_DIR);
		character3dAnimation_setGridRowSpeeds(animation, node);
		character3dAnimation_syncGridClips(animation, node, dir, gait);
		animation->param[ANIMATION_PARAM_CHARGING_SHOOT]      = weight * charging_blend;
		animation->param[ANIMATION_PARAM_CHARGING_SHOOT_IDLE] = (1.0f - weight) * charging_blend;
		animation->param[ANIMATION_PARAM_CHARGING_SHOOT_DIR]  = dir;
		animation->param[ANIMATION_PARAM_CHARGING_SHOOT_GAIT] = gait;
	} else {
		animation->param[ANIMATION_PARAM_CHARGING_SHOOT]      = 0.0f;
		animation->param[ANIMATION_PARAM_CHARGING_SHOOT_IDLE] = 0.0f;
	}

	/* No hand-fading the layers underneath: the stack already dilutes them,
	   and fading twice opens a hole the base idle bleeds through. */

	/* The locomotion underneath stays chained to the aiming cycle the whole
	   time, not just on the way out: it revives mid-fade when the mode
	   drops, and a phase matched every frame gives the crossfade two
	   identical cycles and the exit nothing to correct. */
	bool from_charging = charging_blend >= ready_blend;
	character3dAnimation_snapLocomotionFromGrid(animation,
		from_charging ? def->charging_shoot_node : def->aiming_node,
		animation->param[from_charging ? ANIMATION_PARAM_CHARGING_SHOOT_DIR : ANIMATION_PARAM_AIMING_DIR]);
}


/* --- per-frame driver ------------------------------------------------------- */

static void character3dAnimation_setActiveNodes(Character3DAnimation *animation)
{
	const Character3DAnimationDef *def = animation->def;

	for (int i = 0; i < def->node_count; i++) {
		const Character3DAnimationNode *node = &def->node[i];
		Character3DAnimationNodeType t = node->type;
		if (t == ANIMATION_NODE_SELECT || t == ANIMATION_NODE_SEQUENCE)
			animation->node_active[i] = (animation->param[node->param_cols] != 0.0f);
		if (t != ANIMATION_NODE_BLEND_2D) continue;

		bool was = animation->node_active[i];
		bool now = (animation->param[node->param_weight] != 0.0f);
		animation->node_active[i] = now;
		if (!now || was) continue;

		/* A grid coming back on has sat frozen while another one drove the
		   body: only the clips it shares with that grid kept advancing, the
		   rest hold the phase they stopped at. Its clips resume on the live
		   cycle, the heaviest corner of the previous frame (not advanced yet
		   this frame), so the two grids agree from the first frame of the
		   crossfade. One write per clip, on this frame only. */
		uint8_t clip[4];
		uint8_t count = character3dAnimation_getGridClips(node, animation->param[node->param_cols], animation->param[node->param_rows], clip);
		for (uint8_t m = 0; m < count; m++) {
			Animation *anim = character3dAnimation_clip(animation, clip[m]);
			animation_setTime(anim, animation->locomotion_cycle * animation_getLength(anim));
		}
	}
}

void character3dAnimation_setParams(Character3D *character, float delta)
{
	float gait = character3dAnimation_getGaitAxis(character, delta);

	character3dAnimation_setFooting (character);
	character3dAnimation_setLocomotionSpeed (character, gait);

	character3dAnimation_setLocomotionParam (character, gait);
	character3dAnimation_setIdleRightParam (&character->animation);
	character3dAnimation_setJumpParams (character, delta);
	character3dAnimation_setRollParam (character, delta);
	character3dAnimation_setStrafeParams (character, delta);
	character3dAnimation_setStrafeLockedParams (character, delta);
	character3dAnimation_setAimingParams (character, delta);
	character3dAnimation_setSwimParams (character, delta);
	character3dAnimation_setClimbParams (character, delta);
	character3dAnimation_setActiveNodes (&character->animation);
}


/* --- graph ------------------------------------------------------------------ */

/* a clip shared by two nodes must advance once per frame */
static void character3dAnimation_updateClip(Character3DAnimation *animation, bool *updated, uint8_t clip, float delta)
{
	if (updated[clip]) return;
	updated[clip] = true;
	animation_update(character3dAnimation_clip(animation, clip), delta);
}

void character3dAnimation_evaluateGraph(const Character3DAnimationDef *def, Character3DAnimation *animation, float delta)
{
	Character3DAnimationBuffer blend_buffer;
	blend_buffer.count = 0;

	bool updated[def->clip_count];
	memset(updated, false, sizeof(updated));

	/* the heaviest grid corner of the frame; its cycle is read once, after
	   the walk, instead of dividing every time the lead changes hands */
	float   cycle_weight = 0.0f;
	uint8_t cycle_clip   = 0;

	for (int i = 0; i < def->node_count; i++)
	{
		if (!animation->node_active[i]) continue;

		const Character3DAnimationNode *node = &def->node[i];
		float param_val = animation->param[node->param_cols];

		switch (node->type)
		{
			case ANIMATION_NODE_CLIP:
			{
				character3dAnimation_updateClip(animation, updated, node->animation[0], delta);
				break;
			}

			case ANIMATION_NODE_SELECT:
			{
				uint8_t active = (param_val < 0.0f) ? node->animation[0] : node->animation[1];
				uint8_t inactive = (param_val < 0.0f) ? node->animation[1] : node->animation[0];

				if (animation->node_state[i] != active)
				{
					animation->node_state[i] = active;
					animation_setTime(character3dAnimation_clip(animation, inactive),
					                  character3dAnimation_clip(animation, active)->time);
				}

				character3dAnimation_updateClip(animation, updated, active, delta);
				break;
			}

			case ANIMATION_NODE_SEQUENCE:
			{
				Animation *clip = character3dAnimation_clip(animation, node->animation[0]);
				if (clip->isPlaying)
				{
					float limit = animation_getLength(clip);
					if ((clip->time + delta) < limit)
						character3dAnimation_updateClip(animation, updated, node->animation[0], delta);
					else
						character3dAnimation_updateClip(animation, updated, node->animation[1], delta);
				}
				else
					character3dAnimation_updateClip(animation, updated, node->animation[1], delta);

				break;
			}

			case ANIMATION_NODE_BLEND:
			{
				if (param_val > 0.0f) {
					Armature *buf = (node->buffer == ANIMATION_SLOT_MAIN) ? &animation->main : &animation->buffer[node->buffer];
					character3dAnimation_updateClip(animation, updated, node->animation[0], delta);
					character3dAnimation_addLayer(&blend_buffer, buf, param_val);
				}

				break;
			}

			case ANIMATION_NODE_BLEND_2D:
			{
				float weight = animation->param[node->param_weight];
				if (weight <= 0.0f) break;

				float tx, ty;
				uint8_t col = character3dAnimation_blendSegment(param_val, node->cols, &tx);
				uint8_t row = character3dAnimation_blendSegment(animation->param[node->param_rows], node->rows, &ty);

				/* bilinear share of each corner, adds up to 1 */
				uint8_t corner[4];
				float   share[4];
				uint8_t count = 0;

				corner[count] = node->animation[row * node->cols + col];
				share[count++] = (1.0f - tx) * (1.0f - ty);

				if (tx > 0.0f) {
					corner[count] = node->animation[row * node->cols + col + 1];
					share[count++] = tx * (1.0f - ty);
				}

				if (ty > 0.0f) {
					corner[count] = node->animation[(row + 1) * node->cols + col];
					share[count++] = (1.0f - tx) * ty;
				}

				if (tx > 0.0f && ty > 0.0f) {
					corner[count] = node->animation[(row + 1) * node->cols + col + 1];
					share[count++] = tx * ty;
				}

				/* every layer is diluted by the ones applied after it, so each
				   one is divided by what those leave: the main keeps 1 - weight */
				float layer[4];
				float remain = 1.0f;
				for (int m = count - 1; m >= 0; m--) {
					layer[m] = (remain > 0.0000001f) ? weight * share[m] / remain : 1.0f;
					if (layer[m] > 1.0f) layer[m] = 1.0f;
					remain *= 1.0f - layer[m];
				}

				for (uint8_t m = 0; m < count; m++) {
					character3dAnimation_updateClip(animation, updated, corner[m], delta);
					if (layer[m] > 0.0f)
						character3dAnimation_addLayer(&blend_buffer, character3dAnimation_clipBuffer(def, animation, corner[m]), layer[m]);

					if (weight * share[m] > cycle_weight) {
						cycle_weight = weight * share[m];
						cycle_clip   = corner[m];
					}
				}

				break;
			}

			case ANIMATION_NODE_LAYER:
			{
				float abs_val = fabsf(param_val);
				if (abs_val > 0.0f) {
					Armature *buf = (node->buffer == ANIMATION_SLOT_MAIN) ? &animation->main : &animation->buffer[node->buffer];
					character3dAnimation_addLayer(&blend_buffer, buf, abs_val);
				}

				break;
			}
		}
	}

	if (cycle_weight > 0.0f) {
		Animation *clip = character3dAnimation_clip(animation, cycle_clip);
		animation->locomotion_cycle = clip->time / animation_getLength(clip);
	}

	character3dAnimation_blendLayers(&animation->main, &blend_buffer);
}


/* --- lifecycle -------------------------------------------------------------- */

void character3dAnimation_initGraph(Character3D *character, const Character3DAnimationDef *def)
{
	Character3DAnimation *animation = &character->animation;
	const Model *model = character->entity->mesh->model;

	animation->main = armature_createBuffered(model, FB_COUNT);

	animation->buffer = malloc(def->buffer_count * sizeof(Armature));
	assert(animation->buffer);
	animation->clip = malloc(def->clip_count * sizeof(Animation));
	assert(animation->clip);
	animation->node_state = malloc(def->node_count * sizeof(uint8_t));
	assert(animation->node_state);
	animation->node_active = malloc(def->node_count * sizeof(bool));
	assert(animation->node_active);

	for (int i = 0; i < def->buffer_count; i++)
		animation->buffer[i] = armature_clone(&animation->main, false);

	memset(animation->node_state,   0,    def->node_count * sizeof(uint8_t));
	memset(animation->node_active,  true, def->node_count * sizeof(bool));
	memset(animation->turn_avg,  0,    sizeof(animation->turn_avg));
	animation->turn_avg_idx = 0;
	animation->strafe_turning = false;
	animation->strafe_blend = 0.0f;
	animation->strafe_locked_blend = 0.0f;
	animation->aiming_blend = 0.0f;
	animation->charging_shoot_blend = 0.0f;
	animation->climb_blend = 0.0f;
	animation->climb_dir   = 0.0f;

	/* Clips open on demand through character3dAnimation_clip: a zeroed slot
	   (animRef NULL) is a closed clip. */
	animation->model = model;
	animation->clip_cooldown = malloc(def->clip_count);
	assert(animation->clip_cooldown);
	memset(animation->clip_cooldown, 0, def->clip_count);
	memset(animation->clip, 0, def->clip_count * sizeof(Animation));
	animation->clip_data = calloc(def->clip_count, sizeof(void *));
	assert(animation->clip_data);
}

void character3d_setAnimation(Character3D *character)
{
	if (!character->animation.def) return;

	float delta = time_get()->delta;

	character3dAnimation_setParams(character, delta);
	character3dAnimation_evaluateGraph(character->animation.def, &character->animation, delta);
	character3dAnimation_closeIdleClips(&character->animation);
	skeletonModifiers_apply(&character->skeleton_modifiers, &character->animation.main);
	armature_update(&character->animation.main);
}
