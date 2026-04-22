#ifndef PHYRIB_H
#define PHYRIB_H

#include "phyrib/types.h"
#include "phyrib/physics_world.h"
#include "phyrib/rigid_body.h"
#include "phyrib/shape.h"
#include "phyrib/collision.h"
#include "phyrib/debug_render.h"
#include "phyrib/constraint.h"

// Library version
#define PHYRIB_VERSION "1.0.0"
#define PHYRIB_VERSION_MAJOR 1
#define PHYRIB_VERSION_MINOR 0
#define PHYRIB_VERSION_PATCH 0

// Convenience macros
#define PR_MIN(a, b) ((a) < (b) ? (a) : (b))
#define PR_MAX(a, b) ((a) > (b) ? (a) : (b))
#define PR_CLAMP(v, lo, hi) PR_MIN(PR_MAX(v, lo), hi)
#define PR_ABS(f) ((f) < 0 ? -(f) : (f))
#define PR_SIGN(f) ((f) < 0 ? -1 : (f) > 0 ? 1 : 0)
#define PR_SQ(x) ((x) * (x))

// Radians/Degrees conversion
#define PR_DEG2RAD(deg) ((deg) * 0.01745329251994329576923690768489f) // PI / 180
#define PR_RAD2DEG(rad) ((rad) * 57.295779513082320876798154814105f) // 180 / PI

#endif // PHYRIB_H
