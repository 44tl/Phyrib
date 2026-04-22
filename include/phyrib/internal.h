#ifndef PHYRIB_INTERNAL_H
#define PHYRIB_INTERNAL_H

#include "phyrib/phyrib.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef M_TAU
#define M_TAU 6.28318530717958647692f
#endif

// Inline math helpers
static inline float PR_Clamp(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline float PR_Min(float a, float b) { return a < b ? a : b; }
static inline float PR_Max(float a, float b) { return a > b ? a : b; }

static inline float PR_Sqr(float x) { return x * x; }

static inline void PR_Vec2_Add(Vector2* out, const Vector2 a, const Vector2 b) {
    out->x = a.x + b.x; out->y = a.y + b.y;
}

static inline void PR_Vec2_Sub(Vector2* out, const Vector2 a, const Vector2 b) {
    out->x = a.x - b.x; out->y = a.y - b.y;
}

static inline void PR_Vec2_Scale(Vector2* out, const Vector2 v, float s) {
    out->x = v.x * s; out->y = v.y * s;
}

static inline float PR_Vec2_Dot(const Vector2 a, const Vector2 b) {
    return a.x * b.x + a.y * b.y;
}

static inline float PR_Vec2_Cross(const Vector2 a, const Vector2 b) {
    return a.x * b.y - a.y * b.x;
}

static inline float PR_Vec2_LengthSqr(const Vector2 v) {
    return v.x * v.x + v.y * v.y;
}

static inline float PR_Vec2_Length(const Vector2 v) {
    return sqrtf(v.x * v.x + v.y * v.y);
}

static inline void PR_Vec2_Normalize(Vector2* v) {
    float len = PR_Vec2_Length(*v);
    if (len > 0.00001f) {
        v->x /= len; v->y /= len;
    } else {
        v->x = 0; v->y = 0;
    }
}

static inline void PR_Vec2_Perp(Vector2* out, const Vector2 v) {
    out->x = -v.y; out->y = v.x;
}

static inline void PR_Vec2_Rotate(Vector2* out, const Vector2 v, float rad) {
    float c = cosf(rad), s = sinf(rad);
    out->x = v.x * c - v.y * s;
    out->y = v.x * s + v.y * c;
}

static inline void PR_Vec2_Set(Vector2* v, float x, float y) {
    v->x = x; v->y = y;
}

static inline void PR_Mat2_Init(float* m, float angle) {
    float c = cosf(angle), s = sinf(angle);
    m[0] = c; m[1] = -s;
    m[2] = s; m[3] = c;
}

// Hash table for spatial hashing
#define PHYRIB_HASH_SIZE 1024
#define PHYRIB_HASH(KEY) (((KEY) * 2654435761u) % PHYRIB_HASH_SIZE)

// Linked list node for bodies
typedef struct BodyNode {
    RigidBody* body;
    struct BodyNode* next;
} BodyNode;

// Array for dynamic body storage
typedef struct {
    RigidBody** bodies;
    int count;
    int capacity;
} BodyArray;

// Drawn color configuration
typedef struct {
    Color shape;
    Color aabb;
    Color contact;
    Color static_;
    Color dynamic;
    Color kinematic;
    Color sleeping;
    float lineThickness;
    bool fillShapes;
} DebugRenderState;

// Global debug state
extern DebugRenderState gDebugState;

// Random ID generator
static inline PhysicsBodyID GenerateBodyID() {
    static PhysicsBodyID nextID = 1;
    return nextID++;
}

// Body flags
#define BODY_FLAG_ISLAND     0x01  // Part of an island
#define BODY_FLAG_AWAKE      0x02  // Body is awake
#define BODY_FLAG_SLEEPING   0x04  // Body is sleeping
#define BODY_FLAG_LOCK_AXIS  0x08  // Lock rotation axis

// Helpers
static inline bool Body_IsStatic(const RigidBody* body) {
    return body->type == PR_BODY_STATIC;
}

static inline bool Body_IsDynamic(const RigidBody* body) {
    return body->type == PR_BODY_DYNAMIC;
}

static inline bool Body_IsKinematic(const RigidBody* body) {
    return body->type == PR_BODY_KINEMATIC;
}

static inline bool Body_IsSleeping(const RigidBody* body) {
    return (body->flags & BODY_FLAG_SLEEPING) != 0;
}

static inline void Body_SetAwake(RigidBody* body, bool awake) {
    if (awake) {
        body->flags |= BODY_FLAG_AWAKE;
        body->flags &= ~BODY_FLAG_SLEEPING;
        body->sleepTime = 0.0f;
    } else {
        body->flags &= ~BODY_FLAG_AWAKE;
    }
}

static inline float Body_GetVelocity(const RigidBody* body) {
    return PR_Vec2_Length(body->linearVelocity);
}

static inline float Body_ComputeInertiaForBox(float mass, float width, float height) {
    return (1.0f / 12.0f) * mass * (width * width + height * height);
}

static inline float Body_ComputeInertiaForCircle(float mass, float radius) {
    return (1.0f / 2.0f) * mass * radius * radius;
}

static inline float Body_ComputeInertiaForCapsule(float mass, float radius, float height) {
    float cylInert = (1.0f / 12.0f) * mass * (3.0f * radius * radius + height * height);
    float sphInert = (2.0f / 5.0f) * mass * radius * radius;
    return cylInert + sphInert;
}

// Memory allocation helpers
static inline void* PR_Alloc(size_t size) {
    return malloc(size);
}

static inline void PR_Free(void* ptr) {
    free(ptr);
}

static inline void* PR_AllocZeroed(size_t size) {
    void* p = calloc(1, size);
    return p;
}

static inline void* PR_Realloc(void* ptr, size_t newSize) {
    return realloc(ptr, newSize);
}

#endif // PHYRIB_INTERNAL_H
