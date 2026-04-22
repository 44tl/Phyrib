#ifndef PHYRIB_TYPES_H
#define PHYRIB_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#if defined(PHYRIB_USE_RAYLIB) && PHYRIB_USE_RAYLIB
    #include "raylib.h"
    #define PR_Vector2 Vector2
    #define PR_Color Color
#else
    typedef struct { float x, y; } Vector2;
    typedef struct { float r, g, b, a; } Color;
    #define PR_Vector2 Vector2
    #define PR_Color Color
#endif

typedef uint32_t PhysicsBodyID;
typedef uint32_t ShapeID;

typedef enum {
    PR_SHAPE_CIRCLE = 0,
    PR_SHAPE_RECTANGLE = 1,
    PR_SHAPE_POLYGON = 2,
    PR_SHAPE_CAPSULE = 3,
    PR_SHAPE_EDGE = 4
} ShapeType;

typedef struct {
    float friction;
    float restitution;
    float density;
} PhysicsMaterial;

static const PhysicsMaterial PR_MATERIAL_DEFAULT = { 0.3f, 0.2f, 1.0f };
static const PhysicsMaterial PR_MATERIAL_ICE = { 0.1f, 0.1f, 1.0f };
static const PhysicsMaterial PR_MATERIAL_RUBBER = { 0.9f, 0.8f, 1.0f };
static const PhysicsMaterial PR_MATERIAL_BOUNCY = { 0.2f, 0.95f, 1.0f };
static const PhysicsMaterial PR_MATERIAL_HEAVY = { 0.6f, 0.3f, 5.0f };
static const PhysicsMaterial PR_MATERIAL_LIGHT = { 0.2f, 0.1f, 0.1f };

typedef enum {
    PR_BODY_STATIC = 0,
    PR_BODY_DYNAMIC = 1,
    PR_BODY_KINEMATIC = 2
} BodyType;

typedef struct {
    uint16_t categoryBits;
    uint16_t maskBits;
    uint16_t groupIndex;
} CollisionFilter;

typedef struct {
    ShapeType type;
    union {
        struct { float radius; } circle;
        struct { float width, height; } rectangle;
        struct { Vector2* vertices; int count; } polygon;
        struct { float radius, height; } capsule;
        struct { Vector2 start, end; } edge;
    } data;
} Shape;

typedef struct {
    BodyType type;
    Vector2 position;
    float angle;
    Vector2 linearVelocity;
    float angularVelocity;
    float linearDamping;
    float angularDamping;
    bool fixedRotation;
    bool isSensor;
    PhysicsMaterial material;
    CollisionFilter filter;
    Shape shape;
    void* userData;
} RigidBodyDef;

typedef struct RigidBody RigidBody;

typedef struct {
    RigidBody* bodyA;
    RigidBody* bodyB;
    Vector2 contacts[2];
    Vector2 normals[2];
    int contactCount;
    float penetration;
    PhysicsMaterial material;
} Manifold;

typedef void (*PhysicsCollisionCallback)(RigidBody* a, RigidBody* b, Manifold* m, void* userData);

typedef struct {
    float gravityX;
    float gravityY;
    int velocityIterations;
    int positionIterations;
    float fixedDeltaTime;
    bool allowSleep;
    float sleepTimeThreshold;
    float sleepVelocityThreshold;
} WorldConfig;

static const WorldConfig PR_WORLD_DEFAULT = {
    .gravityX = 0.0f,
    .gravityY = 980.0f,
    .velocityIterations = 8,
    .positionIterations = 3,
    .fixedDeltaTime = 0.0f,
    .allowSleep = true,
    .sleepTimeThreshold = 0.5f,
    .sleepVelocityThreshold = 10.0f
};

typedef struct PhysicsWorld PhysicsWorld;

typedef enum {
    PR_DEBUG_SHAPES = 1 << 0,
    PR_DEBUG_AABB = 1 << 1,
    PR_DEBUG_CONTACTS = 1 << 2,
    PR_DEBUG_VELOCITY = 1 << 3,
    PR_DEBUG_IDS = 1 << 4,
    PR_DEBUG_COLLISION_NORMAL = 1 << 5
} DebugDrawFlags;

#endif // PHYRIB_TYPES_H
