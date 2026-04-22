#ifndef PHYRIB_TYPES_H
#define PHYRIB_TYPES_H

#include <stdbool.h>
#include <stdint.h>

// Import raylib types if available
#if defined(PHYRIB_USE_RAYLIB) && PHYRIB_USE_RAYLIB
    #include "raylib.h"
    #define PR_Vector2 Vector2
    #define PR_Color Color
#else
    // Fallback types when raylib is not available
    typedef struct { float x, y; } Vector2;
    typedef struct { float r, g, b, a; } Color;
    #define PR_Vector2 Vector2
    #define PR_Color Color
#endif

// Basic types
typedef uint32_t PhysicsBodyID;
typedef uint32_t ShapeID;

// Shape types
typedef enum {
    PR_SHAPE_CIRCLE = 0,
    PR_SHAPE_RECTANGLE = 1,
    PR_SHAPE_POLYGON = 2,
    PR_SHAPE_CAPSULE = 3,
    PR_SHAPE_EDGE = 4
} ShapeType;

// Material properties
typedef struct {
    float friction;        // 0.0 - 1.0 (default 0.3)
    float restitution;     // Bounciness 0.0 - 1.0 (default 0.2)
    float density;         // kg/m² (default 1.0)
} PhysicsMaterial;

// Default material
static const PhysicsMaterial PR_MATERIAL_DEFAULT = { 0.3f, 0.2f, 1.0f };
static const PhysicsMaterial PR_MATERIAL_ICE = { 0.1f, 0.1f, 1.0f };
static const PhysicsMaterial PR_MATERIAL_RUBBER = { 0.9f, 0.8f, 1.0f };
static const PhysicsMaterial PR_MATERIAL_BOUNCY = { 0.2f, 0.95f, 1.0f };
static const PhysicsMaterial PR_MATERIAL_HEAVY = { 0.6f, 0.3f, 5.0f };
static const PhysicsMaterial PR_MATERIAL_LIGHT = { 0.2f, 0.1f, 0.1f };

// Rigid body types
typedef enum {
    PR_BODY_STATIC = 0,    // Immovable, infinite mass
    PR_BODY_DYNAMIC = 1,   // Fully simulated
    PR_BODY_KINEMATIC = 2  // User-controlled movement, no forces
} BodyType;

// Collision filtering
typedef struct {
    uint16_t categoryBits;  // What category this body belongs to
    uint16_t maskBits;      // What categories this body collides with
    uint16_t groupIndex;    // Collision group (positive = always collide, negative = never)
} CollisionFilter;

// Shape data for each shape type
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

// Rigid body definition (for creation)
typedef struct {
    BodyType type;                  // Body type
    Vector2 position;               // Initial position
    float angle;                    // Initial angle (radians)
    Vector2 linearVelocity;         // Initial velocity
    float angularVelocity;          // Initial angular velocity
    float linearDamping;            // Air resistance (0-1)
    float angularDamping;           // Rotational damping (0-1)
    bool fixedRotation;             // Prevent rotation
    bool isSensor;                  // Trigger only, no collision response
    PhysicsMaterial material;       // Physical properties
    CollisionFilter filter;         // Collision filtering
    Shape shape;                    // Collision shape
    void* userData;                 // User data pointer
} RigidBodyDef;

// Rigid body (runtime)
typedef struct RigidBody RigidBody;

// Collision manifold (contact information)
typedef struct {
    RigidBody* bodyA;
    RigidBody* bodyB;
    Vector2 contacts[2];            // Contact points
    Vector2 normals[2];             // Contact normals
    int contactCount;               // Number of contacts (1 or 2)
    float penetration;              // Overlap depth
    PhysicsMaterial material;      // Mixed material (for response)
} Manifold;

// Collision event callback
typedef void (*PhysicsCollisionCallback)(RigidBody* a, RigidBody* b, Manifold* m, void* userData);

// Physics world configuration
typedef struct {
    float gravityX;               // Gravity X (default 0)
    float gravityY;               // Gravity Y (default 980.0)
    int velocityIterations;       // Velocity constraint solver iterations (default 8)
    int positionIterations;       // Position constraint solver iterations (default 3)
    float fixedDeltaTime;         // Fixed timestep (0 = variable)
    bool allowSleep;              // Allow bodies to sleep when idle
    float sleepTimeThreshold;     // Time before sleeping (seconds)
    float sleepVelocityThreshold; // Velocity threshold for sleeping
} WorldConfig;

// Default world config
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

// Physics world
typedef struct PhysicsWorld PhysicsWorld;

// Debug draw flags
typedef enum {
    PR_DEBUG_SHAPES = 1 << 0,      // Draw shapes (wireframe)
    PR_DEBUG_AABB = 1 << 1,        // Draw bounding boxes
    PR_DEBUG_CONTACTS = 1 << 2,    // Draw contact points
    PR_DEBUG_VELOCITY = 1 << 3,    // Draw velocity vectors
    PR_DEBUG_IDS = 1 << 4,         // Draw body IDs
    PR_DEBUG_COLLISION_NORMAL = 1 << 5  // Draw collision normals
} DebugDrawFlags;

#endif // PHYRIB_TYPES_H
