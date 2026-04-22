#ifndef PHYRIB_COLLISION_H
#define PHYRIB_COLLISION_H

#include "phyrib/types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct RigidBody RigidBody;

// Collision detection
bool Collision_TestShapes(const Shape* a, float aX, float aY, float aAngle, const Shape* b, float bX, float bY, float bAngle, Manifold* manifold);
void Collision_GenerateContact(Manifold* manifold, RigidBody* a, RigidBody* b);
void Collision_Resolve(Manifold* manifold, float dt);

// Helper functions
Vector2 Collision_ComputeNormal(const Shape* shape, int edgeIndex);
Vector2 Collision_FindSupport(const Shape* shape, Vector2 direction);
float Collision_ComputePenetration(const Shape* a, float aAngle, const Shape* b, float bAngle, Vector2* normal);

// Broadphase
bool Collision_AABBOverlap(Vector2 minA, Vector2 maxA, Vector2 minB, Vector2 maxB);

#ifdef __cplusplus
}
#endif

#endif // PHYRIB_COLLISION_H
