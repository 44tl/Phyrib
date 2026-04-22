#ifndef PHYRIB_PHYSICS_WORLD_H
#define PHYRIB_PHYSICS_WORLD_H

#include <stdbool.h>
#include "phyrib/types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct PhysicsWorld PhysicsWorld;

PhysicsWorld* PhysicsWorld_Create(const WorldConfig* config);
PhysicsWorld* PhysicsWorld_CreateDefault();
void PhysicsWorld_Destroy(PhysicsWorld* world);

void PhysicsWorld_SetGravity(PhysicsWorld* world, float x, float y);
void PhysicsWorld_GetGravity(PhysicsWorld* world, float* x, float* y);
void PhysicsWorld_SetIterations(PhysicsWorld* world, int velocity, int position);
void PhysicsWorld_GetIterations(PhysicsWorld* world, int* velocity, int* position);

void PhysicsWorld_Step(PhysicsWorld* world, float deltaTime);
void PhysicsWorld_StepFixed(PhysicsWorld* world, float accumulator);
void PhysicsWorld_ClearForces(PhysicsWorld* world);

RigidBody* PhysicsWorld_CreateBody(PhysicsWorld* world, const RigidBodyDef* def);
void PhysicsWorld_DestroyBody(PhysicsWorld* world, RigidBody* body);
void PhysicsWorld_AddBody(PhysicsWorld* world, RigidBody* body);
void PhysicsWorld_RemoveBody(PhysicsWorld* world, RigidBody* body);

RigidBody** PhysicsWorld_QueryPoint(PhysicsWorld* world, Vector2 point, int* count);
RigidBody** PhysicsWorld_QueryAABB(PhysicsWorld* world, Vector2 min, Vector2 max, int* count);
RigidBody** PhysicsWorld_QueryRay(PhysicsWorld* world, Vector2 origin, Vector2 direction, float maxDist, int* count);

typedef bool (*PhysicsWorld_RaycastCallback)(RigidBody* body, Vector2 point, Vector2 normal, float fraction, void* userData);
bool PhysicsWorld_Raycast(PhysicsWorld* world, Vector2 origin, Vector2 direction, float maxDist, PhysicsWorld_RaycastCallback callback, void* userData);

void PhysicsWorld_SetCollisionCallback(PhysicsWorld* world, PhysicsCollisionCallback callback, void* userData);
void PhysicsWorld_ClearCollisionCallback(PhysicsWorld* world);

void PhysicsWorld_SetSleeping(PhysicsWorld* world, bool enabled);
void PhysicsWorld_SetSleepThresholds(PhysicsWorld* world, float timeThresh, float velocityThresh);
void PhysicsWorld_WakeUp(PhysicsWorld* world, RigidBody* body);
void PhysicsWorld_WakeUpAll(PhysicsWorld* world);

void PhysicsWorld_DrawDebug(PhysicsWorld* world, int flags);
void PhysicsWorld_SetDebugColor(PhysicsWorld* world, Color shapeColor, Color aabbColor, Color contactColor);

int PhysicsWorld_GetBodyCount(PhysicsWorld* world);
int PhysicsWorld_GetContactCount(PhysicsWorld* world);
float PhysicsWorld_GetKineticEnergy(PhysicsWorld* world);

bool PhysicsWorld_IsPointInside(PhysicsWorld* world, RigidBody* body, Vector2 point);

#ifdef __cplusplus
}
#endif

#endif // PHYRIB_PHYSICS_WORLD_H
