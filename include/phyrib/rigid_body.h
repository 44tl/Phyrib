#ifndef PHYRIB_RIGID_BODY_H
#define PHYRIB_RIGID_BODY_H

#include <stdbool.h>
#include "phyrib/types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Forward declaration
typedef struct RigidBody RigidBody;

// Body info
const char* RigidBody_GetName(RigidBody* body);
PhysicsBodyID RigidBody_GetID(RigidBody* body);
BodyType RigidBody_GetType(RigidBody* body);
void RigidBody_SetType(RigidBody* body, BodyType type);

// Transform
Vector2 RigidBody_GetPosition(RigidBody* body);
void RigidBody_SetPosition(RigidBody* body, Vector2 pos);
float RigidBody_GetAngle(RigidBody* body);
void RigidBody_SetAngle(RigidBody* body, float angle);
void RigidBody_GetTransform(RigidBody* body, Vector2* pos, float* angle);
void RigidBody_SetTransform(RigidBody* body, Vector2 pos, float angle);

// Velocity
Vector2 RigidBody_GetLinearVelocity(RigidBody* body);
void RigidBody_SetLinearVelocity(RigidBody* body, Vector2 vel);
float RigidBody_GetAngularVelocity(RigidBody* body);
void RigidBody_SetAngularVelocity(RigidBody* body, float vel);
void RigidBody_ApplyLinearImpulse(RigidBody* body, Vector2 impulse, Vector2 point);
void RigidBody_ApplyAngularImpulse(RigidBody* body, float impulse);
void RigidBody_ApplyForce(RigidBody* body, Vector2 force, Vector2 point);
void RigidBody_ApplyTorque(RigidBody* body, float torque);
void RigidBody_ApplyForceToCenter(RigidBody* body, Vector2 force);
Vector2 RigidBody_GetForce(RigidBody* body);
float RigidBody_GetTorque(RigidBody* body);
void RigidBody_ClearForces(RigidBody* body);

// Mass properties
float RigidBody_GetMass(RigidBody* body);
void RigidBody_SetMass(RigidBody* body, float mass);
float RigidBody_GetInvMass(RigidBody* body);
float RigidBody_GetInertia(RigidBody* body);
void RigidBody_SetInertia(RigidBody* body, inertia);
float RigidBody_GetInvInertia(RigidBody* body);
void RigidBody_SetFixedRotation(RigidBody* body, bool fixed);
bool RigidBody_IsFixedRotation(RigidBody* body);

// Damping
float RigidBody_GetLinearDamping(RigidBody* body);
void RigidBody_SetLinearDamping(RigidBody* body, damping);
float RigidBody_GetAngularDamping(RigidBody* body);
void RigidBody_SetAngularDamping(RigidBody* body, damping);

// Material & collision filter
PhysicsMaterial RigidBody_GetMaterial(RigidBody* body);
void RigidBody_SetMaterial(RigidBody* body, PhysicsMaterial mat);
CollisionFilter RigidBody_GetCollisionFilter(RigidBody* body);
void RigidBody_SetCollisionFilter(RigidBody* body, CollisionFilter filter);

// Sensor mode
bool RigidBody_IsSensor(RigidBody* body);
void RigidBody_SetSensor(RigidBody* body, bool sensor);

// Sleep state
bool RigidBody_IsAwake(RigidBody* body);
void RigidBody_SetAwake(RigidBody* body, awake);
void RigidBody_Sleep(RigidBody* body);
void RigidBody_WakeUp(RigidBody* body);

// Shape
Shape* RigidBody_GetShape(RigidBody* body);
void RigidBody_SetShape(RigidBody* body, const Shape* shape);

// User data
void* RigidBody_GetUserData(RigidBody* body);
void RigidBody_SetUserData(RigidBody* body, void* data);

// Kinematic control
void RigidBody_MoveTo(RigidBody* body, Vector2 target, float dt);
void RigidBody_MoveBy(RigidBody* body, Vector2 delta, float dt);

// Velocity control (for kinematic bodies)
void RigidBody_SetKinematicVelocity(RigidBody* body, Vector2 vel);

// Geometry
Vector2 RigidBody_GetWorldPoint(RigidBody* body, Vector2 localPoint);
Vector2 RigidBody_GetLocalPoint(RigidBody* body, Vector2 worldPoint);
Vector2 RigidBody_GetWorldVector(RigidBody* body, Vector2 localVec);
Vector2 RigidBody_GetLocalVector(RigidBody* body, Vector2 worldVec);
float RigidBody_GetLinearVelocityAtPoint(RigidBody* body, Vector2 worldPoint);

// Properties
float RigidBody_GetKineticEnergy(RigidBody* body);
float RigidBody_GetPotentialEnergy(RigidBody* body, float gravityMag);

// AABB
void RigidBody_GetAABB(RigidBody* body, Vector2* min, Vector2* max);

// Rotation
void RigidBody_Rotate(RigidBody* body, float angle);
void RigidBody_RotateAround(RigidBody* body, Vector2 point, float angle);

#ifdef __cplusplus
}
#endif

#endif // PHYRIB_RIGID_BODY_H
