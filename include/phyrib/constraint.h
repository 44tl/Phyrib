#ifndef PHYRIB_CONSTRAINT_H
#define PHYRIB_CONSTRAINT_H

#include "phyrib/types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Forward declaration
typedef struct Constraint Constraint;

// Constraint types
typedef enum {
    PR_CONSTRAINT_DISTANCE = 0,
    PR_CONSTRAINT_PIN = 1,
    PR_CONSTRAINT_SPRING = 2,
    PR_CONSTRAINT_WELD = 3,
    PR_CONSTRAINT_MOTOR = 4
} ConstraintType;

// Constraint definition
typedef struct {
    ConstraintType type;
    RigidBody* bodyA;
    RigidBody* bodyB;
    Vector2 localAnchorA;  // Local anchor on body A
    Vector2 localAnchorB;  // Local anchor on body B
    union {
        struct { float distance; } distance;
        struct { float stiffness, damping, restLength; } spring;
        struct { float targetSpeed, maxForce; } motor;
    } data;
} ConstraintDef;

// Create/destroy
Constraint* Constraint_Create(const ConstraintDef* def);
void Constraint_Destroy(Constraint* constraint);
void Constraint_SetBodies(Constraint* constraint, RigidBody* a, RigidBody* b);
void Constraint_GetBodies(Constraint* constraint, RigidBody** a, RigidBody** b);

// Configuration
ConstraintType Constraint_GetType(Constraint* constraint);
void Constraint_SetDistance(Constraint* constraint, float distance);
void Constraint_SetAnchorA(Constraint* constraint, Vector2 local);
void Constraint_SetAnchorB(Constraint* constraint, Vector2 local);

// Spring-specific
void Constraint_SetSpringParams(Constraint* constraint, float stiffness, float damping, float restLength);

// Motor-specific
void Constraint_SetMotorParams(Constraint* constraint, float targetSpeed, float maxForce);
void Constraint_SetMotorEnabled(Constraint* constraint, bool enabled);
bool Constraint_IsMotorEnabled(Constraint* constraint);
float Constraint_GetMotorSpeed(Constraint* constraint);

// Solver
void Constraint_PreSolve(Constraint* constraint, float dt);
void Constraint_Solve(Constraint* constraint, float dt);
void Constraint_PostSolve(Constraint* constraint);

// Debug
void Constraint_DrawDebug(Constraint* constraint, Color color);

#ifdef __cplusplus
}
#endif

#endif // PHYRIB_CONSTRAINT_H
