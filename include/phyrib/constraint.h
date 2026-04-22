#ifndef PHYRIB_CONSTRAINT_H
#define PHYRIB_CONSTRAINT_H

#include "phyrib/types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Constraint Constraint;

typedef enum {
    PR_CONSTRAINT_DISTANCE = 0,
    PR_CONSTRAINT_PIN = 1,
    PR_CONSTRAINT_SPRING = 2,
    PR_CONSTRAINT_WELD = 3,
    PR_CONSTRAINT_MOTOR = 4
} ConstraintType;

typedef struct {
    ConstraintType type;
    RigidBody* bodyA;
    RigidBody* bodyB;
    Vector2 localAnchorA;
    Vector2 localAnchorB;
    union {
        struct { float distance; } distance;
        struct { float stiffness, damping, restLength; } spring;
        struct { float targetSpeed, maxForce; } motor;
    } data;
} ConstraintDef;

Constraint* Constraint_Create(const ConstraintDef* def);
void Constraint_Destroy(Constraint* constraint);
void Constraint_SetBodies(Constraint* constraint, RigidBody* a, RigidBody* b);
void Constraint_GetBodies(Constraint* constraint, RigidBody** a, RigidBody** b);

ConstraintType Constraint_GetType(Constraint* constraint);
void Constraint_SetDistance(Constraint* constraint, float distance);
void Constraint_SetAnchorA(Constraint* constraint, Vector2 local);
void Constraint_SetAnchorB(Constraint* constraint, Vector2 local);

void Constraint_SetSpringParams(Constraint* constraint, float stiffness, float damping, float restLength);

void Constraint_SetMotorParams(Constraint* constraint, float targetSpeed, float maxForce);
void Constraint_SetMotorEnabled(Constraint* constraint, bool enabled);
bool Constraint_IsMotorEnabled(Constraint* constraint);
float Constraint_GetMotorSpeed(Constraint* constraint);

void Constraint_PreSolve(Constraint* constraint, float dt);
void Constraint_Solve(Constraint* constraint, float dt);
void Constraint_PostSolve(Constraint* constraint);

void Constraint_DrawDebug(Constraint* constraint, Color color);

#ifdef __cplusplus
}
#endif

#endif // PHYRIB_CONSTRAINT_H
