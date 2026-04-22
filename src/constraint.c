#include "phyrib/internal.h"
#include "phyrib/constraint.h"
#include "phyrib/rigid_body.h"

struct Constraint {
    ConstraintType type;
    RigidBody* bodyA;
    RigidBody* bodyB;
    Vector2 localAnchorA;
    Vector2 localAnchorB;

    Vector2 worldAnchorA;
    Vector2 worldAnchorB;
    Vector2 rA;
    Vector2 rB;

    Vector2 impulse;
    float normalImpulse;
    float tangentImpulse;

    union {
        struct { float restLength; } distance;
        struct { float stiffness; float damping; float restLength; } spring;
        struct { float targetSpeed; float maxForce; bool enabled; } motor;
    } params;

    float effectiveMass;
    Vector2 massNormal;
};

Constraint* Constraint_Create(const ConstraintDef* def) {
    if (!def) return NULL;

    Constraint* c = (Constraint*)PR_AllocZeroed(sizeof(Constraint));
    if (!c) return NULL;

    c->type = def->type;
    c->bodyA = def->bodyA;
    c->bodyB = def->bodyB;
    c->localAnchorA = def->localAnchorA;
    c->localAnchorB = def->localAnchorB;

    switch (c->type) {
        case PR_CONSTRAINT_DISTANCE:
            c->params.distance.restLength = def->data.distance.distance;
            break;
        case PR_CONSTRAINT_SPRING:
            c->params.spring.stiffness = def->data.spring.stiffness;
            c->params.spring.damping = def->data.spring.damping;
            c->params.spring.restLength = def->data.spring.restLength;
            break;
        default:
            break;
    }

    c->impulse = (Vector2){0};
    c->normalImpulse = 0.0f;
    c->tangentImpulse = 0.0f;

    return c;
}

void Constraint_Destroy(Constraint* c) {
    if (c) PR_Free(c);
}

void Constraint_SetBodies(Constraint* c, RigidBody* a, RigidBody* b) {
    if (!c) return;
    c->bodyA = a;
    c->bodyB = b;
}

void Constraint_GetBodies(Constraint* c, RigidBody** a, RigidBody** b) {
    if (!c) return;
    if (a) *a = c->bodyA;
    if (b) *b = c->bodyB;
}

ConstraintType Constraint_GetType(Constraint* c) {
    return c ? c->type : PR_CONSTRAINT_DISTANCE;
}

void Constraint_SetDistance(Constraint* c, float distance) {
    if (!c || c->type != PR_CONSTRAINT_DISTANCE) return;
    c->params.distance.restLength = distance;
}

void Constraint_SetAnchorA(Constraint* c, Vector2 local) {
    if (c) c->localAnchorA = local;
}

void Constraint_SetAnchorB(Constraint* c, Vector2 local) {
    if (c) c->localAnchorB = local;
}

void Constraint_SetSpringParams(Constraint* c, float stiffness, float damping, float restLength) {
    if (!c || c->type != PR_CONSTRAINT_SPRING) return;
    c->params.spring.stiffness = stiffness;
    c->params.spring.damping = damping;
    c->params.spring.restLength = restLength;
}

void Constraint_SetMotorParams(Constraint* c, float targetSpeed, float maxForce) {
    if (!c || c->type != PR_CONSTRAINT_MOTOR) return;
    c->params.motor.targetSpeed = targetSpeed;
    c->params.motor.maxForce = maxForce;
}
void Constraint_SetMotorEnabled(Constraint* c, bool enabled) {
    if (!c || c->type != PR_CONSTRAINT_MOTOR) return;
    c->params.motor.enabled = enabled;
}
bool Constraint_IsMotorEnabled(Constraint* c) {
    return c && c->type == PR_CONSTRAINT_MOTOR && c->params.motor.enabled;
}
float Constraint_GetMotorSpeed(Constraint* c) {
    return c && c->type == PR_CONSTRAINT_MOTOR ? c->params.motor.targetSpeed : 0.0f;
}

static void Pin_PreSolve(Constraint* c, float dt) {
    if (!c->bodyA && !c->bodyB) return;

    if (c->bodyA) {
        c->worldAnchorA = RigidBody_GetWorldPoint(c->bodyA, c->localAnchorA);
        c->rA = RigidBody_GetWorldVector(c->bodyA, c->localAnchorA);
    } else {
        c->worldAnchorA = c->localAnchorA;
        c->rA = (Vector2){0,0};
    }

    if (c->bodyB) {
        c->worldAnchorB = RigidBody_GetWorldPoint(c->bodyB, c->localAnchorB);
        c->rB = RigidBody_GetWorldVector(c->bodyB, c->localAnchorB);
    } else {
        c->worldAnchorB = c->localAnchorB;
        c->rB = (Vector2){0,0};
    }
}

static void Pin_Solve(Constraint* c, float dt) {
    if (!c->bodyA && !c->bodyB) return;

    RigidBody* bodyA = c->bodyA;
    RigidBody* bodyB = c->bodyB;

    bool hasA = (bodyA != NULL);
    bool hasB = (bodyB != NULL);
    bool aDynamic = hasA && Body_IsDynamic(bodyA);
    bool bDynamic = hasB && Body_IsDynamic(bodyB);

    float invMassA = aDynamic ? bodyA->invMass : 0.0f;
    float invMassB = bDynamic ? bodyB->invMass : 0.0f;
    float invInertiaA = aDynamic ? bodyA->invInertia : 0.0f;
    float invInertiaB = bDynamic ? bodyB->invInertia : 0.0f;

    Vector2 velA = {0,0}, velB = {0,0};
    if (hasA) {
        velA = (Vector2){
            bodyA->linearVelocity.x + (-c->rA.y * bodyA->angularVelocity),
            bodyA->linearVelocity.y + ( c->rA.x * bodyA->angularVelocity)
        };
    }
    if (hasB) {
        velB = (Vector2){
            bodyB->linearVelocity.x + (-c->rB.y * bodyB->angularVelocity),
            bodyB->linearVelocity.y + ( c->rB.x * bodyB->angularVelocity)
        };
    }

    Vector2 vRel = { velB.x - velA.x, velB.y - velA.y };

    Vector2 error = {
        c->worldAnchorB.x - c->worldAnchorA.x,
        c->worldAnchorB.y - c->worldAnchorA.y
    };

    const float baumgarte = 0.2f;
    Vector2 bias = {
        error.x * baumgarte / dt,
        error.y * baumgarte / dt
    };

    float effMassX = 1.0f / (invMassA + invMassB + (c->rA.y * c->rA.y) * invInertiaA + (c->rB.y * c->rB.y) * invInertiaB);
    float effMassY = 1.0f / (invMassA + invMassB + (c->rA.x * c->rA.x) * invInertiaA + (c->rB.x * c->rB.x) * invInertiaB);

    Vector2 impulse = {
        -(vRel.x + bias.x) * effMassX + c->impulse.x,
        -(vRel.y + bias.y) * effMassY + c->impulse.y
    };

    float maxImpulse = 1.0f;
    impulse.x = PR_CLAMP(impulse.x, -maxImpulse, maxImpulse);
    impulse.y = PR_CLAMP(impulse.y, -maxImpulse, maxImpulse);

    Vector2 delta = { impulse.x - c->impulse.x, impulse.y - c->impulse.y };
    c->impulse = impulse;

    if (aDynamic) {
        bodyA->linearVelocity.x -= invMassA * delta.x;
        bodyA->linearVelocity.y -= invMassA * delta.y;
        bodyA->angularVelocity -= invInertiaA * (c->rA.x * delta.y - c->rA.y * delta.x);
    }
    if (bDynamic) {
        bodyB->linearVelocity.x += invMassB * delta.x;
        bodyB->linearVelocity.y += invMassB * delta.y;
        bodyB->angularVelocity += invInertiaB * (c->rB.x * delta.y - c->rB.y * delta.x);
    }
}

static void Pin_PostSolve(Constraint* c) {
    if (!c->bodyA && !c->bodyB) return;

    RigidBody* bodyA = c->bodyA;
    RigidBody* bodyB = c->bodyB;

    bool hasA = (bodyA != NULL);
    bool hasB = (bodyB != NULL);
    bool aDynamic = hasA && Body_IsDynamic(bodyA);
    bool bDynamic = hasB && Body_IsDynamic(bodyB);
    if (!aDynamic && !bDynamic) return;

    Vector2 error = {
        c->worldAnchorB.x - c->worldAnchorA.x,
        c->worldAnchorB.y - c->worldAnchorA.y
    };

    float length = PR_Vec2_Length(error);
    const float slop = 0.005f;
    if (length <= slop) return;

    const float baumgarte = 0.2f;
    float totalInvMass = (aDynamic ? bodyA->invMass : 0.0f) + (bDynamic ? bodyB->invMass : 0.0f);
    if (totalInvMass <= 0.0f) return;

    float correctionMag = (length - slop) * baumgarte / totalInvMass;
    Vector2 n = error;
    PR_Vec2_Normalize(&n);
    Vector2 correction = { n.x * correctionMag, n.y * correctionMag };

    if (aDynamic) {
        bodyA->position.x += bodyA->invMass * correction.x;
        bodyA->position.y += bodyA->invMass * correction.y;
    }
    if (bDynamic) {
        bodyB->position.x -= bodyB->invMass * correction.x;
        bodyB->position.y -= bodyB->invMass * correction.y;
    }
}

static void Distance_PreSolve(Constraint* c, float dt) {
    if (!c->bodyA && !c->bodyB) return;

    if (c->bodyA) {
        c->worldAnchorA = RigidBody_GetWorldPoint(c->bodyA, c->localAnchorA);
        c->rA = RigidBody_GetWorldVector(c->bodyA, c->localAnchorA);
    } else {
        c->worldAnchorA = c->localAnchorA;
        c->rA = (Vector2){0,0};
    }
    if (c->bodyB) {
        c->worldAnchorB = RigidBody_GetWorldPoint(c->bodyB, c->localAnchorB);
        c->rB = RigidBody_GetWorldVector(c->bodyB, c->localAnchorB);
    } else {
        c->worldAnchorB = c->localAnchorB;
        c->rB = (Vector2){0,0};
    }

    Vector2 d = { c->worldAnchorB.x - c->worldAnchorA.x, c->worldAnchorB.y - c->worldAnchorA.y };
    float len = PR_Vec2_Length(d);
    if (len > 0.0001f) {
        c->massNormal.x = d.x / len;
        c->massNormal.y = d.y / len;
    } else {
        c->massNormal = (Vector2){1,0};
    }

    float rACrossN = c->rA.y * c->massNormal.x - c->rA.x * c->massNormal.y;
    float rBCrossN = c->rB.y * c->massNormal.x - c->rB.x * c->massNormal.y;

    float invMassA = (c->bodyA && Body_IsDynamic(c->bodyA)) ? c->bodyA->invMass : 0.0f;
    float invMassB = (c->bodyB && Body_IsDynamic(c->bodyB)) ? c->bodyB->invMass : 0.0f;
    float invInertiaA = (c->bodyA && Body_IsDynamic(c->bodyA)) ? c->bodyA->invInertia : 0.0f;
    float invInertiaB = (c->bodyB && Body_IsDynamic(c->bodyB)) ? c->bodyB->invInertia : 0.0f;

    float invMassSum = invMassA + invMassB;
    float invInertiaSum = (rACrossN * rACrossN) * invInertiaA + (rBCrossN * rBCrossN) * invInertiaB;
    c->effectiveMass = 1.0f / (invMassSum + invInertiaSum);
}

static void Distance_Solve(Constraint* c, float dt) {
    if (!c->bodyA && !c->bodyB) return;

    RigidBody* bodyA = c->bodyA;
    RigidBody* bodyB = c->bodyB;

    bool aDynamic = bodyA && Body_IsDynamic(bodyA);
    bool bDynamic = bodyB && Body_IsDynamic(bodyB);

    float invMassA = aDynamic ? bodyA->invMass : 0.0f;
    float invMassB = bDynamic ? bodyB->invMass : 0.0f;
    float invInertiaA = aDynamic ? bodyA->invInertia : 0.0f;
    float invInertiaB = bDynamic ? bodyB->invInertia : 0.0f;

    Vector2 velA = {0,0}, velB = {0,0};
    if (bodyA) {
        velA = (Vector2){
            bodyA->linearVelocity.x + (-c->rA.y * bodyA->angularVelocity),
            bodyA->linearVelocity.y + ( c->rA.x * bodyA->angularVelocity)
        };
    }
    if (bodyB) {
        velB = (Vector2){
            bodyB->linearVelocity.x + (-c->rB.y * bodyB->angularVelocity),
            bodyB->linearVelocity.y + ( c->rB.x * bodyB->angularVelocity)
        };
    }

    Vector2 vRel = { velB.x - velA.x, velB.y - velA.y };
    float vn = PR_Vec2_Dot(vRel, c->massNormal);

    Vector2 rVec = { c->worldAnchorB.x - c->worldAnchorA.x, c->worldAnchorB.y - c->worldAnchorA.y };
    float currentLength = PR_Vec2_Length(rVec);
    float C = currentLength - c->params.distance.restLength;

    const float baumgarte = 0.2f;
    float bias = C * baumgarte / dt;

    float impulse = -(vn + bias) * c->effectiveMass + c->normalImpulse;

    float maxImpulse = 1.0f;
    float oldImpulse = c->normalImpulse;
    c->normalImpulse = PR_CLAMP(impulse, -maxImpulse, maxImpulse);
    impulse = c->normalImpulse - oldImpulse;

    Vector2 P = { c->massNormal.x * impulse, c->massNormal.y * impulse };

    if (aDynamic) {
        bodyA->linearVelocity.x -= invMassA * P.x;
        bodyA->linearVelocity.y -= invMassA * P.y;
        bodyA->angularVelocity -= invInertiaA * (c->rA.x * P.y - c->rA.y * P.x);
    }
    if (bDynamic) {
        bodyB->linearVelocity.x += invMassB * P.x;
        bodyB->linearVelocity.y += invMassB * P.y;
        bodyB->angularVelocity += invInertiaB * (c->rB.x * P.y - c->rB.y * P.x);
    }
}

static void Distance_PostSolve(Constraint* c) {
    if (!c->bodyA && !c->bodyB) return;

    RigidBody* bodyA = c->bodyA;
    RigidBody* bodyB = c->bodyB;

    bool aDynamic = bodyA && Body_IsDynamic(bodyA);
    bool bDynamic = bodyB && Body_IsDynamic(bodyB);
    if (!aDynamic && !bDynamic) return;

    Vector2 rVec = { c->worldAnchorB.x - c->worldAnchorA.x, c->worldAnchorB.y - c->worldAnchorA.y };
    float currentLength = PR_Vec2_Length(rVec);
    float C = currentLength - c->params.distance.restLength;

    const float slop = 0.005f;
    float absC = PR_ABS(C);
    if (absC <= slop) return;

    const float baumgarte = 0.2f;
    float invMassA = aDynamic ? bodyA->invMass : 0.0f;
    float invMassB = bDynamic ? bodyB->invMass : 0.0f;
    float totalInvMass = invMassA + invMassB;
    if (totalInvMass <= 0.0f) return;

    float correctionMag = (absC - slop) * baumgarte / totalInvMass;
    Vector2 n = c->massNormal;
    float sign = (C > 0) ? 1.0f : -1.0f;

    if (aDynamic) {
        bodyA->position.x += sign * n.x * correctionMag;
        bodyA->position.y += sign * n.y * correctionMag;
    }
    if (bDynamic) {
        bodyB->position.x -= sign * n.x * correctionMag;
        bodyB->position.y -= sign * n.y * correctionMag;
    }
}

static void Spring_PreSolve(Constraint* c, float dt) {
    if (!c->bodyA && !c->bodyB) return;

    if (c->bodyA) {
        c->worldAnchorA = RigidBody_GetWorldPoint(c->bodyA, c->localAnchorA);
        c->rA = RigidBody_GetWorldVector(c->bodyA, c->localAnchorA);
    } else {
        c->worldAnchorA = c->localAnchorA;
        c->rA = (Vector2){0,0};
    }
    if (c->bodyB) {
        c->worldAnchorB = RigidBody_GetWorldPoint(c->bodyB, c->localAnchorB);
        c->rB = RigidBody_GetWorldVector(c->bodyB, c->localAnchorB);
    } else {
        c->worldAnchorB = c->localAnchorB;
        c->rB = (Vector2){0,0};
    }
}

static void Spring_Solve(Constraint* c, float dt) {
    if (!c->bodyA && !c->bodyB) return;

    RigidBody* bodyA = c->bodyA;
    RigidBody* bodyB = c->bodyB;

    bool aDynamic = bodyA && Body_IsDynamic(bodyA);
    bool bDynamic = bodyB && Body_IsDynamic(bodyB);

    float invMassA = aDynamic ? bodyA->invMass : 0.0f;
    float invMassB = bDynamic ? bodyB->invMass : 0.0f;
    float invInertiaA = aDynamic ? bodyA->invInertia : 0.0f;
    float invInertiaB = bDynamic ? bodyB->invInertia : 0.0f;

    Vector2 d = { c->worldAnchorB.x - c->worldAnchorA.x, c->worldAnchorB.y - c->worldAnchorA.y };
    float currentLength = PR_Vec2_Length(d);
    if (currentLength < 0.0001f) return;

    Vector2 n = { d.x / currentLength, d.y / currentLength };

    float stretch = currentLength - c->params.spring.restLength;
    float springForce = -c->params.spring.stiffness * stretch;

    Vector2 velA = {0,0}, velB = {0,0};
    if (bodyA) {
        velA = (Vector2){
            bodyA->linearVelocity.x + (-c->rA.y * bodyA->angularVelocity),
            bodyA->linearVelocity.y + ( c->rA.x * bodyA->angularVelocity)
        };
    }
    if (bodyB) {
        velB = (Vector2){
            bodyB->linearVelocity.x + (-c->rB.y * bodyB->angularVelocity),
            bodyB->linearVelocity.y + ( c->rB.x * bodyB->angularVelocity)
        };
    }
    Vector2 vRel = { velB.x - velA.x, velB.y - velA.y };
    float relVelAlongN = PR_Vec2_Dot(vRel, n);
    float dampingForce = -c->params.spring.damping * relVelAlongN;

    float totalForce = springForce + dampingForce;

    Vector2 impulse = { n.x * totalForce * dt, n.y * totalForce * dt };

    if (aDynamic) {
        bodyA->linearVelocity.x -= invMassA * impulse.x;
        bodyA->linearVelocity.y -= invMassA * impulse.y;
        bodyA->angularVelocity -= invInertiaA * (c->rA.x * impulse.y - c->rA.y * impulse.x);
    }
    if (bDynamic) {
        bodyB->linearVelocity.x += invMassB * impulse.x;
        bodyB->linearVelocity.y += invMassB * impulse.y;
        bodyB->angularVelocity += invInertiaB * (c->rB.x * impulse.y - c->rB.y * impulse.x);
    }
}

static void Spring_PostSolve(Constraint* c) {
}

void Constraint_PreSolve(Constraint* c, float dt) {
    if (!c) return;

    switch (c->type) {
        case PR_CONSTRAINT_PIN:
            Pin_PreSolve(c, dt);
            break;
        case PR_CONSTRAINT_DISTANCE:
            Distance_PreSolve(c, dt);
            break;
        case PR_CONSTRAINT_SPRING:
            Spring_PreSolve(c, dt);
            break;
        case PR_CONSTRAINT_WELD:
        case PR_CONSTRAINT_MOTOR:
        default:
            break;
    }
}

void Constraint_Solve(Constraint* c, float dt) {
    if (!c) return;

    switch (c->type) {
        case PR_CONSTRAINT_PIN:
            Pin_Solve(c, dt);
            break;
        case PR_CONSTRAINT_DISTANCE:
            Distance_Solve(c, dt);
            break;
        case PR_CONSTRAINT_SPRING:
            Spring_Solve(c, dt);
            break;
        case PR_CONSTRAINT_WELD:
        case PR_CONSTRAINT_MOTOR:
        default:
            break;
    }
}

void Constraint_PostSolve(Constraint* c) {
    if (!c) return;

    switch (c->type) {
        case PR_CONSTRAINT_PIN:
            Pin_PostSolve(c);
            break;
        case PR_CONSTRAINT_DISTANCE:
            Distance_PostSolve(c);
            break;
        case PR_CONSTRAINT_SPRING:
            Spring_PostSolve(c);
            break;
        case PR_CONSTRAINT_WELD:
        case PR_CONSTRAINT_MOTOR:
        default:
            break;
    }
}

void Constraint_DrawDebug(Constraint* c, Color color) {
    (void)c;
    (void)color;
}