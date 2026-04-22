#include "phyrib/internal.h"
#include "phyrib/rigid_body.h"
#include "phyrib/shape.h"
#include "phyrib/physics_world.h"
#include <string.h>
#include <stdio.h>

struct RigidBody {
    PhysicsBodyID id;
    BodyType type;
    uint8_t flags;

    Vector2 position;
    float angle;

    Vector2 linearVelocity;
    float angularVelocity;

    Vector2 force;
    float torque;

    float mass;
    float invMass;
    float inertia;
    float invInertia;

    float linearDamping;
    float angularDamping;

    PhysicsMaterial material;
    CollisionFilter filter;

    Shape* shape;

    float sleepTime;
    int32_t islandIndex;

    void* userData;

    RigidBody* next;
    RigidBody* prev;

    char name[32];
};

static PhysicsBodyID gNextBodyID = 1;

static void RigidBody_ComputeMassData(RigidBody* body) {
    if (body->type == PR_BODY_STATIC) {
        body->mass = 0.0f;
        body->invMass = 0.0f;
        body->inertia = 0.0f;
        body->invInertia = 0.0f;
        return;
    }

    if (body->shape) {
        float density = body->material.density;
        Shape_ComputeMass(body->shape, density, &body->inertia);
        body->mass = Shape_GetArea(body->shape) * density;
    } else {
        body->mass = 1.0f;
        body->inertia = 1.0f;
    }

    if (body->mass > 0.0f) {
        body->invMass = 1.0f / body->mass;
    } else {
        body->invMass = 0.0f;
    }

    if (body->inertia > 0.0f && body->type == PR_BODY_DYNAMIC && !body->fixedRotation) {
        body->invInertia = 1.0f / body->inertia;
    } else {
        body->invInertia = 0.0f;
    }
}

RigidBody* RigidBody_Create(const RigidBodyDef* def) {
    if (!def || !def->shape) return NULL;

    RigidBody* body = (RigidBody*)PR_AllocZeroed(sizeof(RigidBody));
    if (!body) return NULL;

    body->id = GenerateBodyID();
    body->type = def->type;
    body->position = def->position;
    body->angle = def->angle;
    body->linearVelocity = def->linearVelocity;
    body->angularVelocity = def->angularVelocity;
    body->linearDamping = PR_Clamp(def->linearDamping, 0.0f, 1.0f);
    body->angularDamping = PR_Clamp(def->angularDamping, 0.0f, 1.0f);
    body->fixedRotation = def->fixedRotation;
    body->material = def->material;
    body->filter = def->filter;
    body->userData = def->userData;
    body->sleepTime = 0.0f;
    body->islandIndex = -1;
    body->flags = BODY_FLAG_AWAKE;

    body->shape = (Shape*)PR_Alloc(sizeof(Shape));
    Shape_Clone(body->shape, def->shape);

    RigidBody_ComputeMassData(body);

    body->force = (Vector2){0, 0};
    body->torque = 0.0f;

    snprintf(body->name, sizeof(body->name), "Body_%u", body->id);

    return body;
}

void RigidBody_Destroy(RigidBody* body) {
    if (!body) return;
    if (body->shape) Shape_Destroy(body->shape);
    PR_Free(body);
}

const char* RigidBody_GetName(RigidBody* body) { return body ? body->name : NULL; }
PhysicsBodyID RigidBody_GetID(RigidBody* body) { return body ? body->id : 0; }
BodyType RigidBody_GetType(RigidBody* body) { return body ? body->type : PR_BODY_STATIC; }

void RigidBody_SetType(RigidBody* body, BodyType type) {
    if (!body) return;
    body->type = type;
    if (type == PR_BODY_STATIC) {
        body->mass = 0; body->invMass = 0;
        body->inertia = 0; body->invInertia = 0;
    } else {
        RigidBody_ComputeMassData(body);
    }
}

Vector2 RigidBody_GetPosition(RigidBody* body) { return body ? body->position : (Vector2){0,0}; }
void RigidBody_SetPosition(RigidBody* body, Vector2 pos) { if (body) body->position = pos; }
float RigidBody_GetAngle(RigidBody* body) { return body ? body->angle : 0.0f; }
void RigidBody_SetAngle(RigidBody* body, float angle) { if (body) body->angle = angle; }

void RigidBody_GetTransform(RigidBody* body, Vector2* pos, float* angle) {
    if (!body) return;
    if (pos) *pos = body->position;
    if (angle) *angle = body->angle;
}

void RigidBody_SetTransform(RigidBody* body, Vector2 pos, float angle) {
    if (!body) return;
    body->position = pos;
    body->angle = angle;
}

Vector2 RigidBody_GetLinearVelocity(RigidBody* body) { return body ? body->linearVelocity : (Vector2){0,0}; }
void RigidBody_SetLinearVelocity(RigidBody* body, Vector2 vel) { if (body) body->linearVelocity = vel; }
float RigidBody_GetAngularVelocity(RigidBody* body) { return body ? body->angularVelocity : 0.0f; }
void RigidBody_SetAngularVelocity(RigidBody* body, float vel) { if (body) body->angularVelocity = vel; }

void RigidBody_ApplyLinearImpulse(RigidBody* body, Vector2 impulse, Vector2 point) {
    if (!body || body->type != PR_BODY_DYNAMIC) return;
    body->linearVelocity.x += body->invMass * impulse.x;
    body->linearVelocity.y += body->invMass * impulse.y;
    body->angularVelocity += body->invInertia * (point.x * impulse.y - point.y * impulse.x);
}

void RigidBody_ApplyAngularImpulse(RigidBody* body, float impulse) {
    if (!body || body->type != PR_BODY_DYNAMIC) return;
    body->angularVelocity += body->invInertia * impulse;
}

void RigidBody_ApplyForce(RigidBody* body, Vector2 force, Vector2 point) {
    if (!body || body->type != PR_BODY_DYNAMIC) return;
    body->force.x += force.x;
    body->force.y += force.y;
    body->torque += point.x * force.y - point.y * force.x;
}

void RigidBody_ApplyTorque(RigidBody* body, float torque) {
    if (!body || body->type != PR_BODY_DYNAMIC) return;
    body->torque += torque;
}

void RigidBody_ApplyForceToCenter(RigidBody* body, Vector2 force) {
    if (!body || body->type != PR_BODY_DYNAMIC) return;
    body->force.x += force.x;
    body->force.y += force.y;
}

Vector2 RigidBody_GetForce(RigidBody* body) { return body ? body->force : (Vector2){0,0}; }
float RigidBody_GetTorque(RigidBody* body) { return body ? body->torque : 0.0f; }

void RigidBody_ClearForces(RigidBody* body) {
    if (body) {
        body->force = (Vector2){0, 0};
        body->torque = 0.0f;
    }
}

float RigidBody_GetMass(RigidBody* body) { return body ? body->mass : 0.0f; }
void RigidBody_SetMass(RigidBody* body, float mass) {
    if (!body || body->type == PR_BODY_STATIC) return;
    body->mass = PR_MAX(mass, 0.001f);
    body->invMass = (body->mass > 0.0f) ? 1.0f / body->mass : 0.0f;
    if (body->inertia > 0.0f) {
        body->invInertia = body->mass / body->inertia;
    }
}
float RigidBody_GetInvMass(RigidBody* body) { return body ? body->invMass : 0.0f; }

float RigidBody_GetInertia(RigidBody* body) { return body ? body->inertia : 0.0f; }
void RigidBody_SetInertia(RigidBody* body, float inertia) {
    if (!body || body->type == PR_BODY_STATIC) return;
    body->inertia = PR_MAX(inertia, 0.001f);
    body->invInertia = (body->inertia > 0.0f && !body->fixedRotation) ? 1.0f / body->inertia : 0.0f;
}
float RigidBody_GetInvInertia(RigidBody* body) { return body ? body->invInertia : 0.0f; }

void RigidBody_SetFixedRotation(RigidBody* body, bool fixed) { if (body) { body->fixedRotation = fixed; RigidBody_ComputeMassData(body); } }
bool RigidBody_IsFixedRotation(RigidBody* body) { return body ? body->fixedRotation : false; }

float RigidBody_GetLinearDamping(RigidBody* body) { return body ? body->linearDamping : 0.0f; }
void RigidBody_SetLinearDamping(RigidBody* body, float damping) { if (body) body->linearDamping = PR_Clamp(damping, 0.0f, 1.0f); }
float RigidBody_GetAngularDamping(RigidBody* body) { return body ? body->angularDamping : 0.0f; }
void RigidBody_SetAngularDamping(RigidBody* body, float damping) { if (body) body->angularDamping = PR_Clamp(damping, 0.0f, 1.0f); }

PhysicsMaterial RigidBody_GetMaterial(RigidBody* body) { return body ? body->material : PR_MATERIAL_DEFAULT; }
void RigidBody_SetMaterial(RigidBody* body, PhysicsMaterial mat) { if (body) { body->material = mat; RigidBody_ComputeMassData(body); } }

CollisionFilter RigidBody_GetCollisionFilter(RigidBody* body) { return body ? body->filter : (CollisionFilter){0xFFFF, 0xFFFF, 0}; }
void RigidBody_SetCollisionFilter(RigidBody* body, CollisionFilter filter) { if (body) body->filter = filter; }

bool RigidBody_IsSensor(RigidBody* body) { return body ? body->type == PR_BODY_KINEMATIC && body->isSensor : false; }
void RigidBody_SetSensor(RigidBody* body, bool sensor) { if (body) body->isSensor = sensor; }

bool RigidBody_IsAwake(RigidBody* body) { return body ? (body->flags & BODY_FLAG_SLEEPING) == 0 : false; }
void RigidBody_SetAwake(RigidBody* body, bool awake) {
    if (!body) return;
    if (awake) {
        body->flags &= ~BODY_FLAG_SLEEPING;
        body->sleepTime = 0.0f;
    } else {
        body->flags |= BODY_FLAG_SLEEPING;
    }
}
void RigidBody_Sleep(RigidBody* body) { if (body) body->flags |= BODY_FLAG_SLEEPING; }
void RigidBody_WakeUp(RigidBody* body) { if (body) { body->flags &= ~BODY_FLAG_SLEEPING; body->sleepTime = 0.0f; } }

Shape* RigidBody_GetShape(RigidBody* body) { return body ? body->shape : NULL; }
void RigidBody_SetShape(RigidBody* body, const Shape* shape) {
    if (!body || !shape) return;
    if (body->shape) Shape_Destroy(body->shape);
    body->shape = (Shape*)PR_Alloc(sizeof(Shape));
    Shape_Clone(body->shape, shape);
    RigidBody_ComputeMassData(body);
}

void* RigidBody_GetUserData(RigidBody* body) { return body ? body->userData : NULL; }
void RigidBody_SetUserData(RigidBody* body, void* data) { if (body) body->userData = data; }

// Kinematic movement
void RigidBody_MoveTo(RigidBody* body, Vector2 target, float dt) {
    if (!body || body->type != PR_BODY_KINEMATIC) return;
    body->position.x = target.x;
    body->position.y = target.y;
}

void RigidBody_MoveBy(RigidBody* body, Vector2 delta, float dt) {
    if (!body || body->type != PR_BODY_KINEMATIC) return;
    body->position.x += delta.x * dt;
    body->position.y += delta.y * dt;
}

void RigidBody_SetKinematicVelocity(RigidBody* body, Vector2 vel) {
    if (!body || body->type != PR_BODY_KINEMATIC) return;
    body->linearVelocity = vel;
}

// Geometry
Vector2 RigidBody_GetWorldPoint(RigidBody* body, Vector2 localPoint) {
    if (!body) return (Vector2){0,0};
    Vector2 result;
    PR_Vec2_Rotate(&result, &localPoint, body->angle);
    result.x += body->position.x;
    result.y += body->position.y;
    return result;
}

Vector2 RigidBody_GetLocalPoint(RigidBody* body, Vector2 worldPoint) {
    if (!body) return (Vector2){0,0};
    Vector2 d = { worldPoint.x - body->position.x, worldPoint.y - body->position.y };
    Vector2 result;
    PR_Vec2_Rotate(&result, &d, -body->angle);
    return result;
}

Vector2 RigidBody_GetWorldVector(RigidBody* body, Vector2 localVec) {
    if (!body) return (Vector2){0,0};
    Vector2 result;
    PR_Vec2_Rotate(&result, &localVec, body->angle);
    return result;
}

Vector2 RigidBody_GetLocalVector(RigidBody* body, Vector2 worldVec) {
    if (!body) return (Vector2){0,0};
    Vector2 result;
    PR_Vec2_Rotate(&result, &worldVec, -body->angle);
    return result;
}

float RigidBody_GetLinearVelocityAtPoint(RigidBody* body, Vector2 worldPoint) {
    if (!body) return 0.0f;
    Vector2 r = { worldPoint.x - body->position.x, worldPoint.y - body->position.y };
    return PR_Vec2_Length((Vector2){ body->linearVelocity.x + (-r.y * body->angularVelocity), body->linearVelocity.y + (r.x * body->angularVelocity) });
}

float RigidBody_GetKineticEnergy(RigidBody* body) {
    if (!body) return 0.0f;
    float v2 = PR_Vec2_LengthSqr(body->linearVelocity);
    float w2 = body->angularVelocity * body->angularVelocity;
    return 0.5f * (body->mass * v2 + body->inertia * w2);
}

float RigidBody_GetPotentialEnergy(RigidBody* body, float gravityMag) {
    if (!body) return 0.0f;
    return body->mass * gravityMag * body->position.y;
}

void RigidBody_GetAABB(RigidBody* body, Vector2* min, Vector2* max) {
    if (!body || !body->shape) return;
    Shape_ComputeAABB(body->shape, body->position.x, body->position.y, body->angle, min, max);
}

// Rotation
void RigidBody_Rotate(RigidBody* body, float angle) { if (body) body->angle += angle; }
void RigidBody_RotateAround(RigidBody* body, Vector2 point, float angle) {
    if (!body) return;
    Vector2 d = { body->position.x - point.x, body->position.y - point.y };
    PR_Vec2_Rotate(&d, &d, angle);
    body->position.x = point.x + d.x;
    body->position.y = point.y + d.y;
    body->angle += angle;
}
