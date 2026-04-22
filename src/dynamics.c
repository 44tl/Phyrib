#include "phyrib/internal.h"
#include "phyrib/collision.h"
#include "phyrib/rigid_body.h"

static inline Vector2 CrossVs(const Vector2 v, float s) {
    return (Vector2){ -v.y * s, v.x * s };
}

static inline Vector2 CrossSv(float s, const Vector2 v) {
    return (Vector2){ s * v.y, -s * v.x };
}

static inline float CrossRn(const Vector2 r, const Vector2 n) {
    return r.x * n.y - r.y * n.x;
}

void Collision_PreSolve(Manifold* m) {
    if (!m || m->contactCount <= 0) return;

    for (int i = 0; i < m->contactCount; i++) {
    }
}

void Collision_Resolve(Manifold* m, float dt) {
    if (!m || m->contactCount <= 0) return;

    RigidBody* bodyA = m->bodyA;
    RigidBody* bodyB = m->bodyB;

    if (Body_IsStatic(bodyA) && Body_IsStatic(bodyB)) return;

    PhysicsMaterial matA = RigidBody_GetMaterial(bodyA);
    PhysicsMaterial matB = RigidBody_GetMaterial(bodyB);
    PhysicsMaterial mat;
    mat.friction = sqrtf(matA.friction * matB.friction);
    mat.restitution = PR_MAX(matA.restitution, matB.restitution);
    mat.density = (matA.density + matB.density) * 0.5f;

    float invMassA = Body_IsDynamic(bodyA) ? bodyA->invMass : 0.0f;
    float invMassB = Body_IsDynamic(bodyB) ? bodyB->invMass : 0.0f;
    float invInertiaA = Body_IsDynamic(bodyA) ? bodyA->invInertia : 0.0f;
    float invInertiaB = Body_IsDynamic(bodyB) ? bodyB->invInertia : 0.0f;

    float e = mat.restitution;
    float mu = mat.friction;

    for (int c = 0; c < m->contactCount; c++) {
        Vector2 contact = m->contacts[c];
        Vector2 normal = m->normals[c];

        Vector2 rA = { contact.x - bodyA->position.x, contact.y - bodyA->position.y };
        Vector2 rB = { contact.x - bodyB->position.x, contact.y - bodyB->position.y };

        Vector2 velA = {
            bodyA->linearVelocity.x + (-rA.y * bodyA->angularVelocity),
            bodyA->linearVelocity.y + ( rA.x * bodyA->angularVelocity)
        };
        Vector2 velB = {
            bodyB->linearVelocity.x + (-rB.y * bodyB->angularVelocity),
            bodyB->linearVelocity.y + ( rB.x * bodyB->angularVelocity)
        };
        Vector2 vRel = { velB.x - velA.x, velB.y - velA.y };

        float vRelDotN = PR_Vec2_Dot(vRel, normal);

        if (vRelDotN > 0.0f) continue;

        float rACrossN = CrossRn(rA, normal);
        float rBCrossN = CrossRn(rB, normal);

        float invMassSum = invMassA + invMassB;
        float invInertiaSum = (rACrossN * rACrossN) * invInertiaA +
                              (rBCrossN * rBCrossN) * invInertiaB;

        float j = -(1.0f + e) * vRelDotN;
        float denom = invMassSum + invInertiaSum;
        if (denom > 0.000001f) {
            j /= denom;
        } else {
            j = 0.0f;
        }

        Vector2 impulse = CrossSv(j, normal);

        if (Body_IsDynamic(bodyA)) {
            bodyA->linearVelocity.x -= invMassA * impulse.x;
            bodyA->linearVelocity.y -= invMassA * impulse.y;
            bodyA->angularVelocity -= invInertiaA * CrossRn(rA, impulse);
        }

        if (Body_IsDynamic(bodyB)) {
            bodyB->linearVelocity.x += invMassB * impulse.x;
            bodyB->linearVelocity.y += invMassB * impulse.y;
            bodyB->angularVelocity += invInertiaB * CrossRn(rB, impulse);
        }

        velA = (Vector2){
            bodyA->linearVelocity.x + (-rA.y * bodyA->angularVelocity),
            bodyA->linearVelocity.y + ( rA.x * bodyA->angularVelocity)
        };
        velB = (Vector2){
            bodyB->linearVelocity.x + (-rB.y * bodyB->angularVelocity),
            bodyB->linearVelocity.y + ( rB.x * bodyB->angularVelocity)
        };
        vRel = (Vector2){ velB.x - velA.x, velB.y - velA.y };

        float vRelDotN_post = PR_Vec2_Dot(vRel, normal);
        Vector2 tangent = {
            vRel.x - vRelDotN_post * normal.x,
            vRel.y - vRelDotN_post * normal.y
        };

        float tangentLen = PR_Vec2_Length(tangent);
        if (tangentLen > 0.0001f) {
            tangent.x /= tangentLen;
            tangent.y /= tangentLen;

            float rACrossT = CrossRn(rA, tangent);
            float rBCrossT = CrossRn(rB, tangent);

            float invMassSumT = invMassA + invMassB;
            float invInertiaSumT = (rACrossT * rACrossT) * invInertiaA +
                                   (rBCrossT * rBCrossT) * invInertiaB;

            float jt = -PR_Vec2_Dot(vRel, tangent);
            float denomT = invMassSumT + invInertiaSumT;
            if (denomT > 0.000001f) {
                jt /= denomT;
            } else {
                jt = 0.0f;
            }

            float maxFriction = mu * PR_ABS(j);
            if (jt > maxFriction) jt = maxFriction;
            if (jt < -maxFriction) jt = -maxFriction;

            Vector2 frictionImpulse = { tangent.x * jt, tangent.y * jt };

            if (Body_IsDynamic(bodyA)) {
                bodyA->linearVelocity.x -= invMassA * frictionImpulse.x;
                bodyA->linearVelocity.y -= invMassA * frictionImpulse.y;
                bodyA->angularVelocity -= invInertiaA * CrossRn(rA, frictionImpulse);
            }

            if (Body_IsDynamic(bodyB)) {
                bodyB->linearVelocity.x += invMassB * frictionImpulse.x;
                bodyB->linearVelocity.y += invMassB * frictionImpulse.y;
                bodyB->angularVelocity += invInertiaB * CrossRn(rB, frictionImpulse);
            }
        }
    }
}

void Collision_PostSolve(Manifold* m) {
    if (!m || m->contactCount <= 0) return;

    RigidBody* bodyA = m->bodyA;
    RigidBody* bodyB = m->bodyB;

    if (Body_IsStatic(bodyA) && Body_IsStatic(bodyB)) return;

    const float slop = 0.01f;
    const float baumgarte = 0.2f;

    float invMassA = Body_IsDynamic(bodyA) ? bodyA->invMass : 0.0f;
    float invMassB = Body_IsDynamic(bodyB) ? bodyB->invMass : 0.0f;

    float totalInvMass = invMassA + invMassB;
    if (totalInvMass <= 0.0f) return;

    float correction = PR_MAX(m->penetration - slop, 0.0f) / totalInvMass * baumgarte;

    Vector2 normal = m->normals[0];

    if (Body_IsDynamic(bodyA)) {
        bodyA->position.x -= invMassA * correction * normal.x;
        bodyA->position.y -= invMassA * correction * normal.y;
    }

    if (Body_IsDynamic(bodyB)) {
        bodyB->position.x += invMassB * correction * normal.x;
        bodyB->position.y += invMassB * correction * normal.y;
    }
}