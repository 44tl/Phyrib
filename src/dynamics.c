#include "phyrib/internal.h"
#include "phyrib/collision.h"
#include "phyrib/rigid_body.h"

// ============================================================================
// MATH HELPERS
// ============================================================================

// Cross product of vector v and scalar s -> vector (v × s)
static inline Vector2 CrossVs(const Vector2 v, float s) {
    return (Vector2){ -v.y * s, v.x * s };
}

// Cross product of scalar s and vector v -> vector (s × v)
static inline Vector2 CrossSv(float s, const Vector2 v) {
    return (Vector2){ s * v.y, -s * v.x };
}

// Compute torque arm cross product: r × n (returns scalar in 2D)
static inline float CrossRn(const Vector2 r, const Vector2 n) {
    return r.x * n.y - r.y * n.x;
}

// ============================================================================
// COLLISION PRE-SOLVE
// ============================================================================

void Collision_PreSolve(Manifold* m) {
    if (!m || m->contactCount <= 0) return;

    // Precompute contact data for warm starting (future extension)
    // For now, we just validate contact points relative to bodies
    for (int i = 0; i < m->contactCount; i++) {
        // Contact points are already in world space
        // Could precompute relative offsets rA, rB here for warm starting
    }
}

// ============================================================================
// COLLISION RESOLVE - Impulse-based velocity resolution
// ============================================================================

void Collision_Resolve(Manifold* m, float dt) {
    if (!m || m->contactCount <= 0) return;

    RigidBody* bodyA = m->bodyA;
    RigidBody* bodyB = m->bodyB;

    // Skip if both bodies are static/kinematic (no mass)
    if (Body_IsStatic(bodyA) && Body_IsStatic(bodyB)) return;

    // Mixed material (geometric mean for friction, max for restitution)
    PhysicsMaterial matA = RigidBody_GetMaterial(bodyA);
    PhysicsMaterial matB = RigidBody_GetMaterial(bodyB);
    PhysicsMaterial mat;
    mat.friction = sqrtf(matA.friction * matB.friction);
    mat.restitution = PR_MAX(matA.restitution, matB.restitution);
    mat.density = (matA.density + matB.density) * 0.5f;

    // Mass properties (handling static bodies with infinite mass)
    float invMassA = Body_IsDynamic(bodyA) ? bodyA->invMass : 0.0f;
    float invMassB = Body_IsDynamic(bodyB) ? bodyB->invMass : 0.0f;
    float invInertiaA = Body_IsDynamic(bodyA) ? bodyA->invInertia : 0.0f;
    float invInertiaB = Body_IsDynamic(bodyB) ? bodyB->invInertia : 0.0f;

    float e = mat.restitution;
    float mu = mat.friction;

    // Process each contact point
    for (int c = 0; c < m->contactCount; c++) {
        Vector2 contact = m->contacts[c];
        Vector2 normal = m->normals[c];

        // Contact-to-center vectors (from body centers to contact point)
        Vector2 rA = { contact.x - bodyA->position.x, contact.y - bodyA->position.y };
        Vector2 rB = { contact.x - bodyB->position.x, contact.y - bodyB->position.y };

        // Relative velocity at contact: vRel = (vB + ωB × rB) - (vA + ωA × rA)
        Vector2 velA = {
            bodyA->linearVelocity.x + (-rA.y * bodyA->angularVelocity),
            bodyA->linearVelocity.y + ( rA.x * bodyA->angularVelocity)
        };
        Vector2 velB = {
            bodyB->linearVelocity.x + (-rB.y * bodyB->angularVelocity),
            bodyB->linearVelocity.y + ( rB.x * bodyB->angularVelocity)
        };
        Vector2 vRel = { velB.x - velA.x, velB.y - velA.y };

        // Relative velocity along normal
        float vRelDotN = PR_Vec2_Dot(vRel, normal);

        // Don't resolve if bodies are separating
        if (vRelDotN > 0.0f) continue;

        // Compute impulse scalar (normal direction)
        // j = -(1 + e) * vRel·n / (1/ma + 1/mb + (rA×n)²/ia + (rB×n)²/ib)
        float rACrossN = CrossRn(rA, normal);  // rA × n
        float rBCrossN = CrossRn(rB, normal);  // rB × n

        float invMassSum = invMassA + invMassB;
        float invInertiaSum = (rACrossN * rACrossN) * invInertiaA +
                              (rBCrossN * rBCrossN) * invInertiaB;

        float j = -(1.0f + e) * vRelDotN;
        float denom = invMassSum + invInertiaSum;
        if (denom > 0.000001f) {
            j /= denom;
        } else {
            j = 0.0f;  // Infinite mass (static vs static) - shouldn't reach here
        }

        // Apply normal impulse
        Vector2 impulse = CrossSv(j, normal);  // j * normal

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

        // ====================================================================
        // FRICTION IMPULSE (Coulomb)
        // ====================================================================

        // Recompute relative velocity after normal impulse
        velA = (Vector2){
            bodyA->linearVelocity.x + (-rA.y * bodyA->angularVelocity),
            bodyA->linearVelocity.y + ( rA.x * bodyA->angularVelocity)
        };
        velB = (Vector2){
            bodyB->linearVelocity.x + (-rB.y * bodyB->angularVelocity),
            bodyB->linearVelocity.y + ( rB.x * bodyB->angularVelocity)
        };
        vRel = (Vector2){ velB.x - velA.x, velB.y - velA.y };

        // Tangential direction: t = normalize(vRel - (vRel·n)n)
        float vRelDotN_post = PR_Vec2_Dot(vRel, normal);
        Vector2 tangent = {
            vRel.x - vRelDotN_post * normal.x,
            vRel.y - vRelDotN_post * normal.y
        };

        float tangentLen = PR_Vec2_Length(tangent);
        if (tangentLen > 0.0001f) {
            tangent.x /= tangentLen;
            tangent.y /= tangentLen;

            // Friction impulse magnitude
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

            // Clamp Coulomb friction: |jt| ≤ μ * |j|
            float maxFriction = mu * PR_ABS(j);
            if (jt > maxFriction) jt = maxFriction;
            if (jt < -maxFriction) jt = -maxFriction;

            // Apply friction impulse
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

// ============================================================================
// COLLISION POST-SOLVE - Positional Correction (Baumgarte)
// ============================================================================

void Collision_PostSolve(Manifold* m) {
    if (!m || m->contactCount <= 0) return;

    RigidBody* bodyA = m->bodyA;
    RigidBody* bodyB = m->bodyB;

    // Only correct if at least one dynamic body
    if (Body_IsStatic(bodyA) && Body_IsStatic(bodyB)) return;

    // Baumgarte stabilization parameters
    const float slop = 0.01f;         // Penetration allowance (cm)
    const float baumgarte = 0.2f;     // Position correction factor

    float invMassA = Body_IsDynamic(bodyA) ? bodyA->invMass : 0.0f;
    float invMassB = Body_IsDynamic(bodyB) ? bodyB->invMass : 0.0f;

    float totalInvMass = invMassA + invMassB;
    if (totalInvMass <= 0.0f) return;

    // Correction scalar: λ = (penetration - slop) / (1/ma + 1/mb) * baumgarte
    float correction = PR_MAX(m->penetration - slop, 0.0f) / totalInvMass * baumgarte;

    // Normal direction (use first contact normal)
    Vector2 normal = m->normals[0];

    // Separate bodies
    if (Body_IsDynamic(bodyA)) {
        bodyA->position.x -= invMassA * correction * normal.x;
        bodyA->position.y -= invMassA * correction * normal.y;
    }

    if (Body_IsDynamic(bodyB)) {
        bodyB->position.x += invMassB * correction * normal.x;
        bodyB->position.y += invMassB * correction * normal.y;
    }
}
