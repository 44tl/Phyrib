#include "phyrib/internal.h"
#include "phyrib/collision.h"
#include <math.h>

static inline Vector2 TransformPoint(Vector2 p, float angle) {
    float c = cosf(angle);
    float s = sinf(angle);
    return (Vector2){ p.x * c - p.y * s, p.x * s + p.y * c };
}

static inline Vector2 InverseTransformPoint(Vector2 p, float angle) {
    float c = cosf(-angle);
    float s = sinf(-angle);
    return (Vector2){ p.x * c - p.y * s, p.x * s + p.y * c };
}

bool Collision_AABBOverlap(Vector2 minA, Vector2 maxA, Vector2 minB, Vector2 maxB) {
    if (maxA.x < minB.x || minA.x > maxB.x) return false;
    if (maxA.y < minB.y || minA.y > maxB.y) return false;
    return true;
}

static bool Collision_TestCircleCircle(const Shape* a, float ax, float ay, float aa,
                                        const Shape* b, float bx, float by, float ba,
                                        Manifold* m) {
    float rA = a->radius;
    float rB = b->radius;

    Vector2 posA = { ax, ay };
    Vector2 posB = { bx, by };

    Vector2 delta;
    PR_Vec2_Sub(&delta, posB, posA);
    float distSqr = PR_Vec2_LengthSqr(delta);
    float radiusSum = rA + rB;

    if (distSqr > radiusSum * radiusSum) {
        return false;
    }

    float dist = sqrtf(distSqr);

    if (dist < 0.0001f) {
        m->normal = (Vector2){1.0f, 0.0f};
        m->penetration = radiusSum;
    } else {
        m->normal.x = delta.x / dist;
        m->normal.y = delta.y / dist;
        m->penetration = radiusSum - dist;
    }

    m->contactCount = 1;
    m->contacts[0] = posA;
    m->contacts[0].x += m->normal.x * (rA - m->penetration * 0.5f);
    m->contacts[0].y += m->normal.y * (rA - m->penetration * 0.5f);

    return true;
}

static bool Collision_TestCirclePolygon(const Shape* circle, float cx, float cy, float cAngle,
                                         const Shape* poly, float px, float py, float pAngle,
                                         Manifold* m) {
    Vector2 polyVerts[64];
    int polyCount = poly->vertexCount;
    if (poly->type == PR_SHAPE_RECTANGLE) {
        polyCount = 4;
        float hw = poly->width * 0.5f;
        float hh = poly->height * 0.5f;
        polyVerts[0] = (Vector2){ hw, hh };
        polyVerts[1] = (Vector2){ -hw, hh };
        polyVerts[2] = (Vector2){ -hw, -hh };
        polyVerts[3] = (Vector2){ hw, -hh };
    } else {
        for (int i = 0; i < polyCount; i++) {
            polyVerts[i] = poly->vertices[i];
        }
    }

    Vector2 circleWorld = { cx, cy };
    Vector2 circleLocal = InverseTransformPoint(circleWorld, pAngle);
    circleLocal.x -= px;
    circleLocal.y -= py;

    float minDistSq = FLT_MAX;
    int closestEdge = -1;
    Vector2 closestLocal = {0, 0};

    for (int i = 0; i < polyCount; i++) {
        int j = (i + 1) % polyCount;
        Vector2 v1 = polyVerts[i];
        Vector2 v2 = polyVerts[j];
        Vector2 edge;
        PR_Vec2_Sub(&edge, v2, v1);
        float edgeLenSq = PR_Vec2_LengthSqr(edge);

        Vector2 toCircle;
        PR_Vec2_Sub(&toCircle, circleLocal, v1);

        if (edgeLenSq < 0.0001f) {
            Vector2 diff;
            PR_Vec2_Sub(&diff, circleLocal, v1);
            float dSq = PR_Vec2_LengthSqr(diff);
            if (dSq < minDistSq) {
                minDistSq = dSq;
                closestEdge = i;
                closestLocal = v1;
            }
            continue;
        }

        float t = PR_Vec2_Dot(toCircle, edge) / edgeLenSq;
        t = PR_Clamp(t, 0.0f, 1.0f);
        Vector2 closest = { v1.x + t * edge.x, v1.y + t * edge.y };
        Vector2 diff;
        PR_Vec2_Sub(&diff, circleLocal, closest);
        float dSq = PR_Vec2_LengthSqr(diff);
        if (dSq < minDistSq) {
            minDistSq = dSq;
            closestEdge = i;
            closestLocal = closest;
        }
    }

    float radius = circle->radius;
    if (minDistSq > radius * radius) return false;
    float dist = sqrtf(minDistSq);

    Vector2 closestWorld = TransformPoint(closestLocal, pAngle);
    closestWorld.x += px;
    closestWorld.y += py;

    if (dist > 0.0001f) {
        m->normal.x = (closestWorld.x - circleWorld.x) / dist;
        m->normal.y = (closestWorld.y - circleWorld.y) / dist;
    } else {
        Vector2 v1 = polyVerts[closestEdge];
        Vector2 v2 = polyVerts[(closestEdge + 1) % polyCount];
        Vector2 edge;
        PR_Vec2_Sub(&edge, v2, v1);
        Vector2 n = { edge.y, -edge.x };
        PR_Vec2_Normalize(&n);
        m->normal = TransformPoint(n, pAngle);
        PR_Vec2_Normalize(&m->normal);
    }

    m->penetration = radius - dist;
    m->contactCount = 1;
    m->contacts[0] = closestWorld;

    return true;
}

static bool Collision_TestPolygonPolygon(const Shape* a, float aX, float aY, float aAngle,
                                          const Shape* b, float bX, float bY, float bAngle,
                                          Manifold* m) {
    Vector2 vertsA[64];
    float normalsA[64];
    int countA;
    if (a->type == PR_SHAPE_RECTANGLE) {
        countA = 4;
        float hw = a->width * 0.5f;
        float hh = a->height * 0.5f;
        vertsA[0] = (Vector2){ hw, hh };
        vertsA[1] = (Vector2){ -hw, hh };
        vertsA[2] = (Vector2){ -hw, -hh };
        vertsA[3] = (Vector2){ hw, -hh };
        for (int i = 0; i < 4; i++) {
            int j = (i + 1) % 4;
            Vector2 e = { vertsA[j].x - vertsA[i].x, vertsA[j].y - vertsA[i].y };
            Vector2 n = { e.y, -e.x };
            PR_Vec2_Normalize(&n);
            normalsA[i] = atan2f(n.y, n.x);
        }
    } else {
        countA = a->vertexCount;
        for (int i = 0; i < countA; i++) {
            vertsA[i] = a->vertices[i];
            normalsA[i] = a->normals[i];
        }
    }

    Vector2 vertsB[64];
    float normalsB[64];
    int countB;
    if (b->type == PR_SHAPE_RECTANGLE) {
        countB = 4;
        float hw = b->width * 0.5f;
        float hh = b->height * 0.5f;
        vertsB[0] = (Vector2){ hw, hh };
        vertsB[1] = (Vector2){ -hw, hh };
        vertsB[2] = (Vector2){ -hw, -hh };
        vertsB[3] = (Vector2){ hw, -hh };
        for (int i = 0; i < 4; i++) {
            int j = (i + 1) % 4;
            Vector2 e = { vertsB[j].x - vertsB[i].x, vertsB[j].y - vertsB[i].y };
            Vector2 n = { e.y, -e.x };
            PR_Vec2_Normalize(&n);
            normalsB[i] = atan2f(n.y, n.x);
        }
    } else {
        countB = b->vertexCount;
        for (int i = 0; i < countB; i++) {
            vertsB[i] = b->vertices[i];
            normalsB[i] = b->normals[i];
        }
    }

    float minOverlap = FLT_MAX;
    Vector2 collisionNormal = {0, 0};
    bool bestFromA = false;
    int bestIndex = -1;

    for (int i = 0; i < countA; i++) {
        Vector2 axis = { cosf(normalsA[i]), sinf(normalsA[i]) };
        float minA = FLT_MAX, maxA = -FLT_MAX;
        for (int v = 0; v < countA; v++) {
            Vector2 wv = TransformPoint(vertsA[v], aAngle);
            wv.x += aX; wv.y += aY;
            float proj = PR_Vec2_Dot(wv, axis);
            if (proj < minA) minA = proj;
            if (proj > maxA) maxA = proj;
        }
        float minB = FLT_MAX, maxB = -FLT_MAX;
        for (int v = 0; v < countB; v++) {
            Vector2 wv = TransformPoint(vertsB[v], bAngle);
            wv.x += bX; wv.y += bY;
            float proj = PR_Vec2_Dot(wv, axis);
            if (proj < minB) minB = proj;
            if (proj > maxB) maxB = proj;
        }
        if (maxA < minB || maxB < minA) return false;
        float overlap = PR_MIN(maxA, maxB) - PR_MAX(minA, minB);
        if (overlap < minOverlap) {
            minOverlap = overlap;
            collisionNormal = axis;
            bestFromA = true;
            bestIndex = i;
        }
    }

    for (int i = 0; i < countB; i++) {
        Vector2 axis = { cosf(normalsB[i]), sinf(normalsB[i]) };
        float minA = FLT_MAX, maxA = -FLT_MAX;
        for (int v = 0; v < countA; v++) {
            Vector2 wv = TransformPoint(vertsA[v], aAngle);
            wv.x += aX; wv.y += aY;
            float proj = PR_Vec2_Dot(wv, axis);
            if (proj < minA) minA = proj;
            if (proj > maxA) maxA = proj;
        }
        float minB = FLT_MAX, maxB = -FLT_MAX;
        for (int v = 0; v < countB; v++) {
            Vector2 wv = TransformPoint(vertsB[v], bAngle);
            wv.x += bX; wv.y += bY;
            float proj = PR_Vec2_Dot(wv, axis);
            if (proj < minB) minB = proj;
            if (proj > maxB) maxB = proj;
        }
        if (maxA < minB || maxB < minA) return false;
        float overlap = PR_MIN(maxA, maxB) - PR_MAX(minA, minB);
        if (overlap < minOverlap) {
            minOverlap = overlap;
            collisionNormal = axis;
            bestFromA = false;
            bestIndex = i;
        }
    }

    Vector2 dir = { bX - aX, bY - aY };
    if (PR_Vec2_Dot(collisionNormal, dir) < 0) {
        collisionNormal.x = -collisionNormal.x;
        collisionNormal.y = -collisionNormal.y;
    }

    const Vector2* refVerts;
    const Vector2* incVerts;
    float refX, refY, refAngle;
    float incX, incY, incAngle;
    int refCount, incCount;
    int refEdgeIdx;

    if (bestFromA) {
        refVerts = vertsA;
        incVerts = vertsB;
        refX = aX; refY = aY; refAngle = aAngle;
        incX = bX; incY = bY; incAngle = bAngle;
        refCount = countA; incCount = countB;
        refEdgeIdx = bestIndex;
    } else {
        refVerts = vertsB;
        incVerts = vertsA;
        refX = bX; refY = bY; refAngle = bAngle;
        incX = aX; incY = aY; incAngle = aAngle;
        refCount = countB; incCount = countA;
        refEdgeIdx = bestIndex;
    }

    Vector2 v1 = TransformPoint(refVerts[refEdgeIdx], refAngle);
    v1.x += refX; v1.y += refY;
    Vector2 v2 = TransformPoint(refVerts[(refEdgeIdx + 1) % refCount], refAngle);
    v2.x += refX; v2.y += refY;

    Vector2 edgeDir;
    PR_Vec2_Sub(&edgeDir, v2, v1);
    PR_Vec2_Normalize(&edgeDir);

    Vector2 incWorld[64];
    for (int i = 0; i < incCount; i++) {
        incWorld[i] = TransformPoint(incVerts[i], incAngle);
        incWorld[i].x += incX;
        incWorld[i].y += incY;
    }

    float refFaceOffset = PR_Vec2_Dot(v1, collisionNormal);
    Vector2 clipped1[64];
    int c1 = 0;
    for (int i = 0; i < incCount; i++) {
        int j = (i + 1) % incCount;
        Vector2 p1 = incWorld[i];
        Vector2 p2 = incWorld[j];
        float d1 = PR_Vec2_Dot(p1, collisionNormal) - refFaceOffset;
        float d2 = PR_Vec2_Dot(p2, collisionNormal) - refFaceOffset;
        if (d1 <= 0) {
            clipped1[c1++] = p1;
        }
        if (d1 * d2 < 0) {
            float t = d1 / (d1 - d2);
            Vector2 ip = { p1.x + t * (p2.x - p1.x), p1.y + t * (p2.y - p1.y) };
            clipped1[c1++] = ip;
        }
    }
    if (c1 == 0) return false;

    Vector2 clipped2[64];
    int c2 = 0;
    float v1proj = PR_Vec2_Dot(v1, edgeDir);
    for (int i = 0; i < c1; i++) {
        int j = (i + 1) % c1;
        Vector2 p1 = clipped1[i];
        Vector2 p2 = clipped1[j];
        float d1 = PR_Vec2_Dot(p1, edgeDir) - v1proj;
        float d2 = PR_Vec2_Dot(p2, edgeDir) - v1proj;
        if (d1 >= 0) {
            clipped2[c2++] = p1;
        }
        if (d1 * d2 < 0) {
            float t = d1 / (d1 - d2);
            Vector2 ip = { p1.x + t * (p2.x - p1.x), p1.y + t * (p2.y - p1.y) };
            clipped2[c2++] = ip;
        }
    }
    if (c2 == 0) return false;

    Vector2 contacts[2];
    int contactCount = 0;
    float v2proj = PR_Vec2_Dot(v2, edgeDir);
    for (int i = 0; i < c2; i++) {
        int j = (i + 1) % c2;
        Vector2 p1 = clipped2[i];
        Vector2 p2 = clipped2[j];
        float d1 = v2proj - PR_Vec2_Dot(p1, edgeDir);
        float d2 = v2proj - PR_Vec2_Dot(p2, edgeDir);
        if (d1 >= 0) {
            if (contactCount < 2) contacts[contactCount++] = p1;
        }
        if (d1 * d2 < 0) {
            float t = d1 / (d1 - d2);
            Vector2 ip = { p1.x + t * (p2.x - p1.x), p1.y + t * (p2.y - p1.y) };
            if (contactCount < 2) contacts[contactCount++] = ip;
        }
    }
    if (contactCount == 0) return false;

    m->normal = collisionNormal;
    m->penetration = minOverlap;
    m->contactCount = contactCount;
    for (int i = 0; i < contactCount; i++) {
        m->contacts[i] = contacts[i];
    }

    return true;
}

bool Collision_TestShapes(const Shape* a, float aX, float aY, float aAngle,
                          const Shape* b, float bX, float bY, float bAngle,
                          Manifold* manifold) {
    if (!manifold) return false;

    manifold->bodyA = NULL;
    manifold->bodyB = NULL;
    manifold->contactCount = 0;
    manifold->penetration = 0.0f;
    manifold->normal = (Vector2){0, 0};

    if (!a || !b) return false;

    ShapeType typeA = a->type;
    ShapeType typeB = b->type;

    if (typeA == PR_SHAPE_CIRCLE && typeB == PR_SHAPE_CIRCLE) {
        return Collision_TestCircleCircle(a, aX, aY, aAngle, b, bX, bY, bAngle, manifold);
    }

    if (typeA == PR_SHAPE_CIRCLE && (typeB == PR_SHAPE_POLYGON || typeB == PR_SHAPE_RECTANGLE)) {
        return Collision_TestCirclePolygon(a, aX, aY, aAngle, b, bX, bY, bAngle, manifold);
    }
    if (typeB == PR_SHAPE_CIRCLE && (typeA == PR_SHAPE_POLYGON || typeA == PR_SHAPE_RECTANGLE)) {
        bool result = Collision_TestCirclePolygon(b, bX, bY, bAngle, a, aX, aY, aAngle, manifold);
        if (result) {
            manifold->normal.x = -manifold->normal.x;
            manifold->normal.y = -manifold->normal.y;
        }
        return result;
    }

    if ((typeA == PR_SHAPE_POLYGON || typeA == PR_SHAPE_RECTANGLE) &&
        (typeB == PR_SHAPE_POLYGON || typeB == PR_SHAPE_RECTANGLE)) {
        return Collision_TestPolygonPolygon(a, aX, aY, aAngle, b, bX, bY, bAngle, manifold);
    }

    return false;
}