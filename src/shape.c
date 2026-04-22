#include "phyrib/internal.h"
#include "phyrib/shape.h"
#include <string.h>

struct Shape {
    ShapeType type;
    float radius;      // for circle/capsule
    float width, height;
    Vector2* vertices; // for polygon
    int vertexCount;
    float* normals;    // precomputed normals
    float inertia;     // precomputed inertia
    float area;        // precomputed area
    Vector2 centroid;  // precomputed centroid
};

Shape* Shape_CreateCircle(float radius) {
    if (radius <= 0) return NULL;
    Shape* s = (Shape*)PR_AllocZeroed(sizeof(Shape));
    s->type = PR_SHAPE_CIRCLE;
    s->radius = radius;
    s->area = M_PI * radius * radius;
    s->centroid = (Vector2){0, 0};
    s->inertia = Body_ComputeInertiaForCircle(1.0f, radius);
    return s;
}

Shape* Shape_CreateRectangle(float width, float height) {
    if (width <= 0 || height <= 0) return NULL;
    Shape* s = (Shape*)PR_AllocZeroed(sizeof(Shape));
    s->type = PR_SHAPE_RECTANGLE;
    s->width = width;
    s->height = height;
    s->area = width * height;
    s->centroid = (Vector2){0, 0};
    s->inertia = Body_ComputeInertiaForBox(1.0f, width, height);
    return s;
}

Shape* Shape_CreatePolygon(const Vector2* vertices, int count) {
    if (count < 3 || count > 64) return NULL;
    Shape* s = (Shape*)PR_AllocZeroed(sizeof(Shape));
    s->type = PR_SHAPE_POLYGON;
    s->vertexCount = count;
    s->vertices = (Vector2*)PR_Alloc(sizeof(Vector2) * count);
    s->normals = (float*)PR_Alloc(sizeof(float) * count);
    memcpy(s->vertices, vertices, sizeof(Vector2) * count);

    // Compute centroid
    float cx = 0, cy = 0;
    float area = 0;
    for (int i = 0; i < count; i++) {
        int j = (i + 1) % count;
        float cross = vertices[i].x * vertices[j].y - vertices[j].x * vertices[i].y;
        area += cross;
        cx += (vertices[i].x + vertices[j].x) * cross;
        cy += (vertices[i].y + vertices[j].y) * cross;
    }
    area *= 0.5f;
    if (area != 0) {
        cx /= (6.0f * area);
        cy /= (6.0f * area);
    }
    s->centroid = (Vector2){cx, cy};
    s->area = PR_ABS(area);

    // Compute normals (pointing outward) and inertia
    float inertia = 0;
    for (int i = 0; i < count; i++) {
        int j = (i + 1) % count;
        Vector2 e = { vertices[j].x - vertices[i].x, vertices[j].y - vertices[i].y };
        Vector2 normal = { e.y, -e.x };
        float len = PR_Vec2_Length(normal);
        if (len > 0) { normal.x /= len; normal.y /= len; }
        s->normals[i] = atan2f(normal.y, normal.x); // Store normal angle

        // Accumulate inertia (simplified)
        float k = PR_ABS(vertices[i].x * vertices[j].y - vertices[j].x * vertices[i].y);
        inertia += k * (PR_SQR(vertices[i].x) + vertices[i].x*vertices[j].x + PR_SQR(vertices[j].x) +
                       PR_SQR(vertices[i].y) + vertices[i].y*vertices[j].y + PR_SQR(vertices[j].y));
    }
    s->inertia = inertia / 12.0f;

    return s;
}

Shape* Shape_CreateCapsule(float radius, float height) {
    if (radius <= 0 || height <= 0) return NULL;
    Shape* s = (Shape*)PR_AllocZeroed(sizeof(Shape));
    s->type = PR_SHAPE_CAPSULE;
    s->radius = radius;
    s->height = height; // Half-height of cylindrical section
    s->area = 2.0f * M_PI * radius * (2.0f * radius + height);
    s->centroid = (Vector2){0, 0};
    s->inertia = Body_ComputeInertiaForCapsule(1.0f, radius, height * 2.0f);
    return s;
}

Shape* Shape_CreateEdge(Vector2 start, Vector2 end) {
    Shape* s = (Shape*)PR_AllocZeroed(sizeof(Shape));
    s->type = PR_SHAPE_EDGE;
    s->vertices = (Vector2*)PR_Alloc(sizeof(Vector2) * 2);
    s->vertices[0] = start;
    s->vertices[1] = end;
    s->vertexCount = 2;
    s->area = 0; // Edge has no area
    s->centroid = (Vector2){(start.x + end.x) * 0.5f, (start.y + end.y) * 0.5f};
    return s;
}

Shape* Shape_CreateBox(float width, float height, float angle) {
    // Creates a rotated rectangle as a 4-vertex polygon
    float hw = width * 0.5f;
    float hh = height * 0.5f;
    Vector2 verts[4];
    verts[0] = (Vector2){ hw, hh};
    verts[1] = (Vector2){-hw, hh};
    verts[2] = (Vector2){-hw, -hh};
    verts[3] = (Vector2){ hw, -hh};

    if (fabsf(angle) > 0.0001f) {
        for (int i = 0; i < 4; i++) {
            PR_Vec2_Rotate(&verts[i], verts[i], angle);
        }
    }

    return Shape_CreatePolygon(verts, 4);
}

void Shape_Destroy(Shape* shape) {
    if (!shape) return;
    if (shape->vertices) PR_Free(shape->vertices);
    if (shape->normals) PR_Free(shape->normals);
    PR_Free(shape);
}

ShapeType Shape_GetType(Shape* shape) { return shape ? shape->type : PR_SHAPE_CIRCLE; }
float Shape_GetArea(Shape* shape) { return shape ? shape->area : 0; }
Vector2 Shape_GetCentroid(Shape* shape) { return shape ? shape->centroid : (Vector2){0,0}; }

Vector2 Shape_GetSupportPoint(const Shape* shape, Vector2 direction) {
    if (!shape) return (Vector2){0,0};

    float best = -FLT_MAX;
    Vector2 result = {0, 0};

    switch (shape->type) {
        case PR_SHAPE_CIRCLE: {
            Vector2 d = direction;
            PR_Vec2_Normalize(&d);
            result.x = shape->radius * d.x;
            result.y = shape->radius * d.y;
            break;
        }
        case PR_SHAPE_RECTANGLE: {
            float hw = shape->width * 0.5f;
            float hh = shape->height * 0.5f;
            result.x = (direction.x >= 0 ? hw : -hw);
            result.y = (direction.y >= 0 ? hh : -hh);
            break;
        }
        case PR_SHAPE_POLYGON: {
            for (int i = 0; i < shape->vertexCount; i++) {
                float proj = PR_Vec2_Dot(shape->vertices[i], direction);
                if (proj > best) {
                    best = proj;
                    result = shape->vertices[i];
                }
            }
            break;
        }
        case PR_SHAPE_CAPSULE: {
            // Support point for capsule: max(radius, half-length + radius)
            Vector2 d = direction;
            PR_Vec2_Normalize(&d);
            float halfH = shape->height;
            // Find farthest point along direction
            if (fabsf(d.x) > fabsf(d.y)) {
                result.x = (d.x >= 0 ? halfH : -halfH);
                result.y = 0;
            } else {
                result.y = (d.y >= 0 ? halfH : -halfH);
                result.x = 0;
            }
            // Add radius in direction
            if (direction.x != 0 || direction.y != 0) {
                Vector2 dirNorm = direction;
                PR_Vec2_Normalize(&dirNorm);
                result.x += shape->radius * dirNorm.x;
                result.y += shape->radius * dirNorm.y;
            }
            break;
        }
        default:
            break;
    }
    return result;
}

void Shape_ComputeAABB(const Shape* shape, float posX, float posY, float angle, Vector2* min, Vector2* max) {
    if (!shape || !min || !max) return;

    // Initialize to extreme values
    min->x = FLT_MAX; min->y = FLT_MAX;
    max->x = -FLT_MAX; max->y = -FLT_MAX;

    float cosA = cosf(angle), sinA = sinf(angle);

    auto updateBounds = [&](float x, float y) {
        float rx = posX + x * cosA - y * sinA;
        float ry = posY + x * sinA + y * cosA;
        if (rx < min->x) min->x = rx;
        if (ry < min->y) min->y = ry;
        if (rx > max->x) max->x = rx;
        if (ry > max->y) max->y = ry;
    };

    switch (shape->type) {
        case PR_SHAPE_CIRCLE: {
            updateBounds(shape->radius, 0);
            updateBounds(-shape->radius, 0);
            updateBounds(0, shape->radius);
            updateBounds(0, -shape->radius);
            break;
        }
        case PR_SHAPE_RECTANGLE: {
            float hw = shape->width * 0.5f;
            float hh = shape->height * 0.5f;
            updateBounds(hw, hh);
            updateBounds(-hw, hh);
            updateBounds(-hw, -hh);
            updateBounds(hw, -hh);
            break;
        }
        case PR_SHAPE_POLYGON: {
            for (int i = 0; i < shape->vertexCount; i++) {
                updateBounds(shape->vertices[i].x, shape->vertices[i].y);
            }
            break;
        }
        case PR_SHAPE_CAPSULE: {
            float halfH = shape->height;
            // Cylinder body
            updateBounds(halfH, shape->radius);
            updateBounds(halfH, -shape->radius);
            updateBounds(-halfH, shape->radius);
            updateBounds(-halfH, -shape->radius);
            // Hemispherical caps
            updateBounds(-halfH + shape->radius, shape->radius);
            updateBounds(halfH - shape->radius, shape->radius);
            updateBounds(-halfH + shape->radius, -shape->radius);
            updateBounds(halfH - shape->radius, -shape->radius);
            break;
        }
        case PR_SHAPE_EDGE: {
            if (shape->vertexCount >= 2) {
                updateBounds(shape->vertices[0].x, shape->vertices[0].y);
                updateBounds(shape->vertices[1].x, shape->vertices[1].y);
            }
            break;
        }
    }

    // Expand by small epsilon to avoid floating-point issues
    float eps = 0.001f;
    min->x -= eps; min->y -= eps;
    max->x += eps; max->y += eps;
}

void Shape_Clone(Shape* dest, const Shape* src) {
    if (!dest || !src) return;
    dest->type = src->type;
    dest->width = src->width;
    dest->height = src->height;
    dest->radius = src->radius;
    dest->area = src->area;
    dest->centroid = src->centroid;
    dest->inertia = src->inertia;

    if (src->vertices) {
        dest->vertexCount = src->vertexCount;
        dest->vertices = (Vector2*)PR_Alloc(sizeof(Vector2) * dest->vertexCount);
        memcpy(dest->vertices, src->vertices, sizeof(Vector2) * dest->vertexCount);
    }
    if (src->normals) {
        dest->normals = (float*)PR_Alloc(sizeof(float) * dest->vertexCount);
        memcpy(dest->normals, src->normals, sizeof(float) * dest->vertexCount);
    }
}

float Shape_ComputeMass(const Shape* shape, float density, float* outInertia) {
    if (!shape) return 0;
    float mass = shape->area * density;
    if (outInertia) {
        *outInertia = shape->inertia * density + mass * PR_SQR(0.1f); // Add small radius for numerical stability
    }
    return mass;
}

bool Shape_PointInside(const Shape* shape, Vector2 point, float posX, float posY, float angle) {
    if (!shape) return false;

    // Transform point to local space
    Vector2 local = point;
    local.x -= posX; local.y -= posY;
    if (fabsf(angle) > 0.0001f) {
        float cosA = cosf(-angle), sinA = sinf(-angle);
        float tempX = local.x * cosA - local.y * sinA;
        float tempY = local.x * sinA + local.y * cosA;
        local.x = tempX; local.y = tempY;
    }

    switch (shape->type) {
        case PR_SHAPE_CIRCLE:
            return PR_Vec2_LengthSqr(local) <= shape->radius * shape->radius;
        case PR_SHAPE_RECTANGLE: {
            float hw = shape->width * 0.5f;
            float hh = shape->height * 0.5f;
            return PR_ABS(local.x) <= hw && PR_ABS(local.y) <= hh;
        }
        case PR_SHAPE_POLYGON: {
            // Point-in-polygon using ray casting
            bool inside = false;
            for (int i = 0, j = shape->vertexCount - 1; i < shape->vertexCount; j = i++) {
                const Vector2* vi = &shape->vertices[i];
                const Vector2* vj = &shape->vertices[j];
                if (((vi->y > local.y) != (vj->y > local.y)) &&
                    (local.x < (vj->x - vi->x) * (local.y - vi->y) / (vj->y - vi->y) + vi->x)) {
                    inside = !inside;
                }
            }
            return inside;
        }
        default:
            return false;
    }
}

bool Shape_Raycast(const Shape* shape, Vector2 origin, Vector2 direction, float maxDist, float* outDist, Vector2* outNormal) {
    if (!shape) return false;
    if (maxDist <= 0) return false;

    // Simple ray-circle test for circle
    if (shape->type == PR_SHAPE_CIRCLE) {
        // TODO: Full implementation
        return false;
    }

    // Simplified rectangle raycast
    if (shape->type == PR_SHAPE_RECTANGLE) {
        // TODO: Full implementation
        return false;
    }

    return false; // Not implemented yet
}
