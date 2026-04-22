#include "phyrib/internal.h"
#include <stdio.h>

#if defined(PHYRIB_USE_RAYLIB) && PHYRIB_USE_RAYLIB
#include "raylib.h"
#endif

DebugRenderState gDebugState;

void DebugRender_Init(PhysicsWorld* world) {
    gDebugState.shape = (Color){255, 255, 255, 255};
    gDebugState.aabb = (Color){255, 0, 0, 255};
    gDebugState.contact = (Color){0, 255, 0, 255};
    gDebugState.static_ = (Color){0, 255, 0, 255};
    gDebugState.dynamic = (Color){0, 0, 255, 255};
    gDebugState.kinematic = (Color){255, 255, 0, 255};
    gDebugState.sleeping = (Color){128, 128, 128, 255};
    gDebugState.lineThickness = 1.0f;
    gDebugState.fillShapes = false;
    (void)world;
}

void DebugRender_SetConfig(const DebugRenderConfig* config) {
    if (!config) return;
    gDebugState.shape = config->shapeColor;
    gDebugState.aabb = config->aabbColor;
    gDebugState.contact = config->contactColor;
    gDebugState.static_ = config->staticColor;
    gDebugState.dynamic = config->dynamicColor;
    gDebugState.kinematic = config->kinematicColor;
    gDebugState.sleeping = config->sleepingColor;
    gDebugState.lineThickness = config->lineThickness;
    gDebugState.fillShapes = config->fillShapes;
}

void DebugRender_GetConfig(DebugRenderConfig* config) {
    if (!config) return;
    config->shapeColor = gDebugState.shape;
    config->aabbColor = gDebugState.aabb;
    config->contactColor = gDebugState.contact;
    config->staticColor = gDebugState.static_;
    config->dynamicColor = gDebugState.dynamic;
    config->kinematicColor = gDebugState.kinematic;
    config->sleepingColor = gDebugState.sleeping;
    config->lineThickness = gDebugState.lineThickness;
    config->fillShapes = gDebugState.fillShapes;
}

void DebugRender_Cleanup(void) {
}

static Color GetBodyColor(RigidBody* body) {
    if (!RigidBody_IsAwake(body)) {
        return gDebugState.sleeping;
    }
    switch (RigidBody_GetType(body)) {
        case PR_BODY_STATIC:   return gDebugState.static_;
        case PR_BODY_DYNAMIC:  return gDebugState.dynamic;
        case PR_BODY_KINEMATIC:return gDebugState.kinematic;
        default:               return gDebugState.dynamic;
    }
}

static inline void TransformPoint(Vector2* out, Vector2 local, float x, float y, float angle) {
    PR_Vec2_Rotate(out, &local, angle);
    out->x += x;
    out->y += y;
}

// Drawing implementation when raylib is available
#if defined(PHYRIB_USE_RAYLIB) && PHYRIB_USE_RAYLIB

void DebugRender_DrawWorld(PhysicsWorld* world, int flags) {
    if (!world) return;

    Vector2 min = { -1000000.0f, -1000000.0f };
    Vector2 max = {  1000000.0f,  1000000.0f };
    int count = 0;
    RigidBody** bodies = PhysicsWorld_QueryAABB(world, min, max, &count);
    if (!bodies) return;

    for (int i = 0; i < count; i++) {
        DebugRender_DrawBody(bodies[i], flags);
    }
}

void DebugRender_DrawBody(RigidBody* body, int flags) {
    if (!body) return;

    Color bodyColor = GetBodyColor(body);
    Vector2 pos = RigidBody_GetPosition(body);
    float angle = RigidBody_GetAngle(body);
    Shape* shape = RigidBody_GetShape(body);

    if ((flags & PR_DEBUG_SHAPES) && shape) {
        DebugRender_DrawShape(shape, pos.x, pos.y, angle, bodyColor);
    }

    if (flags & PR_DEBUG_AABB) {
        Vector2 min, max;
        RigidBody_GetAABB(body, &min, &max);
        DebugRender_DrawAABB(min, max, gDebugState.aabb);
    }

    if (flags & PR_DEBUG_VELOCITY) {
        Vector2 vel = RigidBody_GetLinearVelocity(body);
        DebugRender_DrawVelocity(pos, vel, bodyColor);
    }

    if (flags & PR_DEBUG_IDS) {
        char buf[64];
        const char* name = RigidBody_GetName(body);
        PhysicsBodyID id = RigidBody_GetID(body);
        if (name && name[0]) {
            snprintf(buf, sizeof(buf), "%s (%u)", name, (unsigned)id);
        } else {
            snprintf(buf, sizeof(buf), "ID:%u", (unsigned)id);
        }
        DebugRender_DrawText(buf, pos, bodyColor);
    }
}

void DebugRender_DrawShape(const Shape* shape, float x, float y, float angle, Color color) {
    if (!shape) return;

    switch (Shape_GetType((Shape*)shape)) {
        case PR_SHAPE_CIRCLE: {
            float r = shape->data.circle.radius;
            DrawCircleLines(x, y, r, color);
            break;
        }
        case PR_SHAPE_RECTANGLE: {
            float hw = shape->data.rectangle.width * 0.5f;
            float hh = shape->data.rectangle.height * 0.5f;
            Vector2 local[4] = {
                {-hw, -hh}, {hw, -hh}, {hw, hh}, {-hw, hh}
            };
            Vector2 world[4];
            for (int i = 0; i < 4; i++) {
                TransformPoint(&world[i], local[i], x, y, angle);
            }
            DrawLineV(world[0], world[1], color);
            DrawLineV(world[1], world[2], color);
            DrawLineV(world[2], world[3], color);
            DrawLineV(world[3], world[0], color);
            break;
        }
        case PR_SHAPE_POLYGON: {
            const Vector2* verts = shape->data.polygon.vertices;
            int count = shape->data.polygon.count;
            if (count < 3) break;
            Vector2* world = (Vector2*)PR_Alloc(sizeof(Vector2) * count);
            for (int i = 0; i < count; i++) {
                TransformPoint(&world[i], verts[i], x, y, angle);
            }
            for (int i = 0; i < count; i++) {
                int j = (i + 1) % count;
                DrawLineV(world[i], world[j], color);
            }
            PR_Free(world);
            break;
        }
        case PR_SHAPE_CAPSULE: {
            float r = shape->data.capsule.radius;
            float h = shape->data.capsule.height;
            float halfH = h * 0.5f;

            Vector2 p1_local = {0.0f, -halfH};
            Vector2 p2_local = {0.0f,  halfH};
            Vector2 p1, p2;
            TransformPoint(&p1, p1_local, x, y, angle);
            TransformPoint(&p2, p2_local, x, y, angle);

            DrawCircleLines(p1.x, p1.y, r, color);
            DrawCircleLines(p2.x, p2.y, r, color);

            Vector2 corners_local[4] = {
                {-r, -halfH}, {r, -halfH}, {r, halfH}, {-r, halfH}
            };
            Vector2 corners[4];
            for (int i = 0; i < 4; i++) {
                TransformPoint(&corners[i], corners_local[i], x, y, angle);
            }
            DrawLineV(corners[0], corners[1], color);
            DrawLineV(corners[1], corners[2], color);
            DrawLineV(corners[2], corners[3], color);
            DrawLineV(corners[3], corners[0], color);
            break;
        }
        case PR_SHAPE_EDGE: {
            Vector2 start = shape->data.edge.start;
            Vector2 end = shape->data.edge.end;
            Vector2 ws, we;
            TransformPoint(&ws, start, x, y, angle);
            TransformPoint(&we, end, x, y, angle);
            DrawLineV(ws, we, color);
            break;
        }
        default:
            break;
    }
}

void DebugRender_DrawAABB(Vector2 min, Vector2 max, Color color) {
    DrawLineV((Vector2){min.x, min.y}, (Vector2){max.x, min.y}, color);
    DrawLineV((Vector2){max.x, min.y}, (Vector2){max.x, max.y}, color);
    DrawLineV((Vector2){max.x, max.y}, (Vector2){min.x, max.y}, color);
    DrawLineV((Vector2){min.x, max.y}, (Vector2){min.x, min.y}, color);
}

void DebugRender_DrawContact(Vector2 point, Vector2 normal, Color color) {
    DrawCircleLines(point.x, point.y, 3.0f, color);
    Vector2 end = { point.x + normal.x * 20.0f, point.y + normal.y * 20.0f };
    DrawLineV(point, end, color);
}

void DebugRender_DrawVelocity(Vector2 pos, Vector2 vel, Color color) {
    Vector2 end = { pos.x + vel.x, pos.y + vel.y };
    DrawLineV(pos, end, color);
}

void DebugRender_DrawText(const char* text, Vector2 pos, Color color) {
    if (!text) return;
    DrawText(text, (int)pos.x, (int)pos.y, 12, color);
}

#else // PHYRIB_USE_RAYLIB not defined: stub implementations

void DebugRender_DrawWorld(PhysicsWorld* world, int flags)          { (void)world; (void)flags; }
void DebugRender_DrawBody(RigidBody* body, int flags)               { (void)body; (void)flags; }
void DebugRender_DrawShape(const Shape* shape, float x, float y, float angle, Color color) { (void)shape; (void)x; (void)y; (void)angle; (void)color; }
void DebugRender_DrawAABB(Vector2 min, Vector2 max, Color color)    { (void)min; (void)max; (void)color; }
void DebugRender_DrawContact(Vector2 point, Vector2 normal, Color color) { (void)point; (void)normal; (void)color; }
void DebugRender_DrawVelocity(Vector2 pos, Vector2 vel, Color color){ (void)pos; (void)vel; (void)color; }
void DebugRender_DrawText(const char* text, Vector2 pos, Color color){ (void)text; (void)pos; (void)color; }

void DebugRender_Init(PhysicsWorld* world)                         { (void)world; }
void DebugRender_SetConfig(const DebugRenderConfig* config)        { (void)config; }
void DebugRender_GetConfig(DebugRenderConfig* config)              { (void)config; }
void DebugRender_Cleanup(void)                                     {}

#endif // PHYRIB_USE_RAYLIB
