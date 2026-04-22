#ifndef PHYRIB_DEBUG_RENDER_H
#define PHYRIB_DEBUG_RENDER_H

#include "phyrib/types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    Color shapeColor;
    Color aabbColor;
    Color contactColor;
    Color staticColor;
    Color dynamicColor;
    Color kinematicColor;
    Color sleepingColor;
    float lineThickness;
    bool fillShapes;
} DebugRenderConfig;

void DebugRender_Init(PhysicsWorld* world);
void DebugRender_SetConfig(const DebugRenderConfig* config);
void DebugRender_GetConfig(DebugRenderConfig* config);

void DebugRender_DrawWorld(PhysicsWorld* world, int flags);
void DebugRender_DrawBody(RigidBody* body, int flags);
void DebugRender_DrawShape(const Shape* shape, float x, float y, float angle, Color color);
void DebugRender_DrawAABB(Vector2 min, Vector2 max, Color color);
void DebugRender_DrawContact(Vector2 point, Vector2 normal, Color color);
void DebugRender_DrawVelocity(Vector2 pos, Vector2 vel, Color color);
void DebugRender_DrawText(const char* text, Vector2 pos, Color color);

void DebugRender_Cleanup(void);

#ifdef __cplusplus
}
#endif

#endif // PHYRIB_DEBUG_RENDER_H
