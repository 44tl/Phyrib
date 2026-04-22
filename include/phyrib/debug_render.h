#ifndef PHYRIB_DEBUG_RENDER_H
#define PHYRIB_DEBUG_RENDER_H

#include "phyrib/types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Debug draw settings
typedef struct {
    Color shapeColor;        // Color for shape wireframes
    Color aabbColor;         // Color for AABBs
    Color contactColor;      // Color for contact points
    Color staticColor;       // Color for static bodies
    Color dynamicColor;      // Color for dynamic bodies
    Color kinematicColor;    // Color for kinematic bodies
    Color sleepingColor;     // Color for sleeping bodies
    float lineThickness;     // Line thickness
    bool fillShapes;         // Fill shapes with transparent color
} DebugRenderConfig;

// Initialize debug renderer
void DebugRender_Init(PhysicsWorld* world);
void DebugRender_SetConfig(const DebugRenderConfig* config);
void DebugRender_GetConfig(DebugRenderConfig* config);

// Drawing (requires raylib)
void DebugRender_DrawWorld(PhysicsWorld* world, int flags);
void DebugRender_DrawBody(RigidBody* body, int flags);
void DebugRender_DrawShape(const Shape* shape, float x, float y, float angle, Color color);
void DebugRender_DrawAABB(Vector2 min, Vector2 max, Color color);
void DebugRender_DrawContact(Vector2 point, Vector2 normal, Color color);
void DebugRender_DrawVelocity(Vector2 pos, Vector2 vel, Color color);
void DebugRender_DrawText(const char* text, Vector2 pos, Color color);

// Cleanup
void DebugRender_Cleanup(void);

#ifdef __cplusplus
}
#endif

#endif // PHYRIB_DEBUG_RENDER_H
