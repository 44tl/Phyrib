#ifndef PHYRIB_SHAPE_H
#define PHYRIB_SHAPE_H

#include "phyrib/types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Shape Shape;

// Create shapes
Shape* Shape_CreateCircle(float radius);
Shape* Shape_CreateRectangle(float width, float height);
Shape* Shape_CreatePolygon(const Vector2* vertices, int count);
Shape* Shape_CreateCapsule(float radius, float height);
Shape* Shape_CreateEdge(Vector2 start, Vector2 end);
Shape* Shape_CreateBox(float width, float height, float angle); // Rotated rectangle

// Destroy shape
void Shape_Destroy(Shape* shape);

// Get shape information
ShapeType Shape_GetType(Shape* shape);
float Shape_GetArea(Shape* shape);
Vector2 Shape_GetCentroid(Shape* shape);
Vector2 Shape_GetSupportPoint(Shape* shape, Vector2 direction);
void Shape_ComputeAABB(Shape* shape, float posX, float posY, float angle, Vector2* min, Vector2* max);
void Shape_Clone(Shape* dest, const Shape* src); // Copy data

// Mass computation
float Shape_ComputeMass(Shape* shape, float density, float* outInertia);

// Utility
bool Shape_PointInside(const Shape* shape, Vector2 point, float posX, float posY, float angle);
bool Shape_Raycast(const Shape* shape, Vector2 origin, Vector2 direction, float maxDist, float* outDist, Vector2* outNormal);

#ifdef __cplusplus
}
#endif

#endif // PHYRIB_SHAPE_H
