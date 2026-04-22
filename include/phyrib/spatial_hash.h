#ifndef PHYRIB_SPATIAL_HASH_H
#define PHYRIB_SPATIAL_HASH_H

#include <stddef.h>
#include <stdint.h>

typedef struct RigidBody RigidBody;

typedef struct AABB {
    float min[3];
    float max[3];
} AABB;

typedef struct SpatialHashNode {
    uint64_t bodyID;
    struct SpatialHashNode* next;
} SpatialHashNode;

typedef struct SpatialHashCell {
    SpatialHashNode* head;
} SpatialHashCell;

typedef struct SpatialHash {
    float cellSize;
    int32_t gridSize;
    SpatialHashCell* cells;
    uint64_t* bodyIndices;
    uint64_t bodyCount;
    uint64_t capacity;
} SpatialHash;

#ifdef __cplusplus
extern "C" {
#endif

void SpatialHashInit(SpatialHash* hash, float cellSize);
void SpatialHashClear(SpatialHash* hash);
void SpatialHashInsert(SpatialHash* hash, uint64_t bodyID, const AABB* aabb);
void SpatialHashInsertBody(SpatialHash* hash, RigidBody* body);
SpatialHashNode* SpatialHashQueryPoint(SpatialHash* hash, float x, float y, float z);
SpatialHashNode* SpatialHashQueryAABB(SpatialHash* hash, const AABB* aabb);
void SpatialHashRemove(SpatialHash* hash, uint64_t bodyID);

static inline int32_t SpatialHashGridKey(int32_t x, int32_t y, int32_t z, int32_t gridSize) {
    return ((x * 73856093) ^ (y * 19349663) ^ (z * 83492791)) % gridSize;
}

#ifdef __cplusplus
}
#endif

#endif // PHYRIB_SPATIAL_HASH_H