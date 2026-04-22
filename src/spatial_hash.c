#include "phyrib/internal.h"
#include "phyrib/spatial_hash.h"
#include <stdlib.h>

typedef struct {
    BodyNode* head;
} HashBucket;

typedef struct {
    HashBucket buckets[PHYRIB_HASH_SIZE];
    int activeCount;
} SpatialHash;

static SpatialHash gHash;

void SpatialHash_Init(void) {
    memset(&gHash, 0, sizeof(gHash));
    gHash.activeCount = 0;
}

void SpatialHash_Clear(void) {
    for (int i = 0; i < PHYRIB_HASH_SIZE; i++) {
        BodyNode* node = gHash.buckets[i].head;
        while (node) {
            BodyNode* next = node->next;
            PR_Free(node);
            node = next;
        }
        gHash.buckets[i].head = NULL;
    }
    gHash.activeCount = 0;
}

static uint32_t HashVec2(Vector2 v, int cellSize) {
    int x = (int)floorf(v.x / cellSize);
    int y = (int)floorf(v.y / cellSize);
    uint32_t h = ((uint32_t)x * 73856093) ^ ((uint32_t)y * 19349663);
    return h % PHYRIB_HASH_SIZE;
}

static uint32_t HashAABB(Vector2 min, Vector2 max, int cellSize) {
    int x = (int)floorf((min.x + max.x) * 0.5f / cellSize);
    int y = (int)floorf((min.y + max.y) * 0.5f / cellSize);
    uint32_t h = ((uint32_t)x * 73856093) ^ ((uint32_t)y * 19349663);
    return h % PHYRIB_HASH_SIZE;
}

void SpatialHash_Insert(RigidBody* body, Vector2 min, Vector2 max, int cellSize) {
    uint32_t h = HashAABB(min, max, cellSize);
    BodyNode* node = (BodyNode*)PR_Alloc(sizeof(BodyNode));
    node->body = body;
    node->next = gHash.buckets[h].head;
    gHash.buckets[h].head = node;
    gHash.activeCount++;
}

BodyNode* SpatialHash_Query(Vector2 point, int cellSize) {
    uint32_t h = HashVec2(point, cellSize);
    return gHash.buckets[h].head;
}

BodyNode* SpatialHash_QueryAABB(Vector2 min, Vector2 max, int cellSize) {
    uint32_t h = HashAABB(min, max, cellSize);
    return gHash.buckets[h].head;
}

void SpatialHash_Remove(RigidBody* body, Vector2 min, Vector2 max, int cellSize) {
    uint32_t h = HashAABB(min, max, cellSize);
    BodyNode** curr = &gHash.buckets[h].head;
    while (*curr) {
        if ((*curr)->body == body) {
            BodyNode* tmp = *curr;
            *curr = (*curr)->next;
            PR_Free(tmp);
            gHash.activeCount--;
            return;
        }
        curr = &(*curr)->next;
    }
}
