// Example: Phyrib Demo - Physics library showcase
// Build with: gcc demo.c -o demo -lphyrib -lraylib -lm

#include "phyrib/phyrib.h"
#include "raylib.h"
#include <stdio.h>

#define SCREEN_WIDTH 800
#define SCREEN_HEIGHT 600
#define FPS 60
#define DT (1.0f / FPS)

// Global physics world
static PhysicsWorld* world = NULL;

// Create ground and walls
static void CreateBoundaries(void) {
    // Ground (static rectangle)
    RigidBodyDef groundDef = {0};
    groundDef.type = PR_BODY_STATIC;
    groundDef.position = (Vector2){SCREEN_WIDTH * 0.5f, SCREEN_HEIGHT - 30};
    Shape groundShape = Shape_CreateRectangle(SCREEN_WIDTH - 40, 60);
    groundDef.shape = &groundShape;
    groundDef.material = PR_MATERIAL_RUBBER;
    PhysicsWorld_CreateBody(world, &groundDef);
    Shape_Destroy(groundShape);

    // Left wall
    RigidBodyDef leftWallDef = {0};
    leftWallDef.type = PR_BODY_STATIC;
    leftWallDef.position = (Vector2){20, SCREEN_HEIGHT * 0.5f};
    Shape leftShape = Shape_CreateRectangle(40, SCREEN_HEIGHT);
    leftWallDef.shape = &leftShape;
    PhysicsWorld_CreateBody(world, &leftShape);
    Shape_Destroy(leftShape);

    // Right wall
    RigidBodyDef rightWallDef = {0};
    rightWallDef.type = PR_BODY_STATIC;
    rightWallDef.position = (Vector2){SCREEN_WIDTH - 20, SCREEN_HEIGHT * 0.5f};
    Shape rightShape = Shape_CreateRectangle(40, SCREEN_HEIGHT);
    rightWallDef.shape = &rightShape;
    PhysicsWorld_CreateBody(world, &rightShape);
    Shape_Destroy(rightShape);
}

// Create a stack of boxes
static void CreateBoxStack(int count, float startX, float startY, float size) {
    for (int i = 0; i < count; i++) {
        RigidBodyDef def = {0};
        def.type = PR_BODY_DYNAMIC;
        def.position = (Vector2){startX, startY - i * (size + 2)};
        Shape box = Shape_CreateRectangle(size, size);
        def.shape = &box;
        def.material = PR_MATERIAL_DEFAULT;
        def.linearDamping = 0.1f;
        def.angularDamping = 0.1f;
        PhysicsWorld_CreateBody(world, &def);
        Shape_Destroy(box);
    }
}

// Create a pyramid of circles
static void CreateCirclePyramid(int rows, float startX, float startY, float radius) {
    for (int row = 0; row < rows; row++) {
        int circlesInRow = row + 1;
        float rowWidth = circlesInRow * (radius * 2 + 4);
        float rowStartX = startX - rowWidth * 0.5f + radius;

        for (int col = 0; col < circlesInRow; col++) {
            RigidBodyDef def = {0};
            def.type = PR_BODY_DYNAMIC;
            def.position = (Vector2){rowStartX + col * (radius * 2 + 4), startY - row * (radius * 2 + 2)};
            Shape circle = Shape_CreateCircle(radius);
            def.shape = &circle;
            def.material = PR_MATERIAL_DEFAULT;
            PhysicsWorld_CreateBody(world, &def);
            Shape_Destroy(circle);
        }
    }
}

// Create a kinematic moving platform
static RigidBody* CreateMovingPlatform(void) {
    RigidBodyDef def = {0};
    def.type = PR_BODY_KINEMATIC;
    def.position = (Vector2){SCREEN_WIDTH * 0.5f, SCREEN_HEIGHT - 150};
    Shape plat = Shape_CreateRectangle(200, 30);
    def.shape = &plat;
    def.material = PR_MATERIAL_DEFAULT;
    RigidBody* platform = PhysicsWorld_CreateBody(world, &def);
    Shape_Destroy(plat);
    return platform;
}

// Random color helper
static Color RandomColor(void) {
    return (Color){
        GetRandomValue(100, 255),
        GetRandomValue(100, 255),
        GetRandomValue(100, 255),
        255
    };
}

int main(void) {
    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Phyrib - Modern Physics for Raylib");
    SetTargetFPS(FPS);

    // 1. Initialize physics world
    WorldConfig config = PR_WORLD_DEFAULT;
    config.gravityY = 980.0f;
    config.velocityIterations = 8;
    config.positionIterations = 3;
    world = PhysicsWorld_Create(&config);

    // 2. Setup debug rendering
    DebugRender_Init(world);
    DebugRenderConfig debugConf = {0};
    DebugRender_GetConfig(&debugConf);
    debugConf.shapeColor = (Color){255, 255, 255, 255};
    debugConf.aabbColor = (Color){255, 0, 0, 100};
    debugConf.staticColor = (Color){0, 255, 0, 255};
    debugConf.dynamicColor = (Color){65, 105, 225, 255};
    debugConf.kinematicColor = (Color){255, 165, 0, 255};
    debugConf.sleepingColor = (Color){128, 128, 128, 255};
    debugConf.lineThickness = 2.0f;
    DebugRender_SetConfig(&debugConf);

    // 3. Create scene
    CreateBoundaries();
    CreateBoxStack(10, SCREEN_WIDTH * 0.35f, SCREEN_HEIGHT - 90, 40);
    CreateCirclePyramid(5, SCREEN_WIDTH * 0.65f, SCREEN_HEIGHT - 90, 18);

    // Create some random dynamic shapes
    for (int i = 0; i < 5; i++) {
        RigidBodyDef def = {0};
        def.type = PR_BODY_DYNAMIC;
        def.position = (Vector2){SCREEN_WIDTH * 0.5f + GetRandomValue(-100, 100), 200 + GetRandomValue(0, 80)};
        Shape* shapePtr = NULL;
        int shapeType = GetRandomValue(0, 2);
        if (shapeType == 0) {
            Shape circle = Shape_CreateCircle(GetRandomValue(15, 25));
            shapePtr = &circle;
        } else if (shapeType == 1) {
            Shape rect = Shape_CreateRectangle(GetRandomValue(30, 60), GetRandomValue(30, 60));
            shapePtr = &rect;
        } else {
            // Regular pentagon (approximate as polygon)
            Vector2 verts[5];
            float r = GetRandomValue(20, 30);
            for (int j = 0; j < 5; j++) {
                float ang = (j * 72.0f) * PR_DEG2RAD(72);
                verts[j] = (Vector2){r * cosf(ang), r * sinf(ang)};
            }
            Shape poly = Shape_CreatePolygon(verts, 5);
            shapePtr = &poly;
        }
        def.shape = shapePtr;
        def.material = (PhysicsMaterial){0.3f, 0.2f, 1.0f};
        PhysicsWorld_CreateBody(world, &def);
        if (shapePtr) Shape_Destroy(shapePtr);
    }

    // Kinematic moving platform
    RigidBody* platform = CreateMovingPlatform();
    float platformTimer = 0.0f;

    // 4. Main loop
    char stats[128];
    int debugFlags = PR_DEBUG_SHAPES | PR_DEBUG_AABB | PR_DEBUG_IDS | PR_DEBUG_VELOCITY;

    while (!WindowShouldClose()) {
        // Update kinematic platform
        platformTimer += DT;
        float platformX = SCREEN_WIDTH * 0.5f + sinf(platformTimer) * 250.0f;
        RigidBody_SetPosition(platform, (Vector2){platformX, SCREEN_HEIGHT - 150});

        // Step physics
        PhysicsWorld_Step(world, DT);

        // Render
        BeginDrawing();
        ClearBackground(BLACK);

        // Draw physics world debug overlay
        PhysicsWorld_DrawDebug(world, debugFlags);

        // Stats overlay
        snprintf(stats, sizeof(stats),
                "Bodies: %d | Contacts: %d | KE: %.1f | FPS: %d",
                PhysicsWorld_GetBodyCount(world),
                PhysicsWorld_GetContactCount(world),
                PhysicsWorld_GetKineticEnergy(world),
                GetFPS());
        DrawText(stats, 10, 10, 16, GREEN);

        DrawText("Phyrib Demo - Arrows to toggle debug, R to reset, ESC to quit", 10, SCREEN_HEIGHT - 20, 14, YELLOW);

        // Input handling
        if (IsKeyPressed(KEY_R)) {
            // Reset world
            PhysicsWorld_Destroy(world);
            world = PhysicsWorld_Create(&config);
            DebugRender_Init(world);
            CreateBoundaries();
            CreateBoxStack(10, SCREEN_WIDTH * 0.35f, SCREEN_HEIGHT - 90, 40);
            CreateCirclePyramid(5, SCREEN_WIDTH * 0.65f, SCREEN_HEIGHT - 90, 18);
            platform = CreateMovingPlatform();
            platformTimer = 0.0f;
            for (int i = 0; i < 5; i++) {
                RigidBodyDef def = {0};
                def.type = PR_BODY_DYNAMIC;
                def.position = (Vector2){SCREEN_WIDTH * 0.5f + GetRandomValue(-100, 100), 200 + GetRandomValue(0, 80)};
                Shape* shapePtr = NULL;
                int shapeType = GetRandomValue(0, 2);
                if (shapeType == 0) {
                    Shape circle = Shape_CreateCircle(GetRandomValue(15, 25));
                    shapePtr = &circle;
                } else if (shapeType == 1) {
                    Shape rect = Shape_CreateRectangle(GetRandomValue(30, 60), GetRandomValue(30, 60));
                    shapePtr = &rect;
                } else {
                    Vector2 verts[5];
                    float r = GetRandomValue(20, 30);
                    for (int j = 0; j < 5; j++) {
                        float ang = (j * 72.0f) * PR_DEG2RAD(72);
                        verts[j] = (Vector2){r * cosf(ang), r * sinf(ang)};
                    }
                    Shape poly = Shape_CreatePolygon(verts, 5);
                    shapePtr = &poly;
                }
                def.shape = shapePtr;
                PhysicsWorld_CreateBody(world, &def);
                if (shapePtr) Shape_Destroy(shapePtr);
            }
        }

        if (IsKeyPressed(KEY_ONE))   debugFlags ^= PR_DEBUG_SHAPES;
        if (IsKeyPressed(KEY_TWO))   debugFlags ^= PR_DEBUG_AABB;
        if (IsKeyPressed(KEY_THREE)) debugFlags ^= PR_DEBUG_VELOCITY;
        if (IsKeyPressed(KEY_FOUR))  debugFlags ^= PR_DEBUG_IDS;

        EndDrawing();
    }

    // Cleanup
    PhysicsWorld_Destroy(world);
    DebugRender_Cleanup();
    CloseWindow();

    return 0;
}
