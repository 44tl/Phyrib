# Phyrib - Modern Physics for Raylib fr

A fast, easy-to-use 2D physics library built specifically for [raylib](https://www.raylib.com/). Phyrib provides rigid body dynamics, collision detection, and constraint solving with a clean C API.

## Features

- Simple, intuitive API - create physics worlds in minutes
- Rigid body dynamics (dynamic, static, kinematic)
- Multiple shape types: circle, rectangle, polygon, capsule, edge
- Continuous collision detection (CCD) support
- Accurate impulse-based collision response
- Friction and restitution with realistic material properties
- Constraint system: distance, pin (point-point), spring, weld joints
- Broad phase: spatial hashing for efficient nearby body queries
- Narrow phase: SAT (Separating Axis Theorem) for convex polyhedra
- Debug visualization integrated with raylib
- Collision filtering with category/mask/group system
- Sleep management for performance
- Ray casting and shape queries
- C11 codebase with C++ compatibility
- Cross-platform (Windows, Linux, macOS)

## Quick Start

```c
#include "phyrib/phyrib.h"
#include "raylib.h"

int main(void) {
    InitWindow(800, 600, "Phyrib Demo");
    SetTargetFPS(60);

    // 1. Create physics world
    WorldConfig config = PR_WORLD_DEFAULT;
    config.gravityY = 980.0f;  // Earth-like gravity
    PhysicsWorld* world = PhysicsWorld_Create(&config);

    // 2. Create a dynamic circle
    RigidBodyDef def = {0};
    def.type = PR_BODY_DYNAMIC;
    def.position = (Vector2){400, 100};
    Shape circle = Shape_CreateCircle(30);
    def.shape = &circle;
    def.material = (PhysicsMaterial){0.3f, 0.5f, 1.0f};  // friction, restitution, density
    RigidBody* ball = PhysicsWorld_CreateBody(world, &def);
    Shape_Destroy(circle);

    // 3. Create static ground
    RigidBodyDef groundDef = {0};
    groundDef.type = PR_BODY_STATIC;
    groundDef.position = (Vector2){400, 550};
    Shape ground = Shape_CreateRectangle(800, 60);
    groundDef.shape = &ground;
    PhysicsWorld_CreateBody(world, &groundDef);
    Shape_Destroy(ground);

    // 4. Main loop
    while (!WindowShouldClose()) {
        // Step physics (1/60 second timestep)
        PhysicsWorld_Step(world, 1.0f/60.0f);

        // Render
        BeginDrawing();
        ClearBackground(BLACK);
        PhysicsWorld_DrawDebug(world, PR_DEBUG_SHAPES);  // Wireframe overlay
        EndDrawing();
    }

    // Cleanup
    PhysicsWorld_Destroy(world);
    CloseWindow();
    return 0;
}
```

## Installation

### With CMake

```bash
git clone <repo-url>
cd phyrib
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --config Release
```

To build with raylib support:

```bash
cmake .. -DCMAKE_BUILD_TYPE=Release -Draylib_DIR=/path/to/raylib
```

Or let CMake find raylib automatically if installed system-wide.

### Manual

Copy the `include/phyrib` folder to your project's include path and compile all `.c` files from the `src` directory.

## API Overview

### Physics World

The `PhysicsWorld` is the central container that holds all bodies and steps the simulation.

```c
PhysicsWorld* PhysicsWorld_Create(const WorldConfig* config);
void PhysicsWorld_Destroy(PhysicsWorld* world);
void PhysicsWorld_Step(PhysicsWorld* world, float deltaTime);
void PhysicsWorld_AddBody(PhysicsWorld* world, RigidBody* body);
void PhysicsWorld_RemoveBody(PhysicsWorld* world, RigidBody* body);
RigidBody* PhysicsWorld_CreateBody(PhysicsWorld* world, const RigidBodyDef* def);
```

### Rigid Bodies

Bodies represent physical objects with mass, position, velocity, and shape.

```c
RigidBodyDef def = {0};
def.type = PR_BODY_DYNAMIC;              // or PR_BODY_STATIC, PR_BODY_KINEMATIC
def.position = (Vector2){100, 200};
def.angle = 0.0f;                         // radians
def.linearVelocity = (Vector2){0, 0};
def.angularVelocity = 0.0f;
def.linearDamping = 0.1f;                 // air resistance (0-1)
def.angularDamping = 0.1f;
def.material = PR_MATERIAL_DEFAULT;       // friction, restitution, density
def.shape = &myShape;

RigidBody* body = PhysicsWorld_CreateBody(world, &def);
```

Runtime control:
```c
Vector2 pos = RigidBody_GetPosition(body);
RigidBody_SetPosition(body, (Vector2){x, y});

RigidBody_ApplyForce(body, (Vector2){0, 980}, bodyPosition);
RigidBody_ApplyLinearImpulse(body, (Vector2){10, 0}, bodyPosition);
RigidBody_SetLinearVelocity(body, (Vector2){100, 0});

PhysicsMaterial mat = RigidBody_GetMaterial(body);
RigidBody_SetMaterial(body, (PhysicsMaterial){0.9f, 0.1f, 1.0f}); // bouncy rubber
```

### Shapes

Phyrib supports several convex shapes:

```c
// Circle
Shape circle = Shape_CreateCircle(radius);

// Rectangle (axis-aligned)
Shape rect = Shape_CreateRectangle(width, height);

// Rotated rectangle
Shape box = Shape_CreateBox(width, height, angle); // angle in radians

// Polygon (convex, up to 64 vertices)
Vector2 vertices[] = { {...}, {...}, ... };
Shape poly = Shape_CreatePolygon(vertices, vertexCount);

// Capsule (rounded line segment)
Shape capsule = Shape_CreateCapsule(radius, height); // height = half-length

// Edge (line segment for terrain)
Shape edge = Shape_CreateEdge((Vector2){0,0}, (Vector2){100,0});
```

Shape properties:
```c
float area = Shape_GetArea(shape);
Vector2 centroid = Shape_GetCentroid(shape);
Shape_Destroy(shape);  // when done
```

### Materials

Control physical behavior through materials:

```c
// Built-in presets
PR_MATERIAL_DEFAULT  // f=0.3, r=0.2, d=1.0
PR_MATERIAL_ICE      // f=0.1, r=0.1, d=1.0
PR_MATERIAL_RUBBER   // f=0.9, r=0.8, d=1.0
PR_MATERIAL_BOUNCY   // f=0.2, r=0.95, d=1.0
PR_MATERIAL_HEAVY    // f=0.6, r=0.3, d=5.0
PR_MATERIAL_LIGHT    // f=0.2, r=0.1, d=0.1

// Custom
PhysicsMaterial mat = {0.5f, 0.7f, 2.0f};  // friction, restitution, density
```

- **friction**: 0.0 (ice) to 1.0 (sticky)
- **restitution**: 0.0 (no bounce) to 1.0 (perfect bounce)
- **density**: kg/m² - affects mass calculation

### Collision Filtering

Fine-tune which bodies collide:

```c
CollisionFilter filter = {0};
filter.categoryBits = 0x0001;   // This body is in category 1
filter.maskBits = 0x0002;       // Collides with category 2
filter.groupIndex = 0;          // 0 = default, >0 always collide, <0 never
RigidBody_SetCollisionFilter(body, filter);
```

### Debug Visualization

Render physics shapes wireframes using raylib:

```c
DebugRender_Init(world);

// Configure render style
DebugRenderConfig conf = {0};
DebugRender_GetConfig(&conf);
conf.shapeColor = (Color){255, 255, 255, 255};
conf.aabbColor = (Color){255, 0, 0, 100};
conf.lineThickness = 2.0f;
DebugRender_SetConfig(&conf);

// In render loop:
int flags = PR_DEBUG_SHAPES | PR_DEBUG_AABB | PR_DEBUG_VELOCITY | PR_DEBUG_IDS;
PhysicsWorld_DrawDebug(world, flags);
```

Available flags:
- `PR_DEBUG_SHAPES` - draw shape wireframes
- `PR_DEBUG_AABB` - draw bounding boxes
- `PR_DEBUG_CONTACTS` - draw contact points during collisions
- `PR_DEBUG_VELOCITY` - draw velocity vectors
- `PR_DEBUG_IDS` - display body names/IDs
- `PR_DEBUG_COLLISION_NORMAL` - show collision normals

### Queries

Find bodies at a point or region:

```c
// Point query
int count;
RigidBody** bodies = PhysicsWorld_QueryPoint(world, (Vector2){100, 200}, &count);
for (int i = 0; i < count; i++) {
    // bodies[i] is at that point
}
if (bodies) free(bodies);

// AABB query
RigidBody** bodies = PhysicsWorld_QueryAABB(world, (Vector2){minX, minY}, (Vector2){maxX, maxY}, &count);

// Ray cast
bool hit = PhysicsWorld_Raycast(world, origin, direction, maxDist, callback, userData);
// or use PhysicsWorld_QueryRay for simple point/body retrieval
```

### Constraints (Joints)

Connect two bodies together:

```c
ConstraintDef cd = {0};
cd.type = PR_CONSTRAINT_DISTANCE;
cd.bodyA = bodyA;
cd.bodyB = bodyB;
cd.localAnchorA = (Vector2){10, 0};  // local to bodyA
cd.localAnchorB = (Vector2){-10, 0}; // local to bodyB
cd.data.distance.distance = 50.0f;   // target distance

Constraint* constraint = Constraint_Create(&cd);
// Constraints are automatically managed by world in future (not yet - manual step)
Constraint_Solve(constraint, deltaTime);
Constraint_PostSolve(constraint);
```

Types:
- `PR_CONSTRAINT_DISTANCE` - maintain fixed distance
- `PR_CONSTRAINT_PIN` - point-to-point (ball joint)
- `PR_CONSTRAINT_SPRING` - soft spring with damping
- `PR_CONSTRAINT_WELD` - rigidly weld bodies together
- `PR_CONSTRAINT_MOTOR` - rotational/linear motor

### World Configuration

Tune performance and behavior:

```c
WorldConfig config = PR_WORLD_DEFAULT;
config.gravityX = 0;
config.gravityY = 980.0f;           // pixels/s²
config.velocityIterations = 8;      // solver quality (default 8)
config.positionIterations = 3;      // constraint iterations (default 3)
config.allowSleep = true;          // let inactive bodies sleep
config.sleepTimeThreshold = 0.5f;   // seconds before sleep
config.sleepVelocityThreshold = 10.0f;  // speed below which sleep
PhysicsWorld* world = PhysicsWorld_Create(&config);
```

## Architecture

```
Phyrib consists of:

1. types.h         - Core types, enums, material presets
2. shape.c/h       - Collision shape primitives
3. rigid_body.c/h  - Body transform, velocity, mass
4. spatial_hash.c  - Broad-phasing (broadphase)
5. collision_narrowphase.c - SAT/GJK collision detection
6. dynamics.c      - Impulse resolution, friction
7. physics_world.c - World simulation loop
8. constraint.c    - Joints, springs, motors
9. debug_render.c  - Raylib visualization

internal.h        - Private helpers, math, flags
```

## Performance

- **O(n log n)** broad phase via spatial hash grid
- **Convex only**: All shapes are convex for fast SAT testing
- **Cache-friendly**: Body arrays are contiguous memory
- **Sleep system**: Bodies stop simulating when nearly at rest
- **Configurable iterations**: Trade accuracy for speed

## Limitations

- 2D only (2D physics)
- Convex shapes only (no concave mesh support yet)
- No continuous collision detection (CCD) - tunneling possible at high speeds (use smaller timesteps or increase shape margins)
- No specialized circle-polygon optimizations yet (but SAT handles them fine)
- Constraint solver is sequential impulse (not Baumgarte stabilized for all types yet)

## Future Work I'll be doing

- Continuous Collision Detection (CCD) for fast objects
- Concave trimesh support (via decomposition into convex hulls)
- Swept collision tests
- More constraint types (revolute, prismatic, pulley, gear)
- Joint limits and motors (angular/linear)
- Enhanced warmstarting for faster convergence
- GPU-based broad phase (spatial partitioning acceleration)

## Licensing

Phyrib is released under the MIT License. See LICENSE file for details.

## Contributing

Closed just fork and dev ur self if I'm not updating this.

---

Built for game developers who want real physics without the complexity of Box2D.
