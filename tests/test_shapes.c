// Basic shape tests
#include "phyrib/phyrib.h"
#include <stdio.h>
#include <assert.h>

int main(int argc, char** argv) {
    if (argc < 2) {
        printf("Usage: %s [--shapes|--rigidbodies|--world]\n", argv[0]);
        return 1;
    }

    if (strcmp(argv[1], "--shapes") == 0) {
        printf("Testing shapes...\n");

        // Circle
        Shape circle = Shape_CreateCircle(50);
        assert(circle);
        assert(Shape_GetType(&circle) == PR_SHAPE_CIRCLE);
        float area = Shape_GetArea(&circle);
        assert(area > 0);
        printf("  Circle area: %.2f (expected ~7854)\n", area);
        Shape_Destroy(&circle);

        // Rectangle
        Shape rect = Shape_CreateRectangle(100, 50);
        assert(rect);
        assert(Shape_GetType(&rect) == PR_SHAPE_RECTANGLE);
        area = Shape_GetArea(&rect);
        assert(area == 5000.0f);
        printf("  Rectangle area: %.2f (expected 5000)\n", area);
        Shape_Destroy(&rect);

        // Polygon (pentagon)
        Vector2 pentagon[5];
        for (int i = 0; i < 5; i++) {
            float angle = (i * 72.0f) * PR_DEG2RAD(72);
            pentagon[i] = (Vector2){50 * cosf(angle), 50 * sinf(angle)};
        }
        Shape poly = Shape_CreatePolygon(pentagon, 5);
        assert(poly);
        assert(Shape_GetType(&poly) == PR_SHAPE_POLYGON);
        area = Shape_GetArea(&poly);
        printf("  Pentagonal area: %.2f\n", area);
        Shape_Destroy(&poly);

        printf("All shape tests passed!\n");
        return 0;
    }

    printf("Test module not fully implemented.\n");
    return 0;
}
