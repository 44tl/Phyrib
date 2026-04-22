cmake_minimum_required(VERSION 3.16)
project(PhyribTests)

# Simple test framework
add_executable(phyrib_tests
    test_shapes.c
    test_rigidbody.c
    test_world.c
)
target_link_libraries(phyrib_tests PRIVATE phyrib)

# Register tests
enable_testing()
add_test(NAME Phyrib_Shapes COMMAND phyrib_tests --shapes)
add_test(NAME Phyrib_RigidBodies COMMAND phyrib_tests --rigidbodies)
add_test(NAME Phyrib_World COMMAND phyrib_tests --world)
