cmake_minimum_required(VERSION 3.16)
project(Phyrib VERSION 1.0.0 LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Library sources
set(PHYRIB_SOURCES
    src/physics_world.c
    src/rigid_body.c
    src/shape.c
    src/collision_narrowphase.c
    src/dynamics.c
    src/spatial_hash.c
    src/debug_render.c
    src/constraint.c
    src/phyrib.c
)

# Library headers
set(PHYRIB_HEADERS
    include/phyrib/physics_world.h
    include/phyrib/rigid_body.h
    include/phyrib/shape.h
    include/phyrib/collision.h
    include/phyrib/types.h
    include/phyrib/debug_render.h
    include/phyrib/constraint.h
    include/phyrib/phyrib.h
)

# Create library
add_library(phyrib STATIC ${PHYRIB_SOURCES} ${PHYRIB_HEADERS})

target_include_directories(phyrib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link raylib if found
find_package(raylib QUIET)
if(raylib_FOUND)
    target_link_libraries(phyrib PUBLIC raylib)
    target_compile_definitions(phyrib PUBLIC PHYRIB_USE_RAYLIB=1)
else()
    message(STATUS "raylib not found - debug renderer will be disabled. Install raylib to enable visualization.")
endif()

# Examples
option(PHYRIB_BUILD_EXAMPLES "Build example programs" ON)
if(PHYRIB_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Tests
option(PHYRIB_BUILD_TESTS "Build test programs" ON)
if(PHYRIB_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# Install
include(GNUInstallDirs)
install(TARGETS phyrib
    EXPORT phyribTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
