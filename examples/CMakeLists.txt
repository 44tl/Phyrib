cmake_minimum_required(VERSION 3.16)
project(PhyribExamples)

# Build demo
add_executable(phyrib_demo demo.c)
target_link_libraries(phyrib_demo PRIVATE phyrib)

# If raylib found above, link it (already in phyrib)
if(raylib_FOUND)
    # phyrib already links raylib transitively
endif()

# Install demo (optional)
install(TARGETS phyrib_demo RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
