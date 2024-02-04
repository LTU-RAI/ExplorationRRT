cmake_minimum_required(VERSION 2.8)

# Project name
project(rrt)

# Add the executable
add_executable(optimizer example_optimizer.c)

# Add libraries to the executable
target_link_libraries(optimizer ${CMAKE_SOURCE_DIR}/target/release/librrt.a)
target_link_libraries(optimizer m)
target_link_libraries(optimizer dl)
target_link_libraries(optimizer pthread)

add_custom_target(run
    COMMAND optimizer
    DEPENDS optimizer
    WORKING_DIRECTORY ${CMAKE_PROJECT_DIR}
)