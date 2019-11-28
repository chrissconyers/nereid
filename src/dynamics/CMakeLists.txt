
find_package(Eigen3 REQUIRED)

add_library (
    dynamics
    dynamics_defs.h
    ODESystem.h
    rk4.h
    rk4.cpp
)
target_include_directories (
    dynamics
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
)
