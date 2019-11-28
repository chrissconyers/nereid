
add_library (
    dipc
    DIPC.h
    DIPC.cpp
)
target_include_directories (
    dipc
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries (
    dipc
    dynamics
)
