
add_executable (
    dipc_cpp
    DIPCSim.h
    DIPCSim.cpp
)
target_link_libraries(
    dipc_cpp
    ros2
)
install (
    TARGETS dipc_cpp
    RUNTIME DESTINATION bin
)
