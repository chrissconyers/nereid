
add_executable (
    dipc_sim
    App.cpp
    DIPCSim.h
    DIPCSim.cpp
    DIPCSimNode.h
    DIPCSimNode.cpp
)
target_link_libraries(
    dipc_sim
    dipc
    fmt
    ros2
)
install (
    TARGETS dipc_sim
    RUNTIME DESTINATION bin
)
