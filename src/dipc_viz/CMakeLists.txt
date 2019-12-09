
add_library (
    dipc_viz_node
    DIPCVizNode.h
    DIPCVizNode.cpp
)
target_link_libraries (
    dipc_viz_node
    dipc
    fmt
    ros2
)

add_executable (
    dipc_viz
    App.cpp
)
target_link_libraries (
    dipc_viz
    dipc_viz_node
)

install (
    TARGETS dipc_viz
    RUNTIME DESTINATION bin
)