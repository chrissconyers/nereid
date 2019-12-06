
add_executable (
    dipc_viz
    App.cpp
    DIPCViz.h
    DIPCViz.cpp
)
target_link_libraries (
    dipc_viz
    dipc
    fmt
    ros2
)
install (
    TARGETS dipc_viz
    RUNTIME DESTINATION bin
)
