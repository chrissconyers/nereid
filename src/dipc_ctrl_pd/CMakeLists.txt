
add_library (
    dipc_ctrl_pd_node
    DIPCCtrlPDNode.h
    DIPCCtrlPDNode.cpp
)
target_link_libraries (
    dipc_ctrl_pd_node
    dipc
    ros2
)

add_executable (
    dipc_ctrl_pd
    App.cpp
)
target_link_libraries (
    dipc_ctrl_pd
    dipc_ctrl_pd_node
)
install (
    TARGETS dipc_ctrl_pd
    RUNTIME DESTINATION bin
)
