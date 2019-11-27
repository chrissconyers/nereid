
add_executable (
    nereid
    main.cpp
)
target_include_directories(
    nereid
    PUBLIC
    ${ROS2_DIR}/include
)
target_link_directories(
    nereid
    PUBLIC
    ${ROS2_DIR}/lib
)
target_link_libraries(
    nereid
    rclcpp
    rcl
    rcutils
    tracetools
    std_msgs__rosidl_typesupport_cpp
)
install (
    TARGETS nereid
    RUNTIME DESTINATION bin
)