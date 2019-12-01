
#include "DIPCViz.h"

#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <fmt/core.h>
#include <chrono>
using namespace std::chrono_literals;


namespace nereid
{

class DIPCViz::PrivateImpl
{
public:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub;

    void updateState(const std_msgs::msg::String::SharedPtr msg);
    void iterate(void);
};

DIPCViz::DIPCViz(void)
: rclcpp::Node("dipc_viz")
, impl_(std::make_unique<PrivateImpl>())
{
    impl_->marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("splurp_marker", 10);
    impl_->timer = this->create_wall_timer(500ms, [&](void){impl_->iterate();});

    // a test subscriber
    auto callback =
    [this](const std_msgs::msg::String::SharedPtr msg) -> void
    {
        std::cout << "I heard: [" << msg->data.c_str() << "]" << std::endl;
    };
    impl_->state_sub = this->create_subscription<std_msgs::msg::String>("dipc_state", 10,
        [&](const std_msgs::msg::String::SharedPtr msg){impl_->updateState(msg);} );
}

DIPCViz::~DIPCViz(void)
{
}

void DIPCViz::PrivateImpl::updateState(const std_msgs::msg::String::SharedPtr msg)
{
    fmt::print("state recvd: {}\n", msg->data);

    // compute tfs
    // send tfs
    // (re-)send markers?
}

void DIPCViz::PrivateImpl::iterate(void)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    //marker.lifetime = rclcpp::Duration();
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = 1.0;
    marker.color.a = 1.0;

    marker_pub->publish(marker);
}

}
