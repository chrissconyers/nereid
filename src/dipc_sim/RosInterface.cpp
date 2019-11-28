
#include "RosInterface.h"

#include "std_msgs/msg/string.hpp"


namespace nereid
{
class RosInterface::PrivateImpl
{
public:

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr test_sub;

    PrivateImpl(void){}
    ~PrivateImpl(void){}
};

RosInterface::RosInterface(void)
: rclcpp::Node("dipc_sim")
, impl_(std::make_unique<PrivateImpl>())
{
    impl_->state_pub = this->create_publisher<std_msgs::msg::String>("dipc_state", 10);

#if 0
    // a test subscriber
    auto callback =
    [this](const std_msgs::msg::String::SharedPtr msg) -> void
    {
        std::cout << "I heard: [" << msg->data.c_str() << "]" << std::endl;
    };
    impl_->test_sub = this->create_subscription<std_msgs::msg::String>("dipc_state", 10, callback);
#endif
}

RosInterface::~RosInterface(void){}


void RosInterface::publishState(std::string state_str)
{
    auto message = std_msgs::msg::String();
    message.data = state_str;
    impl_->state_pub->publish(message);
}


}