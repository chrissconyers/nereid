
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"



class SubTest : public rclcpp::Node
{
public:
    SubTest()
    : Node("subtest")
    {
        auto callback =
        [this](const std_msgs::msg::String::SharedPtr msg) -> void
        {
            std::cout << "I heard: [" << msg->data.c_str() << "]" << std::endl;
        };

        subscription_ = this->create_subscription<std_msgs::msg::String>("chatter", 10, callback);
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubTest>());
    rclcpp::shutdown();
    return 0;
}