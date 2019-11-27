
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace ssci
{
    class DIPCSim : public rclcpp::Node
    {
    public:
        DIPCSim(void);

        void publishState(void);

    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        //rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    };
};