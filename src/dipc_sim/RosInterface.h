
#pragma once

#include "rclcpp/rclcpp.hpp"


namespace nereid
{
    class RosInterface : public rclcpp::Node
    {
    public:
        RosInterface(void);
        ~RosInterface(void);

        void publishState(std::string state_str);

    private:
        class PrivateImpl;
        std::unique_ptr<PrivateImpl> impl_;

    };
}
