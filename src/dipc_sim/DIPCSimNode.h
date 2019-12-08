
#pragma once

#include "rclcpp/rclcpp.hpp"

namespace nereid
{

class DIPCSimNode : public rclcpp::Node
{
public:
    DIPCSimNode(void);
    ~DIPCSimNode(void);

private:
    class PrivateImpl;
    std::unique_ptr<PrivateImpl> impl_;

    void init(void);
};

}
