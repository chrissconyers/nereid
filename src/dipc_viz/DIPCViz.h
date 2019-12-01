
#pragma once

#include "rclcpp/rclcpp.hpp"


namespace nereid
{

class DIPCViz : public rclcpp::Node
{
public:
    DIPCViz(void);
    ~DIPCViz(void);

private:
    class PrivateImpl;
    std::unique_ptr<PrivateImpl> impl_;
};

}
