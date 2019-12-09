
#pragma once

#include "rclcpp/rclcpp.hpp"


namespace nereid
{

class DIPCVizNode : public rclcpp::Node
{
public:
    DIPCVizNode(void);
    ~DIPCVizNode(void);

private:
    class PrivateImpl;
    std::unique_ptr<PrivateImpl> impl_;
};

}
