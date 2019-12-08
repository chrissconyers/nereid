
#include "DIPCSimNode.h"

int main(int argc, char** argv)
{
    rclcpp::init(0, nullptr);
    rclcpp::spin(std::make_shared<nereid::DIPCSimNode>());
    rclcpp::shutdown();
    return 0;
}
