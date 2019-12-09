
#include "DIPCCtrlPDNode.h"

int main(int argc, char** argv)
{
    rclcpp::init(0, nullptr);
    rclcpp::spin(std::make_shared<nereid::DIPCCtrlPDNode>());
    rclcpp::shutdown();
    return 0;
}