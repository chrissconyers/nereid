
#include "DIPCViz.h"

int main(int argc, char** argv)
{
    rclcpp::init(0, nullptr);
    rclcpp::spin(std::make_shared<nereid::DIPCViz>());
    rclcpp::shutdown();
}
