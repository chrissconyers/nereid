

#include "DIPCSim.h"

#include <nlohmann/json.hpp>
#include <iostream>
#include <chrono>
#include <thread>
using namespace std::chrono_literals;


namespace ssci
{

DIPCSim::DIPCSim(void)
: Node("dipc_sim")
{
    // create a publisher for sending out state messages (this is "true" state)
    publisher_ = this->create_publisher<std_msgs::msg::String>("dipc_state", 10);

#if 0
    // a test subscriber
    auto callback =
    [this](const std_msgs::msg::String::SharedPtr msg) -> void
    {
        std::cout << "I heard: [" << msg->data.c_str() << "]" << std::endl;
    };
    subscription_ = this->create_subscription<std_msgs::msg::String>("dipc_state", 10, callback);
#endif
}

void DIPCSim::publishState(void)
{
    static int count = 0;

    
    nlohmann::json j;
    j["name"] = "tarquin";
    j["id"] = ++count;
    std::string msg_str = j.dump();

    std::cout << "publishing '" << msg_str << "'" << std::endl;
    auto message = std_msgs::msg::String();
    message.data = msg_str;
    publisher_->publish(message);
}
    
}


int main(int argc, char** argv)
{
    // always init first
    rclcpp::init(0, nullptr);

    // set shutdown callback (ros2 gobbles up SIGINT interrupts)
    bool shutdown = false;
    rclcpp::on_shutdown(
        [&](void) -> void
        {
            std::cout << "ima shutdown" << std::endl;
            shutdown = true;
        }
        );

    auto sim = std::make_shared<ssci::DIPCSim>();

    // add nodes to executor (required for subscriptions)
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(sim);

    // do my own executive management
    while (!shutdown)
    {
        sim->publishState();
        std::this_thread::sleep_for(200ms);
        exec.spin_some();
    }

    // handle shutdown
    rclcpp::shutdown();
    std::cout << "done" << std::endl;
    return 0;
}
