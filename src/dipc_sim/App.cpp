
#include "DIPCSim.h"
#include "RosInterface.h"

#include <chrono>
#include <thread>


std::atomic<bool> do_shutdown(false);
void handleSigterm(int)
{
    do_shutdown = true;
}

int main(int argc, char** argv)
{
    // register callback to handle sigterm
    struct sigaction action;
    memset(&action, 0, sizeof(action));
    action.sa_handler = handleSigterm;
    sigfillset(&action.sa_mask);
    sigaction(SIGTERM, &action, NULL);
    sigaction(SIGINT, &action, NULL);

    // init ros
    rclcpp::init(0, nullptr);
    rclcpp::uninstall_signal_handlers();
    rclcpp::executors::SingleThreadedExecutor exec;
    std::thread ros_thread([&](void){exec.spin();});

    // start ros interface
    auto ros = std::make_shared<nereid::RosInterface>();
    exec.add_node(ros);

    // start DIPCSim
    auto sim = std::make_shared<nereid::DIPCSim>();

    while (!do_shutdown)
    {
        std::string state_str = sim->publishState();
        ros->publishState(state_str);

        using namespace std::chrono_literals;
        std::this_thread::sleep_for(10ms);
    }

    // shutdown
    rclcpp::shutdown();
    ros_thread.join();

    return 0;
}
