
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
    sim->init(0.0, 3.14159, 0.0);

    // use ros clock
    rclcpp::Clock clock;
    auto time = clock.now();

    // loop
    while (!do_shutdown)
    {
        // sleep
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(10ms);

        // tick
        auto new_time = clock.now();
        double dt = new_time.seconds() - time.seconds();
        sim->tick(dt);
        time = new_time;

        // publish
        std::string state_str = sim->stateStr();
        ros->publishState(state_str);
    }

    // shutdown
    rclcpp::shutdown();
    ros_thread.join();

    return 0;
}
