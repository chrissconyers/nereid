
#include "DIPCSimNode.h"

#include "constants.h"
#include "DIPCSim.h"

#include "std_msgs/msg/string.hpp"


namespace nereid
{

class DIPCSimNode::PrivateImpl
{
public:
    rclcpp::Clock clock;
    rclcpp::TimerBase::SharedPtr sim_timer;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ctrl_sub;

    DIPCSim sim;
    rclcpp::Time last_tick_time;
    Input last_input;

    std::unique_ptr<std::thread> init_thread;

    void run(void);
};

DIPCSimNode::DIPCSimNode(void)
: rclcpp::Node("dipc_sim")
, impl_(std::make_unique<PrivateImpl>())
{
    // setup publishers
    impl_->state_pub = this->create_publisher<std_msgs::msg::String>("dipc_state", 10);

    // subscribe to dipc ctrl
    impl_->ctrl_sub = this->create_subscription<std_msgs::msg::String>("dipc_ctrl", 10,
        [&](const std_msgs::msg::String::SharedPtr msg){
            impl_->last_input = DIPC::input_from_json(msg->data);
    } );

    // kick off init function, which will then start worker threads
    impl_->init_thread = std::make_unique<std::thread>([this](void){this->init();});
}

DIPCSimNode::~DIPCSimNode(void)
{
    impl_->init_thread->join();
}

void DIPCSimNode::init(void)
{
    // sleep for 1 second to make sure parameter event publisher has started
    using namespace std::chrono_literals;
    rclcpp::sleep_for(1s);

    // configure DIPC system
    nereid::DIPC::Params params = {1.5, 0.5, 0.75, 0.5, 0.75};
    declare_parameter("m0", params.m0);
    declare_parameter("m1", params.m1);
    declare_parameter("m2", params.m2);
    declare_parameter("L1", params.L1);
    declare_parameter("L2", params.L2);

    // prepare the DIPC sim
    impl_->sim.init(0, Constants::pi/8, 0, params);
    impl_->last_tick_time = impl_->clock.now();
    impl_->last_input = (Eigen::VectorXd(1) << 0.0).finished();

    // start the main worker thread
    using namespace std::chrono_literals;
    impl_->sim_timer = create_wall_timer(20ms, [this](void){this->impl_->run();});
}

void DIPCSimNode::PrivateImpl::run(void)
{
    // compute dt
    auto tick_time = clock.now();
    double dt = (tick_time - last_tick_time).seconds();
    last_tick_time = tick_time;

    // tick the simulation
    sim.setInput(last_input);
    sim.tick(dt);

    // publish state at end of this tick
    std_msgs::msg::String message;
    message.data = sim.stateStr();
    state_pub->publish(message);
}

}
