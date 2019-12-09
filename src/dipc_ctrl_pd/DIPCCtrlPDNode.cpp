
#include "DIPCCtrlPDNode.h"

#include "constants.h"
#include "DIPC.h"
#include "std_msgs/msg/string.hpp"


namespace nereid
{

class DIPCCtrlPDNode::PrivateImpl
{
public:
    double x_desired = 0.0;
    double natural_freq = 3.00;
    double damping_ratio = 1.00;
    double u_max = 500.0;

    double k_p = 0.0;
    double k_d = 0.0;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ctrl_pub;
    rcl_interfaces::msg::SetParametersResult handleParam(const std::vector<rclcpp::Parameter>& params);

    std::unique_ptr<std::thread> init_thread;

    void update(const State& state);

    void computeGains(void);
};

DIPCCtrlPDNode::DIPCCtrlPDNode(void)
: rclcpp::Node("dipc_ctrl_pd")
, impl_(std::make_unique<PrivateImpl>())
{
    // create publisher
    impl_->ctrl_pub = this->create_publisher<std_msgs::msg::String>("dipc_ctrl", 10);

    // subscribe to dipc state
    impl_->state_sub = this->create_subscription<std_msgs::msg::String>("dipc_state", 10,
        [&](const std_msgs::msg::String::SharedPtr msg){
            impl_->update(DIPC::state_from_json(msg->data));
    } );

    // set up parameter callback
    this->set_on_parameters_set_callback(
        [&](const std::vector<rclcpp::Parameter>& params)
            {return impl_->handleParam(params);});

    // kick off init function, which will then start worker threads
    impl_->init_thread = std::make_unique<std::thread>([this](void){this->init();});
}

DIPCCtrlPDNode::~DIPCCtrlPDNode(void)
{
    impl_->init_thread->join();
}

void DIPCCtrlPDNode::init(void)
{
    // sleep for 1 second to make sure parameter event publisher has started
    using namespace std::chrono_literals;
    rclcpp::sleep_for(1s);

    // configure controller
    declare_parameter("x_des", impl_->x_desired);
    declare_parameter("f_nat", impl_->natural_freq);
    declare_parameter("damp", impl_->damping_ratio);
    declare_parameter("u_max", impl_->u_max);
    impl_->computeGains();
}

rcl_interfaces::msg::SetParametersResult DIPCCtrlPDNode::PrivateImpl::handleParam(const std::vector<rclcpp::Parameter>& params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto& param : params)
    {
        if (param.get_name() == "x_des" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            x_desired = param.get_value<double>();
        else if (param.get_name() == "f_nat" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
            natural_freq = param.get_value<double>();
            computeGains();
        }
        else if (param.get_name() == "damp" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
            damping_ratio = param.get_value<double>();
            computeGains();
        }
        else if (param.get_name() == "u_max" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            u_max = param.get_value<double>();
        else
        {
            result.successful = false;
            result.reason = "unknown parameter or invalid type";
        }
    }

    return result;
}

void DIPCCtrlPDNode::PrivateImpl::update(const State& state)
{
    double u = k_p*(x_desired-state[0]) + k_d*(-state[3]); // v_desired is always zero
    if (u > u_max)
        u = u_max;
    if (u < -u_max)
        u = -u_max;
    Input input = (Eigen::VectorXd(1) << u).finished();

    std_msgs::msg::String message;
    message.data = DIPC::input_to_json(input);
    ctrl_pub->publish(message);
}

void DIPCCtrlPDNode::PrivateImpl::computeGains(void)
{
    double alpha = 2.0 * Constants::pi * natural_freq;
    k_p = alpha * alpha;
    k_d = 2 * damping_ratio * alpha;
}

}
