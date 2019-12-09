
#include "DIPCVizNode.h"

#include "constants.h"
#include "DIPC.h"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
//#include "rcl_interfaces/msg/parameter_event.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <fmt/core.h>
#include <chrono>
using namespace std::chrono_literals;


namespace nereid
{

class DIPCVizNode::PrivateImpl
{
public:
    rclcpp::Clock clock;
    rclcpp::TimerBase::SharedPtr marker_timer;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr transform_pub;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr params_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub;

    rclcpp::SyncParametersClient::SharedPtr param_client;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_sub;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster = nullptr;


    DIPC::Params params;

    void updateState(const std_msgs::msg::String::SharedPtr msg);
    void sendMarkers(void);

    void getAllParams(void);
    void handleParamEvent(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);
    void updateParam(const rcl_interfaces::msg::Parameter& param);
};

DIPCVizNode::DIPCVizNode(void)
: rclcpp::Node("dipc_viz")
, impl_(std::make_unique<PrivateImpl>())
{
    impl_->marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("marker", 10);
    impl_->transform_pub = this->create_publisher<geometry_msgs::msg::TransformStamped>("tf", 10);

    // params subscriber
    impl_->params_sub = this->create_subscription<std_msgs::msg::String>("dipc_params", 10,
        [&](const std_msgs::msg::String::SharedPtr msg){
            impl_->params = DIPC::Params::from_json(msg->data);
    } );

    impl_->param_client = std::make_shared<rclcpp::SyncParametersClient>(this, "dipc_sim");
    while (!impl_->param_client->wait_for_service(1s));
    impl_->param_sub = impl_->param_client->on_parameter_event(
        [&](const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
            {impl_->handleParamEvent(event);});
    impl_->getAllParams();

    // state subscriber
    impl_->state_sub = this->create_subscription<std_msgs::msg::String>("dipc_state", 10,
        [&](const std_msgs::msg::String::SharedPtr msg){
            if (impl_->tf_broadcaster == nullptr)
                impl_->tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
            impl_->updateState(msg);
    } );

    // periodically send shapes
    impl_->marker_timer = this->create_wall_timer(10ms, 
        [&](void){
            impl_->sendMarkers();
    } );
}

DIPCVizNode::~DIPCVizNode(void)
{
}

void DIPCVizNode::PrivateImpl::updateState(const std_msgs::msg::String::SharedPtr msg)
{
    if (!tf_broadcaster)
        return;

    // convert state message
    State state = DIPC::from_json(msg->data);

    // some reusable bits
    auto time = clock.now();
    geometry_msgs::msg::TransformStamped tfstamped;
    tf2::Quaternion q;

    // reconceptualize in ned
    tfstamped.header.frame_id = "world";
    tfstamped.header.stamp = time;
    tfstamped.child_frame_id = "ned";
    tfstamped.transform.translation.x = 0.0;
    tfstamped.transform.translation.y = 0.0;
    tfstamped.transform.translation.z = 0.0;
    q.setRPY(0, Constants::pi, 0);
    tfstamped.transform.rotation.x = q.x();
    tfstamped.transform.rotation.y = q.y();
    tfstamped.transform.rotation.z = q.z();
    tfstamped.transform.rotation.w = q.w();
    tf_broadcaster->sendTransform(tfstamped);

    // create and send cart transform
    tfstamped.header.frame_id = "ned";
    tfstamped.header.stamp = time;
    tfstamped.child_frame_id = "cart";
    //tfstamped.transform.translation.x = 0.0;
    tfstamped.transform.translation.x = state[0];
    tfstamped.transform.translation.y = 0.0;
    tfstamped.transform.translation.z = 0.0;
    tfstamped.transform.rotation.x = 0.0;
    tfstamped.transform.rotation.y = 0.0;
    tfstamped.transform.rotation.z = 0.0;
    tfstamped.transform.rotation.w = 1.0;
    tf_broadcaster->sendTransform(tfstamped);

    // create and send pendulum 1 transform
    tfstamped.header.frame_id = "cart";
    tfstamped.header.stamp = time;
    tfstamped.child_frame_id = "pend1_R";
    tfstamped.transform.translation.x = 0;
    tfstamped.transform.translation.y = 0;
    tfstamped.transform.translation.z = 0;
    q.setRPY(0, state[1], 0);
    tfstamped.transform.rotation.x = q.x();
    tfstamped.transform.rotation.y = q.y();
    tfstamped.transform.rotation.z = q.z();
    tfstamped.transform.rotation.w = q.w();
    tf_broadcaster->sendTransform(tfstamped);
    tfstamped.header.frame_id = "pend1_R";
    tfstamped.header.stamp = time;
    tfstamped.child_frame_id = "pend1";
    tfstamped.transform.translation.x = 0;
    tfstamped.transform.translation.y = 0;
    tfstamped.transform.translation.z = -params.L1/2;
    tfstamped.transform.rotation.x = 0;
    tfstamped.transform.rotation.y = 0;
    tfstamped.transform.rotation.z = 0;
    tfstamped.transform.rotation.w = 1;
    tf_broadcaster->sendTransform(tfstamped);

    // create and send pendulum 2 transform
    tfstamped.header.frame_id = "pend1";
    tfstamped.header.stamp = time;
    tfstamped.child_frame_id = "pend2_R";
    tfstamped.transform.translation.x = 0;
    tfstamped.transform.translation.y = 0;
    tfstamped.transform.translation.z = -params.L1/2;
    q.setRPY(0, state[2]-state[1], 0);
    tfstamped.transform.rotation.x = q.x();
    tfstamped.transform.rotation.y = q.y();
    tfstamped.transform.rotation.z = q.z();
    tfstamped.transform.rotation.w = q.w();
    tf_broadcaster->sendTransform(tfstamped);
    tfstamped.header.frame_id = "pend2_R";
    tfstamped.header.stamp = time;
    tfstamped.child_frame_id = "pend2";
    tfstamped.transform.translation.x = 0;
    tfstamped.transform.translation.y = 0;
    tfstamped.transform.translation.z = -params.L2/2;
    tfstamped.transform.rotation.x = 0;
    tfstamped.transform.rotation.y = 0;
    tfstamped.transform.rotation.z = 0;
    tfstamped.transform.rotation.w = 1;
    tf_broadcaster->sendTransform(tfstamped);
}

void DIPCVizNode::PrivateImpl::sendMarkers(void)
{
    auto time = clock.now();
    visualization_msgs::msg::Marker marker;

    // cart shape
    marker.header.frame_id = "cart";
    marker.header.stamp = time;
    marker.ns = "dipc";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    //marker.frame_locked = true;
    //marker.lifetime = rclcpp::Duration();
    marker.scale.x = params.L1;
    marker.scale.y = params.L1/10;
    marker.scale.z = params.L1/2;
    marker.color.r = 1.0;
    marker.color.a = 1.0;
    marker_pub->publish(marker);

    // pend 1 shape
    marker.header.frame_id = "pend1";
    marker.header.stamp = time;
    marker.ns = "dipc";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    //marker.frame_locked = true;
    //marker.lifetime = rclcpp::Duration();
    marker.scale.x = params.L1/8;
    marker.scale.y = params.L1/8;
    marker.scale.z = params.L1;
    marker.color.g = 1.0;
    marker.color.a = 1.0;
    marker_pub->publish(marker);

    // pend 2 shape
    marker.header.frame_id = "pend2";
    marker.header.stamp = time;
    marker.ns = "dipc";
    marker.id = 2;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    //marker.frame_locked = true;
    //marker.lifetime = rclcpp::Duration();
    marker.scale.x = params.L1/8;
    marker.scale.y = params.L1/8;
    marker.scale.z = params.L2;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker_pub->publish(marker);
}

void DIPCVizNode::PrivateImpl::getAllParams(void)
{
    for (auto& param : param_client->get_parameters({"m0", "m1", "m2", "L1", "L2"}))
        updateParam(param.to_parameter_msg());
}

void DIPCVizNode::PrivateImpl::handleParamEvent(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
    for(auto param : event->changed_parameters)
        updateParam(param);
}

void DIPCVizNode::PrivateImpl::updateParam(const rcl_interfaces::msg::Parameter& param)
{
    if (param.name == "m0")
        params.m0 = param.value.double_value;
    else if (param.name == "m1")
        params.m1 = param.value.double_value;
    else if (param.name == "m2")
        params.m2 = param.value.double_value;
    else if (param.name == "L1")
        params.L1 = param.value.double_value;
    else if (param.name == "L2")
        params.L2 = param.value.double_value;
}

}
