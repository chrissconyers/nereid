
#include "DIPCViz.h"

#include "constants.h"
#include "DIPC.h"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <chrono>
using namespace std::chrono_literals;


namespace nereid
{

class DIPCViz::PrivateImpl
{
public:
    rclcpp::Clock clock;
    rclcpp::TimerBase::SharedPtr marker_timer;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr transform_pub;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr params_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster = nullptr;


    DIPC::Params params;

    void updateState(const std_msgs::msg::String::SharedPtr msg);
    void sendMarkers(void);
};

DIPCViz::DIPCViz(void)
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

DIPCViz::~DIPCViz(void)
{
}

void DIPCViz::PrivateImpl::updateState(const std_msgs::msg::String::SharedPtr msg)
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
    tfstamped.transform.translation.x = 0.0;
    //tfstamped.transform.translation.x = state[0];
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

void DIPCViz::PrivateImpl::sendMarkers(void)
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

}
