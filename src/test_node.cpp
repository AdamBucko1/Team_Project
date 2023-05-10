#include <chrono>
#include <memory>

#include "tp_local_odom/local_odom_node.hpp"


class TfSubscriberNode : public rclcpp::Node {
public:
  TfSubscriberNode() : Node("tf_subscriber_node") {
    subscription_ = create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", rclcpp::SensorDataQoS(), std::bind(&TfSubscriberNode::callback, this, std::placeholders::_1));

  }
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

private:
  void callback(const tf2_msgs::msg::TFMessage::SharedPtr msg) const {
    for (const auto& transform : msg->transforms) {
       if (transform.child_frame_id == "camera_link") {
        RCLCPP_INFO(get_logger(), "Transform from '%s' to '%s' with time stamp: %f", transform.header.frame_id.c_str(), transform.child_frame_id.c_str(), transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9);
        RCLCPP_INFO(get_logger(), "Received transform message with time stamp:%s:  %f",transform.child_frame_id, transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9);
       

        geometry_msgs::msg::TransformStamped rotation_transform;

       
        // Copy the timestamp from the "camera_link" transform to the "drone_link" transform
        rotation_transform.header.stamp = transform.header.stamp;

        // Create a transform stamped message to represent the rotation
        
        // rotation_transform.header.stamp = this->now();
        rotation_transform.header.frame_id = "camera_link";// // rename parent acording drone ardupilot frame
        rotation_transform.child_frame_id = "drone_link"; // rename to the 
        //   set translation of frame
        rotation_transform.transform.translation.x = 0.2;
        rotation_transform.transform.translation.y = 0.0;
        rotation_transform.transform.translation.z = 0.0;

        // Set the rotation to a 90 degree roll around the X-axis
        tf2::Quaternion rotation_quaternion;
        rotation_quaternion.setRPY(0*M_PI/2.0, 0.0, 0.0); // 90 degree roll around the X-axis
        rotation_transform.transform.rotation = tf2::toMsg(rotation_quaternion);
        tf_broadcaster_->sendTransform(rotation_transform);
       
       }
    }
  }
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TfSubscriberNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}