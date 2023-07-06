#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/range.hpp>

#include "interfaces/srv/reset_odom.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class PoseRepublisher : public rclcpp::Node  // MODIFY NAME
{
 public:
  PoseRepublisher();

 private:
  float gamma_world_ = 0;
  float roll_camera_ = 0;
  float pitch_camera_ = 0;
  float yaw_camera_ = 0;
  void Init();
  void DefaultValues();
  void SetupSubscribers();
  void SetupPublishers();
  void SetupServices();
  void reset_srv_handle();
  void reset_odom_callback(
      const std::shared_ptr<interfaces::srv::ResetOdom::Request> request,
      std::shared_ptr<interfaces::srv::ResetOdom::Response> response);
  rclcpp::Service<interfaces::srv::ResetOdom>::SharedPtr reset_odom_srv;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      subscription_;
  void vis_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
};
