#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>
#include <chrono>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <interfaces/srv/reset_odom.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>

class PoseRepublisher : public rclcpp::Node {
 public:
  PoseRepublisher();

 private:
  // Methods
  void RotatePoseAroundAxis(geometry_msgs::msg::PoseStamped &pose,
                            const tf2::Vector3 &axis, double angle);
  void GlobalOdomBroadcast(
      const geometry_msgs::msg::TransformStamped &transform);
  void TransformNWUToPoseENU(
      const geometry_msgs::msg::TransformStamped &odom_tf,
      geometry_msgs::msg::PoseStamped &local_pose);
  void BaseCameraStaticTF();
  void SetupGlobalTransform();
  void DefaultValues();
  void StartFunctions();

  // Variables
  double rangefinder_vis_range_info_;
  double range_visual_;
  double corrected_range_;
  double last_range_;
  double rangefinder_offset_;
  double camera_offset_x_;
  double camera_offset_y_;
  double camera_offset_z_;
  double camera_rotation_;  // Radians

  // TF variables
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::StaticTransformBroadcaster tf_broadcaster_;

  // Class output variables
  geometry_msgs::msg::PoseStamped local_pose_;
  geometry_msgs::msg::PoseWithCovarianceStamped local_pose_with_covariance_;
  geometry_msgs::msg::PoseStamped global_pose_;
  geometry_msgs::msg::TransformStamped global_local_tf_;

  // Subscribers
  void PoseVisualSub();
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pose_visual_sub_;
  void CallbackVisualOdom(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  void RangefinderSub();
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr rangefinder_sub_;
  void CallbackRangefinder(const sensor_msgs::msg::Range::SharedPtr msg);

  // Publishers
  void LocalOdomPub();
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      local_odom_pub_;

  void GlobalOdomPub();
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      global_odom_pub_;

  // Services
  void ResetSrvHandle();
  rclcpp::Service<interfaces::srv::ResetOdom>::SharedPtr reset_odom_srv_;

  void ResetOdomCallback(
      const std::shared_ptr<interfaces::srv::ResetOdom::Request> request,
      std::shared_ptr<interfaces::srv::ResetOdom::Response> response);
};
