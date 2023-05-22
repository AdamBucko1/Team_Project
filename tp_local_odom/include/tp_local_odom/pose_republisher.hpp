#include <sensor_msgs/msg/range.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <chrono>
#include <memory>
#include "interfaces/srv/reset_odom.hpp"

class PoseRepublisher : public rclcpp::Node // MODIFY NAME
{
public:

    PoseRepublisher();


 
private:
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

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
    void vis_pose_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

};
