#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_ros/transform_listener.h>

class LocalOdomNode : public rclcpp::Node // MODIFY NAME
{
public:

    LocalOdomNode();
 
private:

    void Rangefinder_sub();
    void VisualOdom_sub();
    void LocalOdom_pub();
    void update_odometry();

    nav_msgs::msg::Odometry translate_odometry(nav_msgs::msg::Odometry::SharedPtr msg, double x, double y, double z);
    nav_msgs::msg::Odometry rotate_odometry(nav_msgs::msg::Odometry::SharedPtr msg);

    void callback_rangefinder(const sensor_msgs::msg::Range::SharedPtr msg);
    void callback_visual_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr rangefinder_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr visual_odom_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_odom_pub_;
};
