#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>

class TfSubscriberNode: public rclcpp::Node // MODIFY NAME
{
public:
    TfSubscriberNode();
 
private:
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;
    //std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ ;

    void ardupilot_frame_broadcaster(const geometry_msgs::msg::TransformStamped& transform);
    void global_odom_broadcast(const geometry_msgs::msg::TransformStamped& transform);
    void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
    void setup_subscriber();
    void base_camera_static_tf();















    tf2_ros::StaticTransformBroadcaster tf_broadcaster_;
    std::string target_frame;


    void Rangefinder_sub();
    void VisualOdom_sub();
    void LocalOdom_pub();
    void update_odometry();
    void InitListener();
    void broadcast_frame();
    nav_msgs::msg::Odometry translate_odometry(nav_msgs::msg::Odometry::SharedPtr msg, double x, double y, double z);
    nav_msgs::msg::Odometry rotate_odometry(nav_msgs::msg::Odometry::SharedPtr msg);

    void callback_rangefinder(const sensor_msgs::msg::Range::SharedPtr msg);
    void callback_visual_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr rangefinder_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr visual_odom_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_odom_pub_;
};

