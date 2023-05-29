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
#include <chrono>
#include <memory>
#include "interfaces/srv/reset_odom.hpp"

class TfSubscriberNode: public rclcpp::Node // MODIFY NAME
{
public:
    TfSubscriberNode();
 
private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    //std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ ;

    void ardupilot_frame_broadcaster(const geometry_msgs::msg::TransformStamped& transform);
    void global_odom_broadcast(const geometry_msgs::msg::TransformStamped& transform);
    void tf_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void setup_subscriber();
    void base_camera_static_tf();
    void setup_global_local_transform();
    void reset_srv_handle();
    void GlobalOdom_pub();
    void LocalOdom_pub();
    void reset_odom_callback(
      const std::shared_ptr<interfaces::srv::ResetOdom::Request> request,
      std::shared_ptr<interfaces::srv::ResetOdom::Response> response);
    rclcpp::Service<interfaces::srv::ResetOdom>::SharedPtr reset_odom_srv;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr global_odom_pub_;



    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

   





    geometry_msgs::msg::TransformStamped global_local_tf;
    tf2_ros::StaticTransformBroadcaster tf_broadcaster_;
    std::string target_frame;
    double rangefinder_vis_range_info=0;
    double range_visual=0;
    double corrected_range=0;
    double last_range=0;
    void Rangefinder_sub();
    void VisualOdom_sub();

    void update_odometry();
    void InitListener();
    void broadcast_frame();
    nav_msgs::msg::Odometry translate_odometry(nav_msgs::msg::Odometry::SharedPtr msg, double x, double y, double z);
    nav_msgs::msg::Odometry rotate_odometry(nav_msgs::msg::Odometry::SharedPtr msg);



      
    void callback_rangefinder(const sensor_msgs::msg::Range::SharedPtr msg);
    void callback_visual_odom(const nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr rangefinder_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr visual_odom_sub_;


};

