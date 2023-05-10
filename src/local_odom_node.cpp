#include "tp_local_odom/local_odom_node.hpp"

LocalOdomNode::LocalOdomNode(): Node("local_odom_node"), tf_broadcaster_(this)
    {
        this->Rangefinder_sub();
        this->VisualOdom_sub();
        this->LocalOdom_pub();
        this->broadcast_frame();
    }
 
void LocalOdomNode::Rangefinder_sub()
{
    rangefinder_sub_=this->create_subscription<sensor_msgs::msg::Range>(
        "/mavros/rangefinder/rangefinder",3,
        std::bind(&LocalOdomNode::callback_rangefinder, this, std::placeholders::_1));
}
void LocalOdomNode::VisualOdom_sub()
{
    visual_odom_sub_=this->create_subscription<nav_msgs::msg::Odometry>(
        "/ov9281_back/visual_odometry_imu/odom",3,
        std::bind(&LocalOdomNode::callback_visual_odom, this, std::placeholders::_1));
}
void LocalOdomNode::LocalOdom_pub(){
    local_odom_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("team_project/local_position/pose",3); 
}

void LocalOdomNode::callback_rangefinder(const sensor_msgs::msg::Range::SharedPtr msg){

}


void LocalOdomNode::callback_visual_odom(const nav_msgs::msg::Odometry::SharedPtr msg){
    
}
void LocalOdomNode::update_odometry(){
    geometry_msgs::msg::PoseStamped local_odometry;


    local_odom_pub_->publish(local_odometry);
}
nav_msgs::msg::Odometry LocalOdomNode::rotate_odometry(nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Create a transform stamped message to represent the rotation
  geometry_msgs::msg::TransformStamped rotation_transform;
  rotation_transform.header.stamp = msg->header.stamp;
  rotation_transform.header.frame_id = "original_frame";
  rotation_transform.child_frame_id = "rotated_frame";
//   set translation of frame
  rotation_transform.transform.translation.x = 1.0;
  rotation_transform.transform.translation.y = 0.0;
  rotation_transform.transform.translation.z = 0.0;

  // Set the rotation to a 90 degree roll around the X-axis
  tf2::Quaternion rotation_quaternion;
  rotation_quaternion.setRPY(M_PI/2.0, 0.0, 0.0); // 90 degree roll around the X-axis
  rotation_transform.transform.rotation = tf2::toMsg(rotation_quaternion);

  // Use the tf2 library to apply the rotation transform to the odometry message
  nav_msgs::msg::Odometry rotated_odom;
  tf2::doTransform(*msg, rotated_odom, rotation_transform);

  // Return the rotated odometry message
  return rotated_odom;
}

nav_msgs::msg::Odometry LocalOdomNode::translate_odometry(nav_msgs::msg::Odometry::SharedPtr msg, double x, double y, double z){
    nav_msgs::msg::Odometry updated_msg = *msg; // Make a new copy of the message
    updated_msg.pose.pose.position.x += x; // Update the copy with the desired translation in the x direction
    updated_msg.pose.pose.position.y += y; // Update the copy with the desired translation in the y direction
    updated_msg.pose.pose.position.z += z; // Update the copy with the desired translation in the z direction
    return updated_msg; // Return the updated copy
}

void LocalOdomNode::broadcast_frame(){
    // Create a transform stamped message to represent the rotation
    geometry_msgs::msg::TransformStamped rotation_transform;
    rotation_transform.header.stamp = this->now();
    rotation_transform.header.frame_id = "original_frame";// // rename parent acording drone ardupilot frame
    rotation_transform.child_frame_id = "rotated_frame"; // rename to the 
    //   set translation of frame
    rotation_transform.transform.translation.x = 1.0;
    rotation_transform.transform.translation.y = 0.0;
    rotation_transform.transform.translation.z = 0.0;

    // Set the rotation to a 90 degree roll around the X-axis
    tf2::Quaternion rotation_quaternion;
    rotation_quaternion.setRPY(M_PI/2.0, 0.0, 0.0); // 90 degree roll around the X-axis
    rotation_transform.transform.rotation = tf2::toMsg(rotation_quaternion);
    tf_broadcaster_.sendTransform(rotation_transform);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    broadcast_frame();
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalOdomNode>();
    

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}