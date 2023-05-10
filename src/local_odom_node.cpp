#include "tp_local_odom/local_odom_node.hpp"

LocalOdomNode::LocalOdomNode(): Node("local_odom_node"), tf_broadcaster_(this)
    {


        // this->InitListener();
        // Declare and acquire `target_frame` parameter

        tf_buffer =
        std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
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

// void LocalOdomNode::InitListener(){
//     tf_listener_ =
//   std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
// }




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

nav_msgs::msg::Odometry LocalOdomNode::translate_odometry(nav_msgs::msg::Odometry::SharedPtr msg, double x, double y, double z){
    nav_msgs::msg::Odometry updated_msg = *msg; // Make a new copy of the message
    updated_msg.pose.pose.position.x += x; // Update the copy with the desired translation in the x direction
    updated_msg.pose.pose.position.y += y; // Update the copy with the desired translation in the y direction
    updated_msg.pose.pose.position.z += z; // Update the copy with the desired translation in the z direction
    return updated_msg; // Return the updated copy
}

void LocalOdomNode::broadcast_frame(){
    while(true){
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        try{

        
        geometry_msgs::msg::TransformStamped rotation_transform;

        geometry_msgs::msg::TransformStamped camera_transform =
        

            (*tf_buffer).lookupTransform("camera_link", "odom", tf2::TimePointZero);

        // Copy the timestamp from the "camera_link" transform to the "drone_link" transform
        rotation_transform.header.stamp = camera_transform.header.stamp;

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
        tf_broadcaster_.sendTransform(rotation_transform);
        RCLCPP_INFO(this->get_logger(),"%f",camera_transform.header.stamp);
        RCLCPP_INFO(this->get_logger(),"%s",camera_transform.header.stamp);

        }
        catch(tf2::TransformException& ex){
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
        }
        

    }
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalOdomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}