#include "tp_local_odom/local_odom_node.hpp"
LocalOdomNode::LocalOdomNode(): Node("local_odom_node")
    {
        this->Rangefinder_sub();
        this->VisualOdom_sub();
        this->LocalOdom_pub();
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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalOdomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}