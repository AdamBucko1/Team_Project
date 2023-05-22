#include "include/tp_local_odom/pose_republisher.hpp"
PoseRepublisher::PoseRepublisher(): Node("local_odom_node")
{
     this->Init();
}
void PoseRepublisher::Init()
{
    this->DefaultValues();
    this->SetupSubscribers();
    this->SetupPublishers();
    this->SetupServices();
}
void PoseRepublisher::DefaultValues()
{

}
void PoseRepublisher::SetupSubscribers()
{
    subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/visual_slam/tracking/vo_pose", rclcpp::SensorDataQoS(), std::bind(&PoseRepublisher::vis_pose_callback, this, std::placeholders::_1));   
}
void PoseRepublisher::SetupPublishers()
{

}
void PoseRepublisher::SetupServices()
{
    this->reset_srv_handle();
}
void PoseRepublisher::reset_odom_callback(
      const std::shared_ptr<interfaces::srv::ResetOdom::Request> request,
      std::shared_ptr<interfaces::srv::ResetOdom::Response> response) 
      {

        if (request->reset_odom=true)
        {

        }

  }

void PoseRepublisher::reset_srv_handle(){

    reset_odom_srv=this->create_service<interfaces::srv::ResetOdom>("reset_odom", 
        std::bind(&PoseRepublisher::reset_odom_callback, this, std::placeholders::_1, std::placeholders::_2));
}
void PoseRepublisher::vis_pose_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg){


 }
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseRepublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
