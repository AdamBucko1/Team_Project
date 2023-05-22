#include "tp_local_odom/pose_republisher.hpp"
#include "tp_local_odom/ae_utils.hpp"
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
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/visual_slam/tracking/vo_pose", rclcpp::SensorDataQoS(), std::bind(&PoseRepublisher::vis_pose_callback, this, std::placeholders::_1));   
}
void PoseRepublisher::SetupPublishers()
{
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("drone_pose", 10);
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
void PoseRepublisher::vis_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){

    // // The initial displacement of the camera from the drone center
    // tf2::Vector3 initial_displacement(0.14, 0, 0);

    // // Convert the PoseStamped message to a tf2 Transform
    // tf2::Transform camera_transform;
    // tf2::fromMsg(msg->pose, camera_transform);

    // // Apply the initial displacement in the drone's frame of reference
    // tf2::Transform initial_transform(tf2::Quaternion::getIdentity(), initial_displacement);
    // tf2::Transform camera_transform_in_drone_frame = initial_transform.inverse() * camera_transform;

    // // The displacement of the drone center from the camera
    // tf2::Vector3 displacement(-0.14, 0, 0);

    // // Apply the displacement in the camera's frame of reference
    // tf2::Transform drone_transform = camera_transform_in_drone_frame * tf2::Transform(tf2::Quaternion::getIdentity(), displacement);

    // // Fill in the drone pose message
    // geometry_msgs::msg::PoseStamped drone_pose;
    // drone_pose.header = msg->header;

    // // Manually create a Pose from the Transform
    // drone_pose.pose.position.x = drone_transform.getOrigin().x();
    // drone_pose.pose.position.y = drone_transform.getOrigin().y();
    // drone_pose.pose.position.z = drone_transform.getOrigin().z();
    // drone_pose.pose.orientation = tf2::toMsg(drone_transform.getRotation());

    // pose_publisher_->publish(drone_pose);


 }
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseRepublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
