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

    double gamma_world_ = 1.5707963;
    double roll_camera_  = 0;
    double pitch_camera_ = 0;
    double yaw_camera_ = 0;
    double x_camera_ = 0.008;
    double y_camera_ = 0.125;
    double z_rangefinder_ = 0.075;
    double coord_offset_en_ = true;
    double pose_diff_limit_ = 0.05;
    tf2::Vector3 coord_offset_ = tf2::Vector3(x_camera_, y_camera_, 0);

    

    tf2::Vector3 position_orig, position_world, final_position_;
    tf2::Quaternion quat_orig, quat_body_to_world, quat_world, quat_cam_to_body, quat_cam_to_world, final_orientation_;

    position_orig.setValue(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    quat_orig.setValue(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    
    quat_body_to_world.setRPY(0, 0, -gamma_world_);
    quat_cam_to_body.setRPY(roll_camera_, pitch_camera_, yaw_camera_);
    quat_cam_to_world = quat_orig.inverse() * quat_body_to_world * quat_orig * quat_cam_to_body;
    quat_world = quat_orig * quat_cam_to_world;
    position_world = tf2::quatRotate(quat_body_to_world, position_orig);
    double yaw_world;
    double roll_world;
    double pitch_world;
      // Convert the tf2::Quaternion to tf2::Matrix3x3
    tf2::Matrix3x3 matrix(quat_world);
    matrix.getRPY(roll_world,pitch_world,yaw_world);

    tf2::Vector3 cam_offset = coord_offset_.rotate(tf2::Vector3(0,0,1), yaw_world - M_PI_2); //transfromation from end of arm to start coord system
    
    final_position_ = position_world - cam_offset + coord_offset_; //transform from arm and difference between coord system
    final_orientation_ = quat_world;
    geometry_msgs::msg::PoseStamped final_pose;

    final_pose.pose.position.x=final_position_.getX();
    final_pose.pose.position.y=final_position_.getY();
    final_pose.pose.position.z=final_position_.getZ();

    final_pose.pose.orientation.x = final_orientation_.getX();
    final_pose.pose.orientation.y = final_orientation_.getY();
    final_pose.pose.orientation.z = final_orientation_.getZ();
    final_pose.pose.orientation.w = final_orientation_.getW();

    pose_publisher_->publish(final_pose);

 }



// void PoseRepublisher::RangefinderCallback(const sensor_msgs::Range::ConstPtr& msg)
// {
//     final_alt_ = z_rangefinder_ + msg->range;
// }

// void PoseRepublisher::CallbackPoseOutTimer(const ros::TimerEvent &event)
// {
//     std_msgs::Header header;
//     geometry_msgs::Pose pose;

//     // Fill header   
//     header.stamp = ros::Time::now();
//     header.frame_id = "map";
//     header.seq = seq++;

//     tf::quaternionTFToMsg(final_orientation_, pose.orientation);
//     tf::pointTFToMsg(final_position_, pose.position);
//     pose.position.z = final_alt_;

//     geometry_msgs::PoseStamped pose_msg;
//     pose_msg.header = header;
//     pose_msg.pose = pose;
//     pose_out_pub_.publish(pose_msg);
// }




int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseRepublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
