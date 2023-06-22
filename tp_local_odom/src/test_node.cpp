#include "tp_local_odom/test_node.hpp"


TfSubscriberNode::TfSubscriberNode(): Node("tf_subscriber_node"),tf_broadcaster_(this), tf_buffer_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME), tf2::durationFromSec(0)),
      tf_listener_(tf_buffer_)
{
  this->LocalOdom_pub();
  this->Rangefinder_sub();
  this->GlobalOdom_pub();
  this->setup_subscriber();
  this->base_camera_static_tf();
  this->setup_global_local_transform();
  this->reset_srv_handle();
  tf_buffer_.setUsingDedicatedThread(true);  // Optional, for better performance    
}

void TfSubscriberNode::setup_subscriber(){
    subscription_= create_subscription<geometry_msgs::msg::PoseStamped>(
        "/visual_slam/tracking/vo_pose", rclcpp::SensorDataQoS(), std::bind(&TfSubscriberNode::tf_callback, this, std::placeholders::_1));
        }

void TfSubscriberNode::Rangefinder_sub()
{//" /mavros/rangefinder/rangefinder
    rangefinder_sub_=this->create_subscription<sensor_msgs::msg::Range>(
        "/aeros/rangefinder",3,
        std::bind(&TfSubscriberNode::callback_rangefinder, this, std::placeholders::_1));
}

void TfSubscriberNode::callback_rangefinder(const sensor_msgs::msg::Range::SharedPtr msg){
  if (msg->range == 0.0){
  corrected_range = last_range+rangefinder_vis_range_info-range_visual;
  last_range=corrected_range;
  }
  else{
  rangefinder_vis_range_info=range_visual;
  last_range = msg->range;
  }
}
void TfSubscriberNode::LocalOdom_pub(){
                                                                              //"" /mavros/mocap/pose
  local_odom_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/aeros/vision_odom/pose",10); 
    
}
void TfSubscriberNode::GlobalOdom_pub(){
    global_odom_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("global_pose",10); 
}

void TfSubscriberNode::reset_odom_callback(
      const std::shared_ptr<interfaces::srv::ResetOdom::Request> request,
      std::shared_ptr<interfaces::srv::ResetOdom::Response> response) 
      {

        if (request->reset_odom==true)
        {
          try {
            global_local_tf = TfSubscriberNode::tf_buffer_.lookupTransform( "drone_link","map",tf2::TimePointZero, tf2::durationFromSec(0.041));
          } catch (tf2::TransformException &ex) {
            // Handle exception if the transform is not available
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("tf2_example"), ex.what());
            return;
          }

          std::cout<<"Reset request recieved, currently no code is written"<<std::endl;
          // global_local_tf.transform.rotation.x = 0;
          // global_local_tf.transform.rotation.y = 0;
          // global_local_tf.transform.rotation.z = 0;
          // global_local_tf.transform.rotation.w = 1;
          global_local_tf.header.stamp = this->now();
          global_local_tf.header.frame_id = "map";
          global_local_tf.child_frame_id = "global_map_link";
          tf_broadcaster_.sendTransform(global_local_tf);
          response->success=true;
        }

  }

void TfSubscriberNode::reset_srv_handle(){

    reset_odom_srv=this->create_service<interfaces::srv::ResetOdom>("reset_odom", 
        std::bind(&TfSubscriberNode::reset_odom_callback, this, std::placeholders::_1, std::placeholders::_2));
}

void TfSubscriberNode::setup_global_local_transform(){
      global_local_tf.header.stamp = this->now();
      global_local_tf.header.frame_id = "map";
      global_local_tf.child_frame_id = "global_map_link";
      global_local_tf.transform.translation.x = 0;
      global_local_tf.transform.translation.y = 0;
      global_local_tf.transform.translation.z = 0;
      global_local_tf.transform.rotation.x = 0;
      global_local_tf.transform.rotation.y = 0;
      global_local_tf.transform.rotation.z = 0;
      global_local_tf.transform.rotation.w = 1;
      tf_broadcaster_.sendTransform(global_local_tf);

      geometry_msgs::msg::TransformStamped global_drone_tf;
      global_drone_tf.header.stamp = this->now();
      global_drone_tf.header.frame_id = "global_map_link";
      global_drone_tf.child_frame_id = "drone_global";
      global_drone_tf.transform.translation.x = 0;
      global_drone_tf.transform.translation.y = 0;
      global_drone_tf.transform.translation.z = 0;
      global_drone_tf.transform.rotation.x = 0;
      global_drone_tf.transform.rotation.y = 0;
      global_drone_tf.transform.rotation.z = 0;
      global_drone_tf.transform.rotation.w = 1;
      tf_broadcaster_.sendTransform(global_drone_tf);


}  

void TfSubscriberNode::base_camera_static_tf(){
    geometry_msgs::msg::TransformStamped rotation_transform;

    // Copy the timestamp from the "camera_link" transform to the "drone_link" transform
    rotation_transform.header.stamp = this->now();

    // Create a transform stamped message to represent the rotation
    
    // rotation_transform.header.stamp = this->now();
    rotation_transform.header.frame_id = "camera_link";// // rename parent acording drone ardupilot frame
    rotation_transform.child_frame_id = "drone_link"; // rename to the 
    //   set translation of frame
    rotation_transform.transform.translation.x = -0.13;  //-0.14425
    rotation_transform.transform.translation.y = 0.0;
    rotation_transform.transform.translation.z = 0.0;

    // Set the rotation to a 90 degree roll around the X-axis
    tf2::Quaternion rotation_quaternion;
    rotation_quaternion.setRPY(0, 0.0, 0.0); // 90 degree roll around the X-axis
    rotation_transform.transform.rotation = tf2::toMsg(rotation_quaternion);
    tf_broadcaster_.sendTransform(rotation_transform);
}

void TfSubscriberNode::global_odom_broadcast(const geometry_msgs::msg::TransformStamped& transform){

    tf_broadcaster_.sendTransform(transform);
}

void TfSubscriberNode::tf_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
  double range_visual= msg->pose.position.z;
  geometry_msgs::msg::TransformStamped global_odom_tf;
  geometry_msgs::msg::TransformStamped odom_tf;
  try {
      odom_tf = TfSubscriberNode::tf_buffer_.lookupTransform( "map","drone_link",tf2::TimePointZero, tf2::durationFromSec(0.041));
    } catch (tf2::TransformException &ex) {
      // Handle exception if the transform is not available
      // RCLCPP_ERROR_STREAM(rclcpp::get_logger("tf2_example"), ex.what());
      return;
    }
    // global_local_tf.transform.rotation.x = 0;
    // global_local_tf.transform.rotation.y = 0;
    // global_local_tf.transform.rotation.z = 0;
    // global_local_tf.transform.rotation.w = 1;
    odom_tf.header.stamp = this->now();
    odom_tf.header.frame_id = "global_map_link";
    odom_tf.child_frame_id = "drone_global";
    tf_broadcaster_.sendTransform(odom_tf);

  try {
    global_odom_tf = TfSubscriberNode::tf_buffer_.lookupTransform( "map","drone_global",tf2::TimePointZero, tf2::durationFromSec(0.041));
    }
    catch (tf2::TransformException &ex) {
      // Handle exception if the transform is not available
      // RCLCPP_ERROR_STREAM(rclcpp::get_logger("tf2_example"), ex.what());
      return;
    }
    // Position transformation from NWU to ENU
    geometry_msgs::msg::PoseStamped local_pose;
    geometry_msgs::msg::PoseStamped global_pose;

    local_pose.header.stamp = odom_tf.header.stamp;
    local_pose.header.frame_id="map";

    global_pose.header.stamp = odom_tf.header.stamp;
    global_pose.header.frame_id="map";



    // Position transformation
    local_pose.pose.position.x = -odom_tf.transform.translation.y;
    local_pose.pose.position.y = odom_tf.transform.translation.x;
    local_pose.pose.position.z = last_range;

    // Quaternion transformation
    tf2::Quaternion localQ(
        odom_tf.transform.rotation.x,
        odom_tf.transform.rotation.y,
        odom_tf.transform.rotation.z,
        odom_tf.transform.rotation.w
    );
    tf2::Quaternion transformQ;
    transformQ.setRPY(0, 0, -M_PI / 2);

    tf2::Quaternion enuLocalQ = transformQ * localQ;

    // Swap roll and pitch
    double roll, pitch, yaw;
    tf2::Matrix3x3(enuLocalQ).getRPY(roll, pitch, yaw);
    double temp = roll;
    roll = pitch;
    pitch = temp;

    // Add minus sign to roll
    roll = -roll;

    // Convert back to quaternion
    tf2::Quaternion correctedQ;
    correctedQ.setRPY(roll, pitch, yaw);

    local_pose.pose.orientation.x = correctedQ.x();
    local_pose.pose.orientation.y = correctedQ.y();
    local_pose.pose.orientation.z = correctedQ.z();
    local_pose.pose.orientation.w = correctedQ.w();
    ///////////////////////////////////TF-NWU --> POSE-ENU/////////////////////////////////////////////////
    //     // Position transformation
    // global_pose.pose.position.x = -global_odom_tf.transform.translation.y;
    // global_pose.pose.position.y = global_odom_tf.transform.translation.x;
    // global_pose.pose.position.z = last_range;

    // // Quaternion transformation
    // tf2::Quaternion globalQ(
    //     global_odom_tf.transform.rotation.x,
    //     global_odom_tf.transform.rotation.y,
    //     global_odom_tf.transform.rotation.z,
    //     global_odom_tf.transform.rotation.w
    // );
    // tf2::Quaternion enuGlobalQ = transformQ * globalQ;

    // global_pose.pose.orientation.x = enuGlobalQ.x();
    // global_pose.pose.orientation.y = enuGlobalQ.y();
    // global_pose.pose.orientation.z = enuGlobalQ.z();
    // global_pose.pose.orientation.w = enuGlobalQ.w();
    ///////////////////////////////////////////////////////////////////////////////////
  try{
    // global_odom_pub_->publish(global_pose);
    local_odom_pub_->publish(local_pose);
    }
    catch (const std::exception &e) {
    std::cerr << "Caught a standard exception: " << e.what() << std::endl;
    }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TfSubscriberNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
