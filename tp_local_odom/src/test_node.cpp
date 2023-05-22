#include "tp_local_odom/test_node.hpp"


TfSubscriberNode::TfSubscriberNode(): Node("tf_subscriber_node"),tf_broadcaster_(this), tf_buffer_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME), tf2::durationFromSec(0)),
      tf_listener_(tf_buffer_)
{
  this->setup_subscriber();
  this->base_camera_static_tf();
  this->setup_global_local_transform();
  this->reset_srv_handle();
  
}

void TfSubscriberNode::setup_subscriber(){
    subscription_ = create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", rclcpp::SensorDataQoS(), std::bind(&TfSubscriberNode::tf_callback, this, std::placeholders::_1));
        }

void TfSubscriberNode::reset_odom_callback(
      const std::shared_ptr<interfaces::srv::ResetOdom::Request> request,
      std::shared_ptr<interfaces::srv::ResetOdom::Response> response) 
      {

        if (request->reset_odom==true)
        {
          try {
            global_local_tf = TfSubscriberNode::tf_buffer_.lookupTransform( "drone_link","map",tf2::TimePointZero);
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
      global_local_tf.header.frame_id = "drone_link";
      global_local_tf.child_frame_id = "global_map_link";
      global_local_tf.transform.translation.x = 0;
      global_local_tf.transform.translation.y = 0;
      global_local_tf.transform.translation.z = 0;
      global_local_tf.transform.rotation.x = 0;
      global_local_tf.transform.rotation.y = 0;
      global_local_tf.transform.rotation.z = 0;
      global_local_tf.transform.rotation.w = 1;
      tf_broadcaster_.sendTransform(global_local_tf);
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
    rotation_transform.transform.translation.x = -0.14425;
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

void TfSubscriberNode::tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg){
geometry_msgs::msg::TransformStamped global_drone_tf;
  for (const auto& transform : msg->transforms) {

    if (transform.child_frame_id == "camera_link") {
        try {
            global_drone_tf = TfSubscriberNode::tf_buffer_.lookupTransform( "map","drone_link",tf2::TimePointZero);
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
          global_drone_tf.header.stamp = this->now();
          global_drone_tf.header.frame_id = "global_map_link";
          global_drone_tf.child_frame_id = "drone_global";
          tf_broadcaster_.sendTransform(global_drone_tf);

    }

  // for (const auto& transform : msg->transforms) {
    // if (transform.child_frame_id == "camera_link") {
        // global_local_tf.header.stamp = this->now();
        // global_local_tf.header.frame_id = "map";
        // global_local_tf.child_frame_id = "global_link";
        // global_local_tf.transform.translation.x = -transform.transform.translation.x;
        // global_local_tf.transform.translation.y = -transform.transform.translation.y;
        // global_local_tf.transform.translation.z = -transform.transform.translation.z;
        // tf2::Quaternion quat(
        //     transform.transform.rotation.x,
        //     transform.transform.rotation.y,
        //     transform.transform.rotation.z,
        //     transform.transform.rotation.w
        // );
        // tf2::Matrix3x3 mat(quat);
        // double roll, pitch, yaw;
        // mat.getRPY(roll, pitch, yaw);
        // quat.setRPY(0.0, 0.0, -yaw);
        // global_local_tf.transform.rotation = tf2::toMsg(quat);
        //std::cout<<global_local_tf<<std::endl;
    // }
    // if (transform.child_frame_id == "camera_link") {
      
    //   RCLCPP_INFO(get_logger(), "Transform from '%s' to '%s' with time stamp: %f", transform.header.frame_id.c_str(), transform.child_frame_id.c_str(), transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9);
    //   RCLCPP_INFO(get_logger(), "Received transform message with time stamp:%s:  %f",transform.child_frame_id, transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9);
    //   // ardupilot_frame_broadcaster(transform);
    //   // global_odom_broadcast(transform);
    // }
   }
 }

void TfSubscriberNode::ardupilot_frame_broadcaster(const geometry_msgs::msg::TransformStamped& transform){
      geometry_msgs::msg::TransformStamped rotation_transform;

      // Copy the timestamp from the "camera_link" transform to the "drone_link" transform
      rotation_transform.header.stamp = transform.header.stamp;

      // Create a transform stamped message to represent the rotation
      
      // rotation_transform.header.stamp = this->now();
      rotation_transform.header.frame_id = "camera_link";// // rename parent acording drone ardupilot frame
      rotation_transform.child_frame_id = "ardupilot_link"; // rename to the 
      //   set translation of frame
      rotation_transform.transform.translation.x = -0.14425;
      rotation_transform.transform.translation.y = 0.0;
      rotation_transform.transform.translation.z = 0.0;

      // Set the rotation to a 90 degree roll around the X-axis
      tf2::Quaternion rotation_quaternion;
      rotation_quaternion.setRPY(0, 0.0, 0.0); // 90 degree roll around the X-axis
      rotation_transform.transform.rotation = tf2::toMsg(rotation_quaternion);
      tf_broadcaster_.sendTransform(rotation_transform);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TfSubscriberNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
