#include "tp_local_odom/pose_republisher_foxy.hpp"

/**
 * @brief Constructor for the PoseRepublisher class.
 */
PoseRepublisher::PoseRepublisher()
    : Node("tf_subscriber_node"),
      tf_broadcaster_(this),
      tf_buffer_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME),
                 tf2::durationFromSec(0)),
      tf_listener_(tf_buffer_) {
  DefaultValues();
  StartFunctions();
}

void PoseRepublisher::DefaultValues() {
  rangefinder_offset_ = 0.075;
  camera_offset_x_ = -0.13;
  camera_offset_y_ = -0.061;
  camera_offset_z_ = 0;
  camera_rotation_ = M_PI;
  rangefinder_vis_range_info_ = 0;
  range_visual_ = 0;
  corrected_range_ = 0;
  last_range_ = 0;
}

void PoseRepublisher::StartFunctions() {
  this->LocalOdomPub();
  this->RangefinderSub();
  this->GlobalOdomPub();
  this->PoseVisualSub();
  this->BaseCameraStaticTF();
  this->SetupGlobalTransform();
  this->ResetSrvHandle();
  tf_buffer_.setUsingDedicatedThread(true);  // Optional, for better performance
}

/**
 * @brief Set up the subscriber for visual SLAM pose with covariance.
 */
void PoseRepublisher::PoseVisualSub() {
  pose_visual_sub_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/visual_slam/tracking/vo_pose_covariance", rclcpp::SensorDataQoS(),
          std::bind(&PoseRepublisher::CallbackVisualOdom, this,
                    std::placeholders::_1));
}

/**
 * @brief Set up the subscriber for the rangefinder sensor data.
 */
void PoseRepublisher::RangefinderSub() {
  rangefinder_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
      "/aeros/rangefinder", 3,
      std::bind(&PoseRepublisher::CallbackRangefinder, this,
                std::placeholders::_1));
}

/**
 * @brief Callback function for the rangefinder sensor data.
 * @param msg The received sensor data message.
 */
void PoseRepublisher::CallbackRangefinder(
    const sensor_msgs::msg::Range::SharedPtr msg) {
  if (msg->range == 0.0) {
    corrected_range_ =
        last_range_ + rangefinder_vis_range_info_ - range_visual_;
    last_range_ = corrected_range_;
  } else {
    rangefinder_vis_range_info_ = range_visual_;
    last_range_ = msg->range;
  }
}

/**
 * @brief Publish local odometry data.
 */
void PoseRepublisher::LocalOdomPub() {
  local_odom_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/aeros/vision_odom/pose", 10);
}

/**
 * @brief Publish global odometry data.
 */
void PoseRepublisher::GlobalOdomPub() {
  global_odom_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/aeros/global_position/pose", 10);
}

/**
 * @brief Callback function for resetting odometry.
 * @param request The service request.
 * @param response The service response.
 */
void PoseRepublisher::ResetOdomCallback(
    const std::shared_ptr<interfaces::srv::ResetOdom::Request> request,
    std::shared_ptr<interfaces::srv::ResetOdom::Response> response) {
  if (request->reset_odom == true) {
    try {
      global_local_tf_ = PoseRepublisher::tf_buffer_.lookupTransform(
          "drone_link", "map", tf2::TimePointZero, tf2::durationFromSec(0.041));
    } catch (tf2::TransformException &ex) {
      // Handle exception if the transform is not available
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("tf2_example"), ex.what());
      return;
    }
    // tf2::Quaternion rotation_quaternion;
    // rotation_quaternion.setRPY(0.0, 0.0, M_PI);

    // tf2::Quaternion rotated_orientation =
    //     rotation_quaternion *
    //     tf2::Quaternion(global_local_tf_.transform.rotation.x,
    //                     global_local_tf_.transform.rotation.y,
    //                     global_local_tf_.transform.rotation.z,
    //                     global_local_tf_.transform.rotation.w);
    // rotated_orientation.normalize();

    // global_local_tf_.transform.rotation.x = rotated_orientation.x();
    // global_local_tf_.transform.rotation.y = rotated_orientation.y();
    // global_local_tf_.transform.rotation.z = rotated_orientation.z();
    // global_local_tf_.transform.rotation.w = rotated_orientation.w();

    global_local_tf_.header.stamp = this->now();
    global_local_tf_.header.frame_id = "map";
    global_local_tf_.child_frame_id = "global_map_link";
    tf_broadcaster_.sendTransform(global_local_tf_);
    response->success = true;
  }
}

/**
 * @brief Set up the service for resetting odometry.
 */
void PoseRepublisher::ResetSrvHandle() {
  reset_odom_srv_ = this->create_service<interfaces::srv::ResetOdom>(
      "reset_odom", std::bind(&PoseRepublisher::ResetOdomCallback, this,
                              std::placeholders::_1, std::placeholders::_2));
}

/**
 * @brief Set up the local to global coordinate frame transforms.
 */
void PoseRepublisher::SetupGlobalTransform() {
  global_local_tf_.header.stamp = this->now();
  global_local_tf_.header.frame_id = "map";
  global_local_tf_.child_frame_id = "global_map_link";
  global_local_tf_.transform.translation.x = 0;
  global_local_tf_.transform.translation.y = 0;
  global_local_tf_.transform.translation.z = 0;
  global_local_tf_.transform.rotation.x = 0;
  global_local_tf_.transform.rotation.y = 0;
  global_local_tf_.transform.rotation.z = 0;
  global_local_tf_.transform.rotation.w = 1;

  tf_broadcaster_.sendTransform(global_local_tf_);

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

/**
 * @brief Set up the static transform between the camera and drone frames.
 */
void PoseRepublisher::BaseCameraStaticTF() {
  geometry_msgs::msg::TransformStamped rotation_transform;

  rotation_transform.header.stamp = this->now();
  rotation_transform.header.frame_id = "camera_link";
  rotation_transform.child_frame_id = "drone_link";
  // Set translation of frame
  rotation_transform.transform.translation.x = camera_offset_x_;
  rotation_transform.transform.translation.y = camera_offset_y_;
  rotation_transform.transform.translation.z = camera_offset_z_;

  // Camera is mounted on the back, meaning it's rotated 180 degrees from the
  // drone's heading
  tf2::Quaternion rotation_quaternion;
  rotation_quaternion.setRPY(0, 0.0, camera_rotation_);
  rotation_transform.transform.rotation = tf2::toMsg(rotation_quaternion);
  tf_broadcaster_.sendTransform(rotation_transform);
}

/**
 * @brief Broadcast the global odometry transform.
 * @param transform The global odometry transform.
 */
void PoseRepublisher::GlobalOdomBroadcast(
    const geometry_msgs::msg::TransformStamped &transform) {
  tf_broadcaster_.sendTransform(transform);
}

/**
 * @brief Callback function for processing visual SLAM pose with covariance
 * messages.
 * @param msg The received pose with covariance message.
 */
void PoseRepublisher::CallbackVisualOdom(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  double range_visual = msg->pose.pose.position.z;
  geometry_msgs::msg::TransformStamped global_odom_tf;
  geometry_msgs::msg::TransformStamped odom_tf;
  try {
    odom_tf = PoseRepublisher::tf_buffer_.lookupTransform(
        "map", "drone_link", tf2::TimePointZero, tf2::durationFromSec(0.041));
  } catch (tf2::TransformException &ex) {
    return;
  }
  odom_tf.header.stamp = this->now();
  odom_tf.header.frame_id = "global_map_link";
  odom_tf.child_frame_id = "drone_global";
  tf_broadcaster_.sendTransform(odom_tf);

  try {
    global_odom_tf = PoseRepublisher::tf_buffer_.lookupTransform(
        "map", "drone_global", tf2::TimePointZero, tf2::durationFromSec(0.041));
  } catch (tf2::TransformException &ex) {
    return;
  }

  local_pose_.header.stamp = odom_tf.header.stamp;
  local_pose_.header.frame_id = "map";

  global_pose_.header.stamp = odom_tf.header.stamp;
  global_pose_.header.frame_id = "map";

  TransformNWUToPoseENU(odom_tf, local_pose_);
  TransformNWUToPoseENU(global_odom_tf, global_pose_);

  // Camera is backwards
  RotatePoseAroundAxis(local_pose_, tf2::Vector3(0.0, 0.0, 1.0),
                       camera_rotation_);
  RotatePoseAroundAxis(global_pose_, tf2::Vector3(0.0, 0.0, 1.0),
                       camera_rotation_);

  local_pose_with_covariance_.header = local_pose_.header;
  local_pose_with_covariance_.pose.pose = local_pose_.pose;
  local_pose_with_covariance_.pose.covariance = msg->pose.covariance;
  try {
    global_odom_pub_->publish(global_pose_);
    local_odom_pub_->publish(local_pose_with_covariance_);
  } catch (const std::exception &e) {
    std::cerr << "Caught a standard exception: " << e.what() << std::endl;
  }
}

/**
 * @brief Transform NWU coordinates to ENU coordinates for pose translation.
 * @param odom_tf The odometry transform.
 * @param local_pose_ The local pose to be transformed.
 */
void PoseRepublisher::TransformNWUToPoseENU(
    const geometry_msgs::msg::TransformStamped &odom_tf,
    geometry_msgs::msg::PoseStamped &local_pose_) {
  // Position transformation
  local_pose_.pose.position.x = -odom_tf.transform.translation.y;
  local_pose_.pose.position.y = odom_tf.transform.translation.x;
  local_pose_.pose.position.z = last_range_ - rangefinder_offset_;

  // Quaternion transformation
  tf2::Quaternion localQ(
      odom_tf.transform.rotation.x, odom_tf.transform.rotation.y,
      odom_tf.transform.rotation.z, odom_tf.transform.rotation.w);
  double roll, pitch, yaw, temp;
  tf2::Matrix3x3(localQ).getRPY(roll, pitch, yaw);
  temp = roll;
  roll = -pitch;
  pitch = temp;
  tf2::Quaternion corrected_quaternion;
  corrected_quaternion.setRPY(roll, pitch, yaw);
  local_pose_.pose.orientation = tf2::toMsg(corrected_quaternion);
}

/**
 * @brief Rotate a pose around an axis by a given angle.
 * @param pose The pose to be rotated.
 * @param axis The axis of rotation.
 * @param angle The angle of rotation in radians.
 */
void PoseRepublisher::RotatePoseAroundAxis(
    geometry_msgs::msg::PoseStamped &pose, const tf2::Vector3 &axis,
    double angle) {
  // Convert pose orientation to tf2::Quaternion
  tf2::Quaternion pose_quat;
  tf2::fromMsg(pose.pose.orientation, pose_quat);

  // Create a tf2::Transform from original pose
  tf2::Transform orig_transform(
      pose_quat, tf2::Vector3(pose.pose.position.x, pose.pose.position.y,
                              pose.pose.position.z));

  // Define the rotation quaternion
  tf2::Quaternion rot_quat(axis, angle);

  // Rotate original transform
  tf2::Transform rot_transform(rot_quat);
  tf2::Transform new_transform = rot_transform * orig_transform;

  // Modify the original PoseStamped
  pose.pose.position.x = new_transform.getOrigin().x();
  pose.pose.position.y = new_transform.getOrigin().y();
  pose.pose.position.z = new_transform.getOrigin().z();
  pose.pose.orientation = tf2::toMsg(new_transform.getRotation());
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseRepublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}