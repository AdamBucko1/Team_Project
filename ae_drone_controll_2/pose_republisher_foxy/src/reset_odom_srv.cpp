#include <rclcpp/rclcpp.hpp>
#include "interfaces/srv/reset_odom.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("reset_odom_client");

  auto client = node->create_client<interfaces::srv::ResetOdom>("reset_odom");

  auto request = std::make_shared<interfaces::srv::ResetOdom::Request>();
  request->reset_odom = true;

  // Send the request to the service
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(node->get_logger(), "Waiting for odom_reset service to become available...");
  }
  auto result_future = client->async_send_request(request);

  // Wait for the result
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::executor::FutureReturnCode::SUCCESS) 
  {
    auto result = result_future.get();
    if (result->success)
    {
      RCLCPP_WARN(node->get_logger(), "Local odometry reset succesful  ");
    }
    else
    {
      RCLCPP_ERROR(node->get_logger(), "Local odometry reset unsuccesful ");
    }
  } 
  else 
    RCLCPP_ERROR(node->get_logger(), "Service call timed out");  

  rclcpp::shutdown();
  return 0;
}