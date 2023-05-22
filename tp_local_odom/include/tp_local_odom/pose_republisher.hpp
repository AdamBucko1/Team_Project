#include <sensor_msgs/msg/range.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <chrono>
#include <memory>
#include "interfaces/srv/reset_odom.hpp"

class PoseRepublisher : public rclcpp::Node // MODIFY NAME
{
public:

    PoseRepublisher();


 
private:
    void Init();
    void DefaultValues();
    void SetupSubscribers();
    void SetupPublishers();
    void SetupServices();
}
