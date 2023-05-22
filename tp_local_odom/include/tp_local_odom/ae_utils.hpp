#include <sstream>
#include <iomanip>
#include <boost/date_time/c_local_time_adjustor.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
//#include <tf2_ros/t>
#include <chrono>
#include <memory>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#define M_PI           3.14159265358979323846  /* pi */
#define RAD_TO_DEG(radians) ((radians) * 180.0 / M_PI)
#define DEG_TO_RAD(degrees) ((degrees) * M_PI / 180.0)
#define SQ(value) ((value) * (value))

#define LIMIT_OUT(value, limit, margin) ((value) < (limit.min - (margin)) || (value) > (limit.max + (margin)))

class Utils
{
    public:
        static double YawFromQuaternion(tf2::Quaternion &q);
};
