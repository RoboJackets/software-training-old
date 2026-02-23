#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <stsl_interfaces/msg/mineral_deposit_array.hpp>
#include <stsl_interfaces/srv/reset_mineral_deposit_tracking.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "kalman_filter.hpp"

namespace mineral_deposit_tracking
{
class MineralDepositTracker : public rclcpp::Node
{
public:
    explicit MineralDepositTracker(const rclcpp::NodeOptions & options)
    : rclcpp::Node("mineral_deposit_tracker", options),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_),
    
}


}