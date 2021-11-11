#include "test_path_generator.hpp"
#include <cmath>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace lqr_control
{

TestPathGenerator::TestPathGenerator(const std::size_t point_count)
: point_count_(point_count),
  t_delta_((2 * M_PI) / (point_count_))
{
}

nav_msgs::msg::Path TestPathGenerator::BuildPath()
{
  nav_msgs::msg::Path path;
  t_ = 0.0;
  std::generate_n(std::back_inserter(path.poses), point_count_, [this] {return GetNextPoint();});
  return path;
}

geometry_msgs::msg::PoseStamped TestPathGenerator::GetNextPoint()
{
  /* Builds a Gerono lemniscate.
   * Math taken from https://mathworld.wolfram.com/EightCurve.html
   */

  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = scale_ * std::sin(t_);
  pose.pose.position.y = scale_ * std::sin(t_) * std::cos(t_);

  const auto dx = scale_ * std::cos(t_);
  const auto dy = (scale_ * std::cos(t_) * std::cos(t_)) - (scale_ * std::sin(t_) * std::sin(t_));

  pose.pose.orientation = tf2::toMsg(Eigen::Quaterniond{Eigen::AngleAxisd{std::atan2(dy,
          dx),
          Eigen::Vector3d::UnitZ()}});

  t_ += t_delta_;
  return pose;
}

}  // namespace lqr_control
