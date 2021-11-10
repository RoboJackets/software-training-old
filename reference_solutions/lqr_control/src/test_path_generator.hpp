#ifndef TEST_PATH_GENERATOR_HPP_
#define TEST_PATH_GENERATOR_HPP_

#include <nav_msgs/msg/path.hpp>

namespace lqr_control
{

class TestPathGenerator
{
public:

  explicit TestPathGenerator(const std::size_t point_count);

  nav_msgs::msg::Path BuildPath();

private:
  const double scale_ = 0.5;
  const std::size_t point_count_;
  const double t_delta_;
  double t_ = 0.0;

  geometry_msgs::msg::PoseStamped GetNextPoint();

};

}  // namespace lqr_control

#endif  // TEST_PATH_GENERATOR_HPP_