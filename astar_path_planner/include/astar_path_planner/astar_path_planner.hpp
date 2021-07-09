#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <fstream>
#include "astar_path_planner/utils.hpp"

namespace astar_path_planner
{

class AStarPathPlanner
{
public:
    static void DeclareParameters(rclcpp_lifecycle::LifecycleNode::SharedPtr node);

    AStarPathPlanner(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> ros_costmap);

    std::vector<Point> Plan(const Point& start, const Point& goal);

private:
    rclcpp::Logger logger_;
    Point goal_;
    double goal_threshold_;
    double grid_size_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> ros_costmap_;

    bool IsGoal(const Point& point);
    
    std::vector<Point> GetAdjacentPoints(const Point& point);

    double GetPathCost(const std::vector<Point>& path);

    bool PointInCollision(const Point& point);

    void ReducePath(std::vector<Point>& path);
};

}
