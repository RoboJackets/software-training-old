#ifndef ASTAR_PATH_PLANNER__PATH_REDUCTION_HPP
#define ASTAR_PATH_PLANNER__PATH_REDUCTION_HPP

#include <vector>
#include "utils.hpp"

namespace astar_path_planner
{


void ReducePath(std::vector<Point> & path);

bool PointsAreCollinear(std::vector<Point>::iterator first, std::vector<Point>::iterator last);

}

#endif  // ASTAR_PATH_PLANNER__PATH_REDUCTION_HPP
