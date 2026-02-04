
#include <amrl_libs/path_planning/PathPlanningFunctions.hpp>
#include <amrl_common/util/bresenham_line.hpp>
#include <amrl_common/util/util.hpp>


namespace amrl {
namespace util {

std::vector<Point<uint32_t>> smooth_path(
  const std::vector<Point<uint32_t>> &path_cells, 
  std::shared_ptr<GridGraph> graph)
{
  std::vector<Point<uint32_t>> smooth_path;
  smooth_path.push_back(path_cells[0]);

  size_t start = 0;
  size_t idx   = 1;
  Point<int32_t> p1 = path_cells[start];

  for(size_t idx = 1; idx < path_cells.size(); ++idx) {
    bool add_point    = false;
    Point<int32_t> p2 = path_cells[idx];

    auto pts = util::bresenham_line(p1, p2);
    for(const auto &p : pts) {
      if(graph->cell_is_occupied(p)) {
        add_point = true;
        break;
      }
    }

    if(add_point && idx > 1) {
      start = idx - 1;
      p1    = path_cells[start];
      smooth_path.push_back(p1);
    }
  }

  smooth_path.push_back(*path_cells.rbegin());
  return smooth_path;
}


}
}
