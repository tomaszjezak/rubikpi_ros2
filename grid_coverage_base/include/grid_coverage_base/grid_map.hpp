#ifndef GRID_COVERAGE_BASE__GRID_MAP_HPP_
#define GRID_COVERAGE_BASE__GRID_MAP_HPP_

#include <vector>
#include <cmath>

namespace GRID_COVERAGE_BASE__GRID_MAP_HPP_
{
enum class CellState {
        UNKNOWN = 0,
        VISITED = 1,
        WALL = 2,
};

class GridMap
{
public:
  GridMap();

  /**
   * Initialize grid map with workspace bounds and cell size
   * @param workspace_min_x Minimum x coordinate of workspace (meters)
   * @param workspace_max_x Maximum x coordinate of workspace (meters)
   * @param workspace_min_y Minimum y coordinate of workspace (meters)
   * @param workspace_max_y Maximum y coordinate of workspace (meters)
   * @param cell_size Size of each grid cell in meters
   */
  void initialize(
    double workspace_min_x, double workspace_max_x,
    double workspace_min_y, double workspace_max_y,
    double cell_size);

    



}







} // namespace grid_coverage_base

#endif // GRID_COVERAGE_BASE__GRID_MAP_HPP_