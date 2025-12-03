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

  /**
   * Convert world coordinates to grid cell indices
   * @param world_x X coordinate in meters
   * @param world_y Y coordinate in meters
   * @return Pair of (grid_x, grid_y) indices
   */
   std::pair<int, int> worldToGrid(double world_x, double world_y) const;

   /**
    * Convert grid cell indices to world coordinates (center of cell)
    * @param grid_x Grid x index
    * @param grid_y Grid y index
    * @return Pair of (world_x, world_y) in meters
    */
   std::pair<double, double> gridToWorld(int grid_x, int grid_y) const;
 
   /**
    * Get cell state at grid coordinates
    * @param grid_x Grid x index
    * @param grid_y Grid y index
    * @return CellState enum value
    */
    CellState getCellState(int grid_x, int grid_y) const;

    /**
     * Set cell state at grid coordinates
     * @param grid_x Grid x index
     * @param grid_y Grid y index
     * @param state CellState to set
     */
    void setCellState(int grid_x, int grid_y, CellState state);
  
    /**
     * Mark cell as visited at world coordinates
     * @param world_x X coordinate in meters
     * @param world_y Y coordinate in meters
     */
    void markVisited(double world_x, double world_y);
  
    /**
     * Mark cells within radius as visited (for robot footprint)
     * @param world_x X coordinate in meters
     * @param world_y Y coordinate in meters
     * @param radius Radius in meters
     */
     void markVisitedRadius(double world_x, double world_y, double radius);

     /**
      * Check if grid coordinates are within bounds
      * @param grid_x Grid x index
      * @param grid_y Grid y index
      * @return True if within bounds
      */
     bool isInBounds(int grid_x, int grid_y) const;
   
     /**
      * Check if world coordinates are within workspace bounds
      * @param world_x X coordinate in meters
      * @param world_y Y coordinate in meters
      * @return True if within workspace
      */
     bool isInWorkspace(double world_x, double world_y) const;
   
     /**
      * Get grid dimensions
      * @return Pair of (width, height) in cells
      */
     std::pair<int, int> getGridDimensions() const;
   
     /**
      * Get cell size
      * @return Cell size in meters
      */
     double getCellSize() const;
   
     /**
      * Get workspace bounds
      * @return Tuple of (min_x, max_x, min_y, max_y)
      */
     std::tuple<double, double, double, double> getWorkspaceBounds() const;
   
     /**
      * Get 8-connected neighbors of a cell
      * @param grid_x Grid x index
      * @param grid_y Grid y index
      * @return Vector of (grid_x, grid_y) pairs for valid neighbors
      */
     std::vector<std::pair<int, int>> getNeighbors8(int grid_x, int grid_y) const;
   
     /**
      * Get number of visited cells
      * @return Count of visited cells
      */
     int getVisitedCount() const;
   
     /**
      * Get total number of cells (excluding walls)
      * @return Total count of explorable cells
      */
     int getTotalCells() const;
   
   private:
     double workspace_min_x_;
     double workspace_max_x_;
     double workspace_min_y_;
     double workspace_max_y_;
     double cell_size_;
     int grid_width_;
     int grid_height_;
     
     std::vector<std::vector<CellState>> grid_;
     
     int visited_count_;
     int total_cells_;
     
     /**
      * Clamp grid coordinates to valid range
      */
     void clampGridCoords(int & grid_x, int & grid_y) const;
};

} // namespace grid_coverage_base

#endif // GRID_COVERAGE_BASE__GRID_MAP_HPP_