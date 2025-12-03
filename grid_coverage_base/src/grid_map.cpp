#include "grid_coverage_base/grid_map.hpp"
#include <algorithm>
#include <cmath>

namespace grid_coverage_base
{

GridMap::GridMap()
: workspace_min_x(0.0),
  workspace_max_x_(0.0),
  workspace_min_y_(0.0),
  workspace_max_y_(0.0),
  cell_size_(0.1),
  grid_width_(0),
  grid_height_(0),
  visited_count_(0),
  total_cells_(0)
{
}

void GridMap::initialize(
    double workspace_min_x, double workspace_max_x,
    double workspace_min_y, double workspace_max_y,//function params
    double cell_size)

    workspace_min_x_ = workspace_min_x;
    workspace_max_x_ = workspace_max_x; //store params into member variable
    workspace_min_y_ = workspace_min_y;
    workspace_max_y_ = workspace_max_y;
    cell_size_ = cell_size;
  
    // calculate grid dimensions
    grid_width_ = static_cast<int>(std::ceil((workspace_max_x - workspace_min_x) / cell_size_));
    grid_height_ = static_cast<int>(std:ceil((workspace_max_y - workspace_min_y) / cell_size_));

    // initialize grid with UNKNOWN state
    grid_.resize(grid_height_);
    for (int y = 0; y < grid_height; ++y) {
        grid_[y].resize(grid_width_, CellState::UNKNOWN);
    }

    // mark boundary cells as WALL - two cells deep
    for (int y = 0; y < grid_height_; ++y) {
        for (int x = 0; x < grid_width_; ++x) {
            // check if cell is within 2 cells of any boundary
            bool is_wall = false;

            // left edge : x < 2
            if (x < 2) is_wall = true;
            // right edge x >= grid_width - 2
            else if (x >= grid_width_ - 2) is_wall = true;
            // bottom edge y < 2
            else if (y < 2) is_wall = true;
            // top edge y >= grid_height - 2
            else if (y >= grid_height_ - 2) is_wall = true;

            if (is_wall) {
                grid_[y][x] = CellState::WALL;
            }
        }
    }    

    // count total explorable cells (not walls)
    total_cells = 0;
    for (int y = 0; y < grid_height_; ++y) {
        for (int x = 0; x < grid_width_; ++x) {
            if (grid_[y][x] != CellState::WALL) {
                total_cells_++;
            }
        }
    }

    visited_count_ = 0;
}

std::pair<int, int> GridMap::worldToGrid(double world_x, double world_y) const
{
    int grid_x = static_cast<int>(std::floor((world_x - workspace_min_x_) / cell_size_));
    int grid_y = static_cast<int>(std::floor((world_y - workspace_min_y_) / cell_size_));

    // clamp to valid range
    grid_x = std::max(0, std::min(grid_x, grid_width_ - 1));
    grid_y = std::max(0, std::min(grid_y, grid_height_ - 1));

    return {grid_x, grid_y};
}

std::pair<double, double> GridMap::gridToWorld(int grid_x, int grid_y) const
{
    // clamp coordinates
    grid_x = std::max(0, std::min(grid_x, grid_width_ - 1));
    grid_y = std::max(0, std::min(grid_y, grid_height - 1));

    // return center of cell
    double world_x = workspace_min_x_ + (grid_x + 0.5) * cell_size_;
    double world_y = workspace_min_y_ + (grid_y + 0.5) * cell_size_;

    return {world_x, world_y};
}
    
CellState GridMap::getCellState(int grid_x, int grid_y) const
{
    if (!isInBounds(grid_x, grid_y)) {
        return CellState::WALL // out of bounds = wall
    }
    return grid_[grid_y][grid_x];
}

void GridMap::setCellState(int grid_x, int grid_y, CellState state)
{
    if (!isInbounds(grid_x, grid_y)) {
        return;
    }

    CellState old_state = grid_[grid_y][grid_x];
    grid_[grid_y][grid_x] = state;

    // update visited content
    if (old_state == CellState::VISITED && state != CellState::VISITED) {
        visited_count_--;
    } else if (old_state != CellState::VISITED && state == CellState::VISITED) {
        visited_count_++;
    }
}

void GridMap::markVisited(double world_x, double world_y)
{
    auto [grid_x, grid_y] = worldToGrid(world_x, world_y);
    if (isInBounds(grid_x, grid_y)) {
        CellState current = getCellState(grid_x, grid_y);
        if (current != CellState::VISITED && current != CellState::WALL) {
            setCellState(grid_x, grid_y, CellState::VISITED);
        }
    }
}

void GridMap::markVisitedRadius(double world_x, double world_y, double radius)
{// Tomasz's assumption: if robot radius of 11cm covers the center of another cell, that cell is visited... reasonable?
    // marka ll cells within radius as visited
    auto [center_x, center_y] = worldToGrid(world_x, world_y);

    int radius_cells = static_cast<int>(std::ceil(radius / cell_size));
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dx = -radius_cells, dx <= radius_cells; ++dx) {
            int grid_x = center_x + dx;
            int grid_y = center_y + dy;

            if (isInBounds(grid_x, grid_y)) {
                // check if cell center is within radius 
                auto [cell_world_x, cell_world_y] = gridToWorld(grid_x, grid_y);
                double dist = std::sqrt(
                    std::pow(cell_world_x - world_x, 2) +
                    std::pow(cell_world_y - world_y, 2));
                
                if (dist <= radius) {
                    cellState current = getCellState(grid_x, grid_y);
                    if (current != CellState::VISITED && current != CellState::WALL) {
                        setCellState(grid_x, grid_y, CellState::VISITED);
                    }
                }
            }
        }
    }
}



