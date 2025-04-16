#include "vrmapping/partition_helper.h"
namespace partition_helper
{
  // std::vector<std::vector<int>> &matrix
  void partitionGridHelper(Eigen::MatrixXi matrix,
                           int x_start, int y_start, int size,
                           int threshold, std::vector<PlannerGrid> &grids,
                           Eigen::Vector2d StartPosition,
                           double cell_resolution,
                           double valid_threshold,
                           std::vector<PlannerGrid> &ObstacleGrids,
                           Eigen::MatrixXd ElevationMatrix)
  {
    int occupiedCount = 0;
    std::vector<PlannerCell> subGridCells;
    subGridCells.clear();
    int valid_point_number_ = 0;

    for (int x = x_start; x < x_start + size; x++)
    {
      for (int y = y_start; y < y_start + size; y++)
      {
        if (matrix(x, y) != 2)
        {
          PlannerCell cell;
          cell[0] = StartPosition[0] + x * cell_resolution;
          cell[1] = StartPosition[1] + y * cell_resolution;
          cell[2] = ElevationMatrix(x, y);
          cell[3] = matrix(x, y);
          subGridCells.push_back(cell);
        }
        else
        {
          continue;
        }

        if (matrix(x, y) == 1)
        {
          occupiedCount++;
        }
        if (matrix(x, y) == 0)
        {
          valid_point_number_++;
        }
      }
    }

    if ((occupiedCount == 0) && ((double)(valid_point_number_) >= valid_threshold * (double)(size) * (double)(size)))
    {
      PlannerGrid grid;
      double x_index_double = (double)(x_start) + (double)(size) / 2;
      grid.centerX = x_index_double * cell_resolution + StartPosition[0];
      double y_index_double = (double)(y_start) + (double)(size) / 2;
      grid.centerY = y_index_double * cell_resolution + StartPosition[1];
      grid.size = (double)(size)*cell_resolution;
      grid.cells = subGridCells;
      bool IfUpdateZ_;
      IfUpdateZ_ = grid.UpdateCenterZ();
      if (IfUpdateZ_)
      {
        grids.push_back(grid);
      }
      else
      {
        std::cout << "////////////////ERROR////////////////"
                  << "Faild to updateZ"
                  << std::endl;
      }
    }
    else if ((size / 2 < threshold) && (occupiedCount > 0))
    {
      // obstacle;
      PlannerGrid grid;
      double x_index_double = (double)(x_start) + (double)(size) / 2;
      grid.centerX = x_index_double * cell_resolution + StartPosition[0];
      double y_index_double = (double)(y_start) + (double)(size) / 2;
      grid.centerY = y_index_double * cell_resolution + StartPosition[1];
      grid.size = (double)(size)*cell_resolution;
      grid.cells = subGridCells;
      bool IfUpdateZ_;
      IfUpdateZ_ = grid.UpdateCenterZ();
      if (IfUpdateZ_)
      {
        ObstacleGrids.push_back(grid);
      }
      else
      {
        std::cout << "////////////////ERROR////////////////"
                  << "Faild to updateZ"
                  << std::endl;
      }
    }
    else if (size / 2 >= threshold)
    {
      partitionGridHelper(matrix, x_start, y_start, size / 2, threshold, grids, StartPosition,
                          cell_resolution, valid_threshold, ObstacleGrids, ElevationMatrix);
      partitionGridHelper(matrix, x_start, y_start + size / 2, size / 2, threshold, grids,
                          StartPosition, cell_resolution, valid_threshold, ObstacleGrids,
                          ElevationMatrix);
      partitionGridHelper(matrix, x_start + size / 2, y_start, size / 2, threshold, grids,
                          StartPosition, cell_resolution, valid_threshold, ObstacleGrids,
                          ElevationMatrix);
      partitionGridHelper(matrix, x_start + size / 2, y_start + size / 2, size / 2, threshold,
                          grids, StartPosition, cell_resolution, valid_threshold, ObstacleGrids,
                          ElevationMatrix);
    }
  }

  std::vector<PlannerGrid> partitionGrid(std::vector<PlannerCell> &cells,
                                         double size, double threshold,
                                         Eigen::Vector2d CenterPosition,
                                         double cell_resolution,
                                         double valid_threshold,
                                         std::vector<PlannerGrid> &ObstacleGrids)
  {
    std::vector<PlannerGrid> grids;
    int size_int = (int)(size / cell_resolution);

    // Pre-processing cells into a 2D array for faster access
    // std::vector<std::vector<int>> matrix(size_int, std::vector<int>(size_int, 0));
    // Eigen::MatrixXi matrix(size_int, size_int);
    Eigen::MatrixXi matrix = Eigen::MatrixXi::Constant(size_int, size_int, 2);
    // matrix.resize(size_int, size_int);

    Eigen::MatrixXd ElevationMatrix = Eigen::MatrixXd::Constant(size_int, size_int, 0.0);

    double start_position_x = CenterPosition[0] - size / 2;
    double start_position_y = CenterPosition[1] - size / 2;

    // std::cout << "CenterPosition is " << CenterPosition << std::endl;

    for (size_t CellIndex_ = 0; CellIndex_ < cells.size(); ++CellIndex_)
    {
      int x_index = (int)((cells[CellIndex_][0] - start_position_x) / cell_resolution);
      int y_index = (int)((cells[CellIndex_][1] - start_position_y) / cell_resolution);
      // std::cout << "cell is " << cells[CellIndex_][0] << " "
      //           << cells[CellIndex_][1] << " "
      //           << cells[CellIndex_][2] << " "
      //           << cells[CellIndex_][3] << " "
      //           << std::endl;

      if (x_index < 0)
      {
        std::cout << "CenterPosition is " << CenterPosition << std::endl;
        std::cout << "x_index < 0 and x_index is " << x_index << std::endl
                  << "cells[CellIndex_][0] is " << cells[CellIndex_][0] << " start_position_x is " << start_position_x << std::endl;
      }
      if (y_index < 0)
      {
        std::cout << "CenterPosition is " << CenterPosition << std::endl;
        std::cout << "y_index < 0 and y_index is " << y_index << std::endl
                  << "cells[CellIndex_][1] is " << cells[CellIndex_][1] << " start_position_y is " << start_position_y << std::endl;
      }
      if (x_index == size_int)
      {
        x_index = size_int - 1;
      }

      if (y_index == size_int)
      {
        y_index = size_int - 1;
      }
      ElevationMatrix(x_index, y_index) = cells[CellIndex_][2];
      if (cells[CellIndex_][3] > 0)
      {

        matrix(x_index, y_index) = 1;
      }
      else
      {

        matrix(x_index, y_index) = 0;
      }
    }

    Eigen::Vector2d StartPosition_(start_position_x, start_position_y);
    int threshold_int = int(threshold / cell_resolution);
    grids.clear();
    ObstacleGrids.clear();
    partitionGridHelper(matrix, 0, 0, size_int, threshold_int,
                        grids, StartPosition_, cell_resolution, valid_threshold, ObstacleGrids,
                        ElevationMatrix);
    return grids;
  }
}