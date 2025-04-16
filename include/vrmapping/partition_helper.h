#ifndef PARTITION_HELPER_H_
#define PARTITION_HELPER_H_

#include <string>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>

typedef Eigen::Vector4d PlannerCell; // x,y,z,IfObstacle.
struct PlannerGrid
{
    double centerX, centerY, centerZ, size;
    std::vector<PlannerCell> cells;
    bool UpdateCenterZ()
    {
        double z_out = 0;
        for (PlannerCell &cell : cells)
        {
            z_out = z_out + cell[2];
        }
        if (cells.size() == 0)
        {
            // std::cout << "/****************ERROR*******************/" << std::endl
            //           << "cells.size() is " << cells.size() << std::endl;
            return false;
        }
        else
        {
            z_out = z_out / cells.size();
        }
        if (z_out != z_out)
        {
            return false;
        }

        centerZ = z_out;
        return true;
    }
};

namespace partition_helper
{

    void partitionGridHelper(Eigen::MatrixXi matrix,
                             int x_start, int y_start, int size,
                             int threshold, std::vector<PlannerGrid> &grids,
                             Eigen::Vector2d StartPosition,
                             double cell_resolution,
                             double valid_threshold,
                             std::vector<PlannerGrid> &ObstacleGrids,
                             Eigen::MatrixXd ElevationMatrix);

    std::vector<PlannerGrid> partitionGrid(std::vector<PlannerCell> &cells,
                                           double size, double threshold,
                                           Eigen::Vector2d CenterPosition,
                                           double cell_resolution,
                                           double valid_threshold,
                                           std::vector<PlannerGrid> &ObstacleGrids);
}
#endif
