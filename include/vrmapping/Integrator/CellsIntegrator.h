#ifndef INTEGRATOR_CELLSINTEGRATOR_H_
#define INTEGRATOR_CELLSINTEGRATOR_H_
#include <vector>
#include <eigen3/Eigen/Dense>
#include <vrmapping/partition_helper.h>
#include <vrmapping/Integrator/CellsIntegratorUtils.cuh>
namespace CellsIntegratorGPUNS
{
    void setResolution(double Resolution);
    void integrateCellsGPU(const std::vector<PlannerCell> &Cells1, const std::vector<PlannerCell> &Cells2,
                           double minX, double maxX, double minY, double maxY, std::vector<PlannerCell> &integratedCells);
    void HostPrintResolutionInDevice();
} // namespace CellsIntegratorGPUNS
#endif /* INTEGRATOR_CELLSINTEGRATOR_H_ */
