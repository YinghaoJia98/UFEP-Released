#include <vrmapping/Integrator/CellsIntegrator.h>

namespace CellsIntegratorGPUNS
{
    __device__ __constant__ double ResolutionInDevice_;
    double ResolutionInHost_;

    // Function to set the resolution (must be a host function)
    void setResolution(double Resolution)
    {
        cudaMemcpyToSymbol(ResolutionInDevice_, &Resolution, sizeof(double));
        ResolutionInHost_ = Resolution;
        // HostPrintResolutionInDevice();
    }

    // Kernel to check the value on the device
    __global__ void printResolutionInDevice()
    {
        printf("ResolutionInDevice_ is %f\n", ResolutionInDevice_);
    }

    // __device__ function for calculating X index
    __device__ int CalculateXIndex(double x, double minX)
    {
        return static_cast<int>((x - minX) / ResolutionInDevice_);
    }

    // __device__ function for calculating Y index
    __device__ int CalculateYIndex(double y, double minY)
    {
        return static_cast<int>((y - minY) / ResolutionInDevice_);
    }

    // CUDA Kernel for integrating cells
    __global__ void integrateCellsKernel(double *OutputTraversability, double *OutputElevation, const PlannerCell *d_Cells, int numCells,
                                         int numRows, int numCols, double minX, double minY)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;

        if (idx < numCells)
        {
            // Access individual elements of PlannerCell (Eigen::Vector4d)
            double x = d_Cells[idx][0];
            double y = d_Cells[idx][1];
            // printf("x is %f.", x);
            // printf("y is %f.", y);
            double ElevationTem_ = d_Cells[idx][2];
            double TraversabilityTem_ = d_Cells[idx][3];

            // Calculate grid indices using the custom functions
            int row = CalculateYIndex(y, minY);
            int col = CalculateXIndex(x, minX);

            if (row >= 0 && row < numRows && col >= 0 && col < numCols)
            {
                OutputTraversability[row * numCols + col] = TraversabilityTem_;
                OutputElevation[row * numCols + col] = ElevationTem_;
            }
        }
    }

    // Function to integrate cells on the GPU
    void integrateCellsGPU(const std::vector<PlannerCell> &Cells1, const std::vector<PlannerCell> &Cells2,
                           double minX, double maxX, double minY, double maxY, std::vector<PlannerCell> &integratedCells)
    {
        // std::cout << "Max and Min X are" << maxX << " and " << minX << std::endl;
        // std::cout << "Max and Min Y are" << maxY << " and " << minY << std::endl;
        // std::cout << "ResolutionInDevice_ is " << ResolutionInDevice_ << std::endl;
        // std::cout << "ResolutionInHost_ is " << ResolutionInHost_ << std::endl;
        int numRows = static_cast<int>(std::ceil((maxY - minY) / ResolutionInHost_) + 1);
        int numCols = static_cast<int>(std::ceil((maxX - minX) / ResolutionInHost_) + 1);
        // std::cout << "numRows is " << numRows << std::endl;
        // std::cout << "numCols is " << numCols << std::endl;
        int numCells1 = Cells1.size();
        int numCells2 = Cells2.size();
        // std::cout << "numCells1 is " << numCells1 << std::endl;
        // std::cout << "numCells2 is " << numCells2 << std::endl;

        // Host and device memory allocation
        std::vector<double> outputTra(numRows * numCols, -1.0); // Default to -1 for uninitialized cells
        std::vector<double> outputEle(numRows * numCols, -1.0); // Default to -1 for uninitialized cells
        // std::cout << "Initially, the size of outputTra is " << outputTra.size() << std::endl;
        // std::cout << "Initially, the size of outputEle is " << outputEle.size() << std::endl;
        PlannerCell *Device_Cells1;
        PlannerCell *Device_Cells2;
        double *Device_OutputTraversability_;
        double *Device_OutputElevation_;

        cudaMalloc(&Device_Cells1, numCells1 * sizeof(PlannerCell));
        cudaMalloc(&Device_Cells2, numCells2 * sizeof(PlannerCell));
        cudaMalloc(&Device_OutputTraversability_, numRows * numCols * sizeof(double));
        cudaMalloc(&Device_OutputElevation_, numRows * numCols * sizeof(double));

        // Copy data to device
        cudaMemcpy(Device_Cells1, Cells1.data(), numCells1 * sizeof(PlannerCell), cudaMemcpyHostToDevice);
        cudaMemcpy(Device_Cells2, Cells2.data(), numCells2 * sizeof(PlannerCell), cudaMemcpyHostToDevice);
        cudaMemcpy(Device_OutputTraversability_, outputTra.data(), numRows * numCols * sizeof(double), cudaMemcpyHostToDevice);
        cudaMemcpy(Device_OutputElevation_, outputEle.data(), numRows * numCols * sizeof(double), cudaMemcpyHostToDevice);

        // Kernel execution for Cells2 first
        int threadsPerBlock = 256;
        int blocksPerGrid = (numCells2 + threadsPerBlock - 1) / threadsPerBlock;
        integrateCellsKernel<<<blocksPerGrid, threadsPerBlock>>>(Device_OutputTraversability_, Device_OutputElevation_, Device_Cells2, numCells2, numRows, numCols, minX, minY);
        cudaDeviceSynchronize();

        // Kernel execution for Cells1, which will overwrite conflicts
        blocksPerGrid = (numCells1 + threadsPerBlock - 1) / threadsPerBlock;
        integrateCellsKernel<<<blocksPerGrid, threadsPerBlock>>>(Device_OutputTraversability_, Device_OutputElevation_, Device_Cells1, numCells1, numRows, numCols, minX, minY);
        cudaDeviceSynchronize();

        // Copy result back to host
        cudaMemcpy(outputTra.data(), Device_OutputTraversability_, numRows * numCols * sizeof(double), cudaMemcpyDeviceToHost);
        cudaMemcpy(outputEle.data(), Device_OutputElevation_, numRows * numCols * sizeof(double), cudaMemcpyDeviceToHost);

        // std::cout << "outputTra.size is " << outputTra.size() << std::endl;
        // std::cout << "outputEle.size is " << outputEle.size() << std::endl;
        // Transform d_output into std::vector<PlannerCell>
        for (int i = 0; i < numRows; ++i)
        {
            for (int j = 0; j < numCols; ++j)
            {
                double TraversabilityTem_ = outputTra[i * numCols + j];
                double ElevationTem_ = outputEle[i * numCols + j];
                if (TraversabilityTem_ != -1)
                { // Ignore uninitialized cells
                    // TODO Might need to be rewritten, the 0.5 is a small hot code.
                    double x = minX + (double)(j + 0.5) * ResolutionInHost_;
                    double y = minY + (double)(i + 0.5) * ResolutionInHost_;
                    integratedCells.emplace_back(x, y, ElevationTem_, TraversabilityTem_);
                }
            }
        }
        // std::cout << "integratedCells.size is " << integratedCells.size() << std::endl;

        // Cleanup
        cudaFree(Device_Cells1);
        cudaFree(Device_Cells2);
        cudaFree(Device_OutputTraversability_);
        cudaFree(Device_OutputElevation_);
    }

    void HostPrintResolutionInDevice()
    {
        // Launch a kernel to print the resolution from the device side
        printResolutionInDevice<<<1, 1>>>();
        cudaDeviceSynchronize();
    }
} // namespace CellsIntegratorGPUNS
