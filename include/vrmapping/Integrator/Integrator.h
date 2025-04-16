#ifndef INTEGRATOR_INTEGRATOR_H_
#define INTEGRATOR_INTEGRATOR_H_
#include <algorithm>
#include <atomic>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <Eigen/Core>
// #include <shared_mutex>
#include <vrmapping/Integrator/IntegratorUtils.h>
#include <vrmapping/Integrator/CellsIntegrator.h>

namespace IntegratorNS
{
    enum class IntegratorType : int
    {
        kSimple = 1,
        kFast = 2,
        kFastSimple = 3
    };

    static constexpr size_t kNumIntegratorTypes = 3u;

    const std::array<std::string, kNumIntegratorTypes> kIntegratorTypeNames = {{/*kSimple*/ "simple",
                                                                                /*kFast*/ "fast",
                                                                                /*kFastSimple*/ "FastSimple"}};

    /**
     * Base class to the simple, merged and fast Occupancy integrators. The integrator
     * takes in a pointcloud + pose and uses this information to update the Occupancy
     * information in the given Occupancy layer. Note most functions in this class state
     * if they are thread safe. Unless explicitly stated otherwise, this thread
     * safety is based on the assumption that any pointers passed to the functions
     * point to objects that are guaranteed to not be accessed by other threads.
     */
    class IntegratorBase
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<IntegratorBase> Ptr;

        struct Config
        {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            size_t integrator_threads = std::thread::hardware_concurrency();

            /// Mode of the ThreadSafeIndex, determines the integration order of the
            /// rays. Options: "mixed", "FastSimple"
            std::string integration_order_mode = "FastSimple";

            /// fast integrator specific
            float max_integration_time_s = std::numeric_limits<float>::max();

            std::string print() const;
            double LimitBoxZ_ = 1.5;
            double VRMapResolutionMax_ = 1.28;
            double VRMapResolutionMin_ = 0.16;
            double ElevationMapResolution_ = 0.08;
            double ValidThresholdForMultiMapResolution_ = 0.9;
            double LimitBoxZForConnectEdge_ = 0.25;
        };

        IntegratorBase(const Config &config, std::shared_ptr<VRGraphManager> VRGMPtr);

        virtual void integrateGridCells(const std::unordered_map<VOXEL_LOC, std::vector<PlannerCell>> &GridClusterVector) = 0;

        /// Returns a CONST ref of the config.
        const Config &getConfig() const
        {
            return config_;
        }

        void setMapPtr(std::shared_ptr<VRGraphManager> VRGMPtr);
        void setElevationMapResolutionParam(double ElevationMapResolution);

    protected:
        Config config_;
        void updateMapWithStoredGrids();

        std::shared_ptr<VRGraphManager> VRGMPtr_;

        // std::mutex temp_block_mutex_;
        std::mutex VRGMPtr_Mutex_;
        // mutable std::shared_mutex VRGMPtr_SharedMutex_;
        std::vector<Vertex *> VerticesNeedRemoved_;
        std::mutex GeneratedGrids_Mutex_;
        std::vector<PlannerGrid> GeneratedFreeGrids_;
        std::vector<PlannerGrid> GeneratedObstacleGrids_;

        double LimitBoxZ_;
        double VRMapResolutionMax_;
        double VRMapResolutionMin_;
        double ElevationMapResolution_; // Cell resolution.
        double ValidThresholdForMultiMapResolution_;
        double LimitBoxZForConnectEdge_;
    };

    /// Creates a Occupancy integrator of the desired type.
    class IntegratorFactory
    {
    public:
        static IntegratorBase::Ptr create(const std::string &integrator_type_name, const IntegratorBase::Config &config,
                                          std::shared_ptr<VRGraphManager> VRGMPtr);
        static IntegratorBase::Ptr create(const IntegratorType integrator_type, const IntegratorBase::Config &config,
                                          std::shared_ptr<VRGraphManager> VRGMPtr);
    };

    /**
     * Basic Occupancy integrator. Every point is raycast through all the voxels, which
     * are updated individually. An exact but very slow approach.
     */
    class SimpleIntegrator : public IntegratorBase
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        SimpleIntegrator(const Config &config, std::shared_ptr<VRGraphManager> VRGMPtr) : IntegratorBase(config, VRGMPtr)
        {
        }

        void integrateGridCells(const std::unordered_map<VOXEL_LOC, std::vector<PlannerCell>> &GridClusterVector);

        void integrateFunction(const std::unordered_map<VOXEL_LOC, std::vector<PlannerCell>> &GridClusterVector, ThreadSafeIndex *index_getter);

    protected:
    };

    /**
     * An integrator that prioritizes speed over everything else. Rays are cast from
     * the pointcloud to the sensor origin. If a ray intersects
     * a voxel that has already been updated by other rays from the same cloud,
     * it is terminated early, unless the ray is within the truncation distance.
     * This results in a large reduction in the number of freespace updates and
     * greatly improves runtime while ensuring all voxels receive at least a minimum
     * number of updates. Speed is further enhanced through limiting the number of
     * rays cast from each voxel as set by start_voxel_subsampling_factor and use of
     * the ApproxHashSet. Up to an order of magnitude faster then the other
     * integrators for small voxels.
     */
    class FastIntegrator : public IntegratorBase
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        FastIntegrator(const Config &config, std::shared_ptr<VRGraphManager> VRGMPtr) : IntegratorBase(config, VRGMPtr)
        {
        }

        void integrateFunction(const std::unordered_map<VOXEL_LOC, std::vector<PlannerCell>> &GridClusterVector, ThreadSafeIndex *index_getter);

        void integrateGridCells(const std::unordered_map<VOXEL_LOC, std::vector<PlannerCell>> &GridClusterVector);

    protected:
    private:
        /**
         * uses 2^20 bytes (8 megabytes) of ram per tester
         * A testers false negative rate is inversely proportional to its size
         */
        static constexpr size_t masked_bits_ = 20;

        /// Used in terminating the integration early if it exceeds a time limit.
        std::chrono::time_point<std::chrono::steady_clock> integration_start_time_;
    };

    /**
     * An integrator that prioritizes speed over everything by simplifying free node integration.
     */
    class FastSimpleIntegrator : public IntegratorBase
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        FastSimpleIntegrator(const Config &config, std::shared_ptr<VRGraphManager> VRGMPtr) : IntegratorBase(config, VRGMPtr)
        {
        }

        void integrateFunction(const std::unordered_map<VOXEL_LOC, std::vector<PlannerCell>> &GridClusterVector, ThreadSafeIndex *index_getter);

        void integrateGridCells(const std::unordered_map<VOXEL_LOC, std::vector<PlannerCell>> &GridClusterVector);

    protected:
    private:
        /// Used in terminating the integration early if it exceeds a time limit.
        std::chrono::time_point<std::chrono::steady_clock> integration_start_time_;
    };
} // namespace IntegratorNS
#endif /* INTEGRATOR_INTEGRATOR_H_ */
