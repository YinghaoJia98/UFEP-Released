#ifndef INTEGRATOR_INTEGRATORHTILS_H_
#define INTEGRATOR_INTEGRATORHTILS_H_
#include <algorithm>
#include <array>
#include <atomic>
#include <vector>
#include <Eigen/Core>
#include <vrmapping/partition_helper.h>
#include <vrmapping/GraphBase.h>
#include <unordered_map>
#include <vrmapping/VRGraphManager.h>
namespace IntegratorNS
{
    /*
     * Small class that can be used by multiple threads that need mutually exclusive
     * indexes to the same array, while still covering all elements.
     */
    class ThreadSafeIndex
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        // NOTE: The ThreadSafeIndex base destructor must be marked virtual.
        //       Otherwise the destructors of derived classes don't get called when
        //       derived class instances are destructed through base class pointers.
        //       This would result leaking memory due to derived class member
        //       variables not being freed.
        virtual ~ThreadSafeIndex() = default;

        /// returns true if index is valid, false otherwise
        bool getNextIndex(VOXEL_LOC *idx);

        void reset();

    protected:
        explicit ThreadSafeIndex(size_t number_of_points);

        virtual VOXEL_LOC getNextIndexImpl(size_t base_idx) = 0;

        std::atomic<size_t> atomic_idx_;
        const size_t number_of_points_;
    };

    /*
     * The class attempts to ensure that the points are read in an order that gives
     * good coverage over the pointcloud very quickly. This is so that the
     * integrator can be terminated before all points have been read (due to time
     * constraints) and still capture most of the geometry.
     */
    class MixedThreadSafeIndex : public ThreadSafeIndex
    {
    public:
        explicit MixedThreadSafeIndex(size_t number_of_points);

    protected:
        virtual VOXEL_LOC getNextIndexImpl(size_t sequential_idx);

    private:
        const size_t number_of_groups_;

        /// 1024 bins
        static constexpr size_t num_bits = 10;
        static constexpr size_t step_size_ = 1 << num_bits;
        static constexpr size_t bit_mask_ = step_size_ - 1;
    };

    /*
     * This class will sort the indices such that the nearest points are integrated
     * first. This has two favorable effects. First, in case the integration reaches
     * the time limit, we are more likely to have integratedg geometry in the robots
     * immediate vacinity, which is more relevant for planning/collision avoidance.
     * The other reason is that the FastTsdfIntegrator will behave nicer and is less
     * likely to destroy small features in the geometry.
     */
    class FastSimpleThreadSafeIndex : public ThreadSafeIndex
    {
    public:
        explicit FastSimpleThreadSafeIndex(const std::unordered_map<VOXEL_LOC, std::vector<PlannerCell>> &GridClusterVector);

    protected:
        virtual VOXEL_LOC getNextIndexImpl(size_t sequential_idx);

    private:
        std::vector<std::pair<size_t, VOXEL_LOC>> indices_and_keys_;
    };

    class ThreadSafeIndexFactory
    {
    public:
        static ThreadSafeIndex *get(const std::string &mode, const std::unordered_map<VOXEL_LOC, std::vector<PlannerCell>> &GridClusterVector);
    };

    bool IfVertexExistInGraphManager(StateVec state_new,
                                     std::vector<Vertex *> *nearest_vertices,
                                     std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                     double MapResolution,
                                     double LimitBoxZ);

    void GenerateCellsByPositionAndResolution(std::vector<PlannerCell> &CellsOutPut,
                                              Eigen::Vector3d CenterPosition,
                                              double resolution,
                                              double CellResolution,
                                              bool IfFillWithTraversableGrid);
    void AddGridsToGraphManager(std::vector<PlannerGrid> Grids,
                                std::shared_ptr<VRGraphManager> GlobalGraphForMultiResolutionMap,
                                bool IfTraversable,
                                double VRMapResolutionMax,
                                double VRMapResolutionMin,
                                double ElevationMapResolution,
                                double LimitBoxZForConnectEdge);

} // namespace IntegratorNS
#endif /* INTEGRATOR_INTEGRATORHTILS_H_ */
