#include <vrmapping/Integrator/Integrator.h>
namespace IntegratorNS
{
    IntegratorBase::Ptr IntegratorFactory::create(const std::string &integrator_type_name,
                                                  const IntegratorBase::Config &config, std::shared_ptr<VRGraphManager> VRGMPtr)
    {

        int integrator_type = 1;
        for (const std::string &valid_integrator_type_name : kIntegratorTypeNames)
        {
            if (integrator_type_name == valid_integrator_type_name)
            {
                return create(static_cast<IntegratorType>(integrator_type), config, VRGMPtr);
            }
            ++integrator_type;
        }
        ROS_ERROR("Unknown integrator type: %s.", integrator_type_name);

        return IntegratorBase::Ptr();
    }

    IntegratorBase::Ptr IntegratorFactory::create(const IntegratorType integrator_type,
                                                  const IntegratorBase::Config &config, std::shared_ptr<VRGraphManager> VRGMPtr)
    {

        switch (integrator_type)
        {
        case IntegratorType::kSimple:
            return IntegratorBase::Ptr(new SimpleIntegrator(config, VRGMPtr));
            break;
        case IntegratorType::kFast:
            return IntegratorBase::Ptr(new FastIntegrator(config, VRGMPtr));
            break;
        case IntegratorType::kFastSimple:
            return IntegratorBase::Ptr(new FastSimpleIntegrator(config, VRGMPtr));
            break;
        default:
            break;
        }
        return IntegratorBase::Ptr();
    }

    // Note many functions state if they are thread safe. Unless explicitly stated
    // otherwise, this thread safety is based on the assumption that any pointers
    // passed to the functions point to objects that are guaranteed to not be
    // accessed by other threads.

    IntegratorBase::IntegratorBase(const Config &config, std::shared_ptr<VRGraphManager> VRGMPtr) : config_(config)
    {
        setMapPtr(VRGMPtr);

        VerticesNeedRemoved_.clear();
        GeneratedFreeGrids_.clear();
        GeneratedObstacleGrids_.clear();

        if (config_.integrator_threads == 0)
        {
            ROS_INFO("Automatic core count failed, defaulting to 1 threads");
            config_.integrator_threads = 1;
        }
        LimitBoxZ_ = config_.LimitBoxZ_;
        VRMapResolutionMax_ = config_.VRMapResolutionMax_;
        VRMapResolutionMin_ = config_.VRMapResolutionMin_;
        ElevationMapResolution_ = config_.ElevationMapResolution_;
        ValidThresholdForMultiMapResolution_ = config_.ValidThresholdForMultiMapResolution_;
        LimitBoxZForConnectEdge_ = config_.LimitBoxZForConnectEdge_;
        std::cout << config_.print() << std::endl;
        CellsIntegratorGPUNS::setResolution(ElevationMapResolution_);
    }

    void IntegratorBase::setMapPtr(std::shared_ptr<VRGraphManager> VRGMPtr)
    {

        VRGMPtr_ = VRGMPtr;
    }

    void IntegratorBase::setElevationMapResolutionParam(double ElevationMapResolution)
    {
        ElevationMapResolution_ = ElevationMapResolution;
        std::cout << "setElevationMapResolutionParam to " << ElevationMapResolution_ << std::endl;
        CellsIntegratorGPUNS::setResolution(ElevationMapResolution_);
    }

    // NOT thread safe
    void IntegratorBase::updateMapWithStoredGrids()
    {
        std::unique_lock<std::mutex> lock2(GeneratedGrids_Mutex_);
        AddGridsToGraphManager(GeneratedFreeGrids_, VRGMPtr_, true, VRMapResolutionMax_, VRMapResolutionMin_, ElevationMapResolution_, LimitBoxZForConnectEdge_);
        AddGridsToGraphManager(GeneratedObstacleGrids_, VRGMPtr_, false, VRMapResolutionMax_, VRMapResolutionMin_, ElevationMapResolution_, LimitBoxZForConnectEdge_);
        GeneratedFreeGrids_.clear();
        GeneratedObstacleGrids_.clear();
    }

    void SimpleIntegrator::integrateGridCells(const std::unordered_map<VOXEL_LOC, std::vector<PlannerCell>> &GridClusterVector)
    {
        std::unique_ptr<ThreadSafeIndex> index_getter(ThreadSafeIndexFactory::get(config_.integration_order_mode, GridClusterVector));

        std::list<std::thread> integration_threads;
        for (size_t i = 0; i < config_.integrator_threads; ++i)
        {
            integration_threads.emplace_back(&SimpleIntegrator::integrateFunction, this, GridClusterVector,
                                             index_getter.get());
        }

        for (std::thread &thread : integration_threads)
        {
            thread.join();
        }

        updateMapWithStoredGrids();
    }

    void SimpleIntegrator::integrateFunction(const std::unordered_map<VOXEL_LOC, std::vector<PlannerCell>> &GridClusterVector,
                                             ThreadSafeIndex *index_getter)
    {

        VOXEL_LOC point_idx;
        while (index_getter->getNextIndex(&point_idx))
        {
            // // TODO The Diff is a small complentation. Might more precise supplements are needed.
            // const Point &point_C = points_C[point_idx];
            // Point DiffTem_(voxel_size_ / 2, voxel_size_ / 2, voxel_size_ / 2);
            // const Point &PointCAfterRevisedTem_ = point_C - DiffTem_;

            // Block<OccupancyVoxel>::Ptr block = nullptr;
            // BlockIndex block_idx;
            // GlobalIndex global_voxel_idx = getGridIndexFromPoint<GlobalIndex>(PointCAfterRevisedTem_ * voxel_size_inv_);
            // OccupancyVoxel *OccupancyVoxelTem_ = allocateStorageAndGetVoxelPtr(global_voxel_idx, &block, &block_idx);
            // UpdateOccupancyVoxel(IfFreePoints, OccupancyVoxelTem_);
        }
    }

    void FastIntegrator::integrateFunction(const std::unordered_map<VOXEL_LOC, std::vector<PlannerCell>> &GridClusterVector,
                                           ThreadSafeIndex *index_getter)
    {
        VOXEL_LOC point_idx;
        while (index_getter->getNextIndex(&point_idx) && (std::chrono::duration_cast<std::chrono::microseconds>(
                                                              std::chrono::steady_clock::now() - integration_start_time_)
                                                              .count() < config_.max_integration_time_s * 1000000))
        {
            // // TODO The Diff is a small complentation. Might more precise supplements are needed.
            // const Point &point_C = points_C[point_idx];
            // Point DiffTem_(voxel_size_ / 2, voxel_size_ / 2, voxel_size_ / 2);
            // const Point &PointCAfterRevisedTem_ = point_C - DiffTem_;
            // Block<OccupancyVoxel>::Ptr block = nullptr;
            // BlockIndex block_idx;
            // GlobalIndex global_voxel_idx = getGridIndexFromPoint<GlobalIndex>(PointCAfterRevisedTem_ * voxel_size_inv_);
            // OccupancyVoxel *OccupancyVoxelTem_ = allocateStorageAndGetVoxelPtr(global_voxel_idx, &block, &block_idx);
            // UpdateOccupancyVoxel(IfFreePoints, OccupancyVoxelTem_);
        }
    }

    void FastIntegrator::integrateGridCells(const std::unordered_map<VOXEL_LOC, std::vector<PlannerCell>> &GridClusterVector)
    {
        integration_start_time_ = std::chrono::steady_clock::now();

        std::unique_ptr<ThreadSafeIndex> index_getter(ThreadSafeIndexFactory::get(config_.integration_order_mode, GridClusterVector));

        std::list<std::thread> integration_threads;
        for (size_t i = 0; i < config_.integrator_threads; ++i)
        {
            integration_threads.emplace_back(&FastIntegrator::integrateFunction, this, GridClusterVector,
                                             index_getter.get());
        }

        for (std::thread &thread : integration_threads)
        {
            thread.join();
        }

        updateMapWithStoredGrids();
    }

    void FastSimpleIntegrator::integrateFunction(const std::unordered_map<VOXEL_LOC, std::vector<PlannerCell>> &GridClusterVector,
                                                 ThreadSafeIndex *index_getter)
    {

        VOXEL_LOC point_idx;
        while (index_getter->getNextIndex(&point_idx) && (std::chrono::duration_cast<std::chrono::microseconds>(
                                                              std::chrono::steady_clock::now() - integration_start_time_)
                                                              .count() < config_.max_integration_time_s * 1000000))
        {
            Eigen::Vector2d LocalCellsCurrentCenterPositionTem_(point_idx.x,
                                                                point_idx.y);
            std::vector<PlannerCell> CellsNewTem_ = GridClusterVector.at(point_idx);

            double zTem_ = 0.0;
            double zSumTem_ = 0.0;
            for (size_t i = 0; i < CellsNewTem_.size(); i++)
            {
                zSumTem_ = zSumTem_ + CellsNewTem_[i][2];
            }
            zTem_ = zSumTem_ / CellsNewTem_.size();
            StateVec state_in_LocalMultiResolutionMap(point_idx.x,
                                                      point_idx.y,
                                                      zTem_, 0);
            std::vector<Vertex *> nearest_vertices_GlobalMultiResolutionMap;
            nearest_vertices_GlobalMultiResolutionMap.clear();
            std::vector<PlannerCell> OldPlannerCellsAlltem_;
            OldPlannerCellsAlltem_.clear();
            // std::cout << "CellsNewTem_.size is " << CellsNewTem_.size() << std::endl;
            std::unique_lock<std::mutex> lock1(VRGMPtr_Mutex_);
            // std::shared_lock<std::shared_mutex> SharedLock1(VRGMPtr_SharedMutex_);
            if (IfVertexExistInGraphManager(state_in_LocalMultiResolutionMap, &nearest_vertices_GlobalMultiResolutionMap,
                                            VRGMPtr_, VRMapResolutionMax_, LimitBoxZ_))
            {
                for (size_t id_GlobalMultiResolutionMap_middle = 0;
                     id_GlobalMultiResolutionMap_middle < nearest_vertices_GlobalMultiResolutionMap.size();
                     id_GlobalMultiResolutionMap_middle++)
                {
                    Vertex *VertexNeedProcessed_ = nearest_vertices_GlobalMultiResolutionMap[id_GlobalMultiResolutionMap_middle];
                    Eigen::Vector3d CenterPosition_(VertexNeedProcessed_->state[0], VertexNeedProcessed_->state[1],
                                                    VertexNeedProcessed_->state[2]);
                    double VertexResolution_ = VertexNeedProcessed_->resolution;
                    bool IfTraversable_ = !VertexNeedProcessed_->is_obstacle;
                    std::vector<PlannerCell> OldPlannerCellsMiddleTem_;
                    OldPlannerCellsMiddleTem_.clear();
                    GenerateCellsByPositionAndResolution(OldPlannerCellsMiddleTem_, CenterPosition_, VertexResolution_, ElevationMapResolution_,
                                                         IfTraversable_);
                    OldPlannerCellsAlltem_.insert(OldPlannerCellsAlltem_.end(), OldPlannerCellsMiddleTem_.begin(), OldPlannerCellsMiddleTem_.end());
                    // VerticesNeedRemoved_.push_back(VertexNeedProcessed_);
                }
            }
            lock1.unlock();
            // SharedLock1.unlock();

            std::vector<PlannerCell> GeneratedPlannerCellsAlltem_;
            GeneratedPlannerCellsAlltem_.clear();
            CellsIntegratorGPUNS::integrateCellsGPU(CellsNewTem_, OldPlannerCellsAlltem_, point_idx.x - 0.5 * VRMapResolutionMax_, point_idx.x + 0.5 * VRMapResolutionMax_,
                                                    point_idx.y - 0.5 * VRMapResolutionMax_, point_idx.y + 0.5 * VRMapResolutionMax_, GeneratedPlannerCellsAlltem_);

            // std::cout << "OldPlannerCellsAlltem_.size is " << OldPlannerCellsAlltem_.size() << std::endl;
            // std::cout << "GeneratedPlannerCellsAlltem_.size is " << GeneratedPlannerCellsAlltem_.size() << std::endl;

            std::vector<PlannerGrid> FreeGridsForMultiResolutionMapToGlobalGraphTem_;
            FreeGridsForMultiResolutionMapToGlobalGraphTem_.clear();
            std::vector<PlannerGrid> ObstacleGridsForMultiResolutionMapToGlobalGraphTem_;
            ObstacleGridsForMultiResolutionMapToGlobalGraphTem_.clear();
            // ROSTIME StartTimeTem4_;
            // START_TIME(StartTimeTem4_);
            FreeGridsForMultiResolutionMapToGlobalGraphTem_ = partition_helper::partitionGrid(
                GeneratedPlannerCellsAlltem_, VRMapResolutionMax_, VRMapResolutionMin_, LocalCellsCurrentCenterPositionTem_, ElevationMapResolution_,
                ValidThresholdForMultiMapResolution_, ObstacleGridsForMultiResolutionMapToGlobalGraphTem_);
            std::unique_lock<std::mutex> lock2(GeneratedGrids_Mutex_);
            GeneratedFreeGrids_.insert(GeneratedFreeGrids_.end(), FreeGridsForMultiResolutionMapToGlobalGraphTem_.begin(), FreeGridsForMultiResolutionMapToGlobalGraphTem_.end());
            GeneratedObstacleGrids_.insert(GeneratedObstacleGrids_.end(), ObstacleGridsForMultiResolutionMapToGlobalGraphTem_.begin(), ObstacleGridsForMultiResolutionMapToGlobalGraphTem_.end());
            lock2.unlock();

            Eigen::Vector3d CenterPosition3D(point_idx.x,
                                             point_idx.y, zTem_);
            // std::unique_lock<std::mutex> lock1(VRGMPtr_Mutex_);
            std::unique_lock<std::mutex> lock1_1(VRGMPtr_Mutex_);
            // lock1.lock();
            VRGMPtr_->removeVerticesInBox(nearest_vertices_GlobalMultiResolutionMap,
                                          CenterPosition3D, VRMapResolutionMax_ / 2,
                                          VRMapResolutionMax_ / 2, LimitBoxZ_);
            lock1_1.unlock();

            // // TODO The Diff is a small complentation. Might more precise supplements are needed.
            // const Point &point_C = points_C[point_idx];
            // Point DiffTem_(voxel_size_ / 2, voxel_size_ / 2, voxel_size_ / 2);
            // const Point &PointCAfterRevisedTem_ = point_C - DiffTem_;
            // Block<OccupancyVoxel>::Ptr block = nullptr;
            // BlockIndex block_idx;
            // GlobalIndex global_voxel_idx = getGridIndexFromPoint<GlobalIndex>(PointCAfterRevisedTem_ * voxel_size_inv_);
            // bool IfNeedIgnoringThisVoxelTem_;
            // OccupancyVoxel *OccupancyVoxelTem_ = allocateStorageAndGetVoxelPtrIngoringFreeBlocks(
            //     IfFreePoints, global_voxel_idx, &block, &block_idx, IfNeedIgnoringThisVoxelTem_);
            // if (IfNeedIgnoringThisVoxelTem_)
            // {
            //     return;
            // }
            // UpdateOccupancyVoxel(IfFreePoints, OccupancyVoxelTem_);
        }
    }

    void FastSimpleIntegrator::integrateGridCells(const std::unordered_map<VOXEL_LOC, std::vector<PlannerCell>> &GridClusterVector)
    {
        integration_start_time_ = std::chrono::steady_clock::now();

        std::unique_ptr<ThreadSafeIndex> index_getter(ThreadSafeIndexFactory::get(config_.integration_order_mode, GridClusterVector));

        std::list<std::thread> integration_threads;
        VerticesNeedRemoved_.clear();
        std::unique_lock<std::mutex> lock2(GeneratedGrids_Mutex_);
        GeneratedFreeGrids_.clear();
        GeneratedObstacleGrids_.clear();
        lock2.unlock();
        for (size_t i = 0; (i < config_.integrator_threads) && (i < GridClusterVector.size()); ++i)
        {
            integration_threads.emplace_back(&FastSimpleIntegrator::integrateFunction, this, GridClusterVector,
                                             index_getter.get());
        }
        for (std::thread &thread : integration_threads)
        {
            thread.join();
        }
        updateMapWithStoredGrids();
    }

    std::string IntegratorBase::Config::print() const
    {
        std::stringstream ss;
        // clang-format off
  ss << "======================= Integrator Config ====================\n";
  ss << " General: \n";
  ss << " - integrator_threads:                        " << integrator_threads << "\n";
  ss << " - integration_order_mode:                    " << integration_order_mode << "\n";
  ss << " FastSimpleIntegrator: \n";
  ss << " - max_integration_time_s:                    " << max_integration_time_s << "\n";
  ss << " - LimitBoxZ_:                                " << LimitBoxZ_ << "\n";
  ss << " - VRMapResolutionMax_:                       " << VRMapResolutionMax_ << "\n";
  ss << " - VRMapResolutionMin_:                       " << VRMapResolutionMin_ << "\n";
  ss << " - ElevationMapResolution_:                   " << ElevationMapResolution_ << "\n";
  ss << " - ValidThresholdForMultiMapResolution_:      " << ValidThresholdForMultiMapResolution_ << "\n";
  ss << " - LimitBoxZForConnectEdge_:                  " << LimitBoxZForConnectEdge_ << "\n";
  ss << "==============================================================\n";
        // clang-format on
        return ss.str();
    }
} // namespace IntegratorNS