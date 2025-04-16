#include <vrmapping/Integrator/IntegratorUtils.h>
namespace IntegratorNS
{
    ThreadSafeIndex *ThreadSafeIndexFactory::get(const std::string &mode, const std::unordered_map<VOXEL_LOC, std::vector<PlannerCell>> &GridClusterVector)
    {
        if (mode == "mixed")
        {
            return new MixedThreadSafeIndex(GridClusterVector.size());
        }
        else if (mode == "FastSimple")
        {
            return new FastSimpleThreadSafeIndex(GridClusterVector);
        }
        else
        {
            LOG(FATAL) << "Unknown integration order mode: '" << mode << "'!";
        }
        return nullptr;
    }

    ThreadSafeIndex::ThreadSafeIndex(size_t number_of_points) : atomic_idx_(0), number_of_points_(number_of_points)
    {
    }

    MixedThreadSafeIndex::MixedThreadSafeIndex(size_t number_of_points)
        : ThreadSafeIndex(number_of_points), number_of_groups_(number_of_points / step_size_)
    {
    }

    FastSimpleThreadSafeIndex::FastSimpleThreadSafeIndex(const std::unordered_map<VOXEL_LOC, std::vector<PlannerCell>> &GridClusterVector) : ThreadSafeIndex(GridClusterVector.size())
    {
        indices_and_keys_.reserve(GridClusterVector.size());
        size_t idx = 0;
        for (auto &IterateGridVectorMapMultiResolutionMap_ : GridClusterVector)
        {

            indices_and_keys_.emplace_back(idx, IterateGridVectorMapMultiResolutionMap_.first);
            ++idx;
        }
    }

    // returns true if index is valid, false otherwise
    bool ThreadSafeIndex::getNextIndex(VOXEL_LOC *idx)
    {
        size_t sequential_idx = atomic_idx_.fetch_add(1);

        if (sequential_idx >= number_of_points_)
        {
            return false;
        }
        else
        {
            *idx = getNextIndexImpl(sequential_idx);
            return true;
        }
    }

    void ThreadSafeIndex::reset()
    {
        atomic_idx_.store(0);
    }

    VOXEL_LOC MixedThreadSafeIndex::getNextIndexImpl(size_t sequential_idx)
    {
        if (number_of_groups_ * step_size_ <= sequential_idx)
        {
            return sequential_idx;
        }

        const size_t group_num = sequential_idx % number_of_groups_;
        const size_t position_in_group = sequential_idx / number_of_groups_;

        // return group_num * step_size_ + position_in_group;
        VOXEL_LOC VoxelTem_(0, 0, 0);
        return VoxelTem_;
    }

    VOXEL_LOC FastSimpleThreadSafeIndex::getNextIndexImpl(size_t sequential_idx)
    {
        return indices_and_keys_[sequential_idx].second;
    }

    bool IfVertexExistInGraphManager(StateVec state_new, std::vector<Vertex *> *nearest_vertices,
                                     std::shared_ptr<VRGraphManager> SourceVRGraphManager, double MapResolution,
                                     double LimitBoxZ)
    {
        if (SourceVRGraphManager->getNearestVerticesInBox(&state_new, MapResolution / 2, MapResolution / 2, LimitBoxZ,
                                                          nearest_vertices))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void GenerateCellsByPositionAndResolution(std::vector<PlannerCell> &CellsOutPut, Eigen::Vector3d CenterPosition,
                                              double resolution, double CellResolution,
                                              bool IfFillWithTraversableGrid)
    {
        int traversability = 0;
        if (IfFillWithTraversableGrid)
        {
            traversability = 0;
        }
        else
        {
            traversability = 1;
        }
        double start_x = CenterPosition[0] - resolution / 2;
        double end_x = CenterPosition[0] + resolution / 2;
        double start_y = CenterPosition[1] - resolution / 2;
        double end_y = CenterPosition[1] + resolution / 2;
        int row_strat;
        int row_end;
        int col_start;
        int col_end;

        if (start_x < 0.0)
        {
            row_strat = (start_x) / CellResolution - 0.5;
        }
        else
        {
            row_strat = (start_x) / CellResolution + 0.5;
        }

        if (end_x < 0.0)
        {
            row_end = (end_x) / CellResolution - 0.5;
        }
        else
        {
            row_end = (end_x) / CellResolution + 0.5;
        }
        ////////////
        if (start_y < 0.0)
        {
            col_start = (start_y) / CellResolution - 0.5;
        }
        else
        {
            col_start = (start_y) / CellResolution + 0.5;
        }

        if (end_y < 0.0)
        {
            col_end = (end_y) / CellResolution - 0.5;
        }
        else
        {
            col_end = (end_y) / CellResolution + 0.5;
        }

        for (int iRowMiddle_ = row_strat; iRowMiddle_ <= row_end; iRowMiddle_++)
        {
            for (int iColMiddle_ = col_start; iColMiddle_ <= col_end; iColMiddle_++)
            {

                PlannerCell CellMiddle_;
                CellMiddle_ << ((double)(iRowMiddle_)-0.5) * CellResolution, ((double)(iColMiddle_)-0.5) * CellResolution,
                    CenterPosition[2], traversability;
                if (CellMiddle_[0] < 0)
                {
                    CellMiddle_[0] = CellMiddle_[0] + CellResolution;
                }
                if (CellMiddle_[1] < 0)
                {
                    CellMiddle_[1] = CellMiddle_[1] + CellResolution;
                }

                if ((CellMiddle_[0] < start_x) || (CellMiddle_[0] > end_x) || (CellMiddle_[1] < start_y) ||
                    (CellMiddle_[1] > end_y))
                {
                    continue;
                }
                // std::cout << "Cell Generated is " << CellMiddle_[0]
                //           << " " << CellMiddle_[1]
                //           << " " << CellMiddle_[2] << std::endl;
                CellsOutPut.push_back(CellMiddle_);
            }
        }
    }

    void AddGridsToGraphManager(std::vector<PlannerGrid> Grids,
                                std::shared_ptr<VRGraphManager> GlobalGraphForMultiResolutionMap,
                                bool IfTraversable,
                                double VRMapResolutionMax,
                                double VRMapResolutionMin,
                                double ElevationMapResolution,
                                double LimitBoxZForConnectEdge)
    {
        // std::cout << "IntegratorNS::AddGridsToGraphManager" << std::endl;
        for (std::vector<PlannerGrid>::iterator iterGridsVector_ = Grids.begin(); iterGridsVector_ != Grids.end();
             ++iterGridsVector_)
        {
            StateVec StateNew_(iterGridsVector_->centerX, iterGridsVector_->centerY, iterGridsVector_->centerZ, 0);
            // std::cout << "New state is " << StateNew_ << std::endl;
            Vertex *VertexNew = new Vertex(GlobalGraphForMultiResolutionMap->generateVertexID(), StateNew_);
            VertexNew->resolution = iterGridsVector_->size;
            VertexNew->is_obstacle = !IfTraversable;

            GlobalGraphForMultiResolutionMap->addVertex(VertexNew);
            Eigen::Vector2d VertexNewPosition(VertexNew->state[0], VertexNew->state[1]);
            if (IfTraversable)
            {
                // Add edge.
                std::vector<Vertex *> NearestVerticesForConnectingEdge_;
                NearestVerticesForConnectingEdge_.clear();
                GlobalGraphForMultiResolutionMap->getNearestVerticesInBox(
                    &StateNew_, VRMapResolutionMax + 0.5 * VRMapResolutionMin, VRMapResolutionMax + 0.5 * VRMapResolutionMin,
                    LimitBoxZForConnectEdge, &NearestVerticesForConnectingEdge_);
                for (size_t iForEdge_ = 0; iForEdge_ < NearestVerticesForConnectingEdge_.size(); iForEdge_++)
                {
                    if (NearestVerticesForConnectingEdge_[iForEdge_]->is_obstacle)
                    {
                        continue;
                    }
                    if (VertexNew->id == NearestVerticesForConnectingEdge_[iForEdge_]->id)
                    {
                        continue;
                    }
                    double distanceForEdge_ = 0.0;
                    Eigen::Vector2d VertexOldPosition_(NearestVerticesForConnectingEdge_[iForEdge_]->state[0],
                                                       NearestVerticesForConnectingEdge_[iForEdge_]->state[1]);
                    Eigen::Vector2d DistanceVectorMiddle_ = VertexOldPosition_ - VertexNewPosition;
                    distanceForEdge_ = DistanceVectorMiddle_.norm();
                    bool IfCouldConnectTwoVertices = false;
                    double VNR_ = VertexNew->resolution;
                    double VOR_ = NearestVerticesForConnectingEdge_[iForEdge_]->resolution;
                    double DFEIX = abs(DistanceVectorMiddle_[0]); // Distance For Edge In X-axe
                    double DFEIY = abs(DistanceVectorMiddle_[1]); // Distance For Edge In Y-axe
                    if ((DFEIX <= (VNR_ + VOR_) / 2 + ElevationMapResolution) &&
                        (DFEIY <= (VNR_ + VOR_) / 2 + ElevationMapResolution))
                    {
                        IfCouldConnectTwoVertices = true;
                    }
                    if (IfCouldConnectTwoVertices)
                    {
                        GlobalGraphForMultiResolutionMap->removeEdge(VertexNew, NearestVerticesForConnectingEdge_[iForEdge_]);
                        GlobalGraphForMultiResolutionMap->addEdge(VertexNew, NearestVerticesForConnectingEdge_[iForEdge_],
                                                                  distanceForEdge_);
                    }
                }
            }
        }
    }

} // namespace IntegratorNS