#include <vrmapping/Sampler.h>
Sampler::Sampler()
{
    last_map_update_time_ = 0.0;
    GSOGM_.reset(new VRGraphManager());
    LSOGM_.reset(new VRGraphManager());
}

Sampler::~Sampler()
{
}

bool Sampler::SetGridMap(grid_map::GridMap ElevationMap)
{
    ElevationMap_ = ElevationMap;
    return true;
}

void Sampler::samplePositionInMapFromDist(grid_map::GridMap SourceElevationMap,
                                          std::string ElevationLayer,
                                          std::string CumProbLayer,
                                          std::string CumProbRowwiseHackLayer,
                                          std::string NormalXLayer,
                                          std::string NormalYLayer,
                                          std::string NormalZLayer,
                                          Eigen::Vector3d &Position3DOutput,
                                          Eigen::Vector3d &NormalVectorOutput)
{
    // Sample uniform values.
    const auto samp_col = uniDist_(generator_);
    const auto samp_row = uniDist_(generator_);

    Eigen::Index col, row;

    const auto &cum_prob = SourceElevationMap.get(CumProbLayer.c_str());
    auto CubProRowHackTem_ = SourceElevationMap.get(CumProbRowwiseHackLayer.c_str()).row(0);

    int colTem_ = 0;
    for (colTem_ = 0; colTem_ < cum_prob.cols() - 1; colTem_++)
    {
        if (CubProRowHackTem_(0, colTem_) > 0)
        {
            break;
        }
    }
    const Eigen::Matrix<grid_map::DataType, Eigen::Dynamic, 1> cum_prob_rowwise =
        SourceElevationMap.get(CumProbRowwiseHackLayer.c_str()).col(colTem_);

    // Apply distribution
    for (row = cum_prob_rowwise.rows() - 2; row > 0; --row)
    {
        if (cum_prob_rowwise(row, 0) > samp_row)
        {
            // std::cout << "cum_prob_rowwise(row, 0) is " << cum_prob_rowwise(row, 0) << std::endl;
            // std::cout << "row is " << row << std::endl;
            break;
        }
    }
    for (col = cum_prob.cols() - 2; col > 0; --col)
    {
        if (cum_prob(row, col) > samp_col)
        {
            // std::cout << "cum_prob(row, col) is " << cum_prob(row, col) << std::endl;
            // std::cout << "col is " << col << std::endl;
            break;
        }
    }
    // grid_map::Position Position2DTem_;
    // SourceElevationMap.getPosition(grid_map::Index(row, col), Position2DTem_);
    grid_map::Position3 Position3DTem_;
    // std::cout << "cum_prob_rowwise.rows() is " << cum_prob_rowwise.rows() << std::endl;
    // std::cout << "cum_prob.cols() is " << cum_prob.cols() << std::endl;
    // std::cout << "samp_row is " << samp_row << std::endl;
    // std::cout << "samp_col is " << samp_col << std::endl;
    grid_map::Index IndexTem_(row, col);
    // std::cout << "IndexTem_ is " << IndexTem_ << std::endl;
    SourceElevationMap.getPosition3(ElevationLayer,
                                    IndexTem_,
                                    Position3DTem_);
    Position3DOutput = Position3DTem_;
    double NormalXTem_ = SourceElevationMap.at(NormalXLayer.c_str(),
                                               IndexTem_);
    double NormalYTem_ = SourceElevationMap.at(NormalYLayer.c_str(),
                                               IndexTem_);
    double NormalZTem_ = SourceElevationMap.at(NormalZLayer.c_str(),
                                               IndexTem_);
    Eigen::Vector3d NormalVectorTem_(NormalXTem_,
                                     NormalYTem_,
                                     NormalZTem_);
    NormalVectorOutput = NormalVectorTem_;
    return;
}

void Sampler::SampleUniform(grid_map::GridMap SourceElevationMap,
                            std::string ElevationLayer,
                            std::string CumProbLayer,
                            std::string CumProbRowwiseHackLayer,
                            std::string NormalXLayer,
                            std::string NormalYLayer,
                            std::string NormalZLayer,
                            Eigen::Vector3d &PositionSampledOutput)
{
    // Update z value with height from map.
    grid_map::Position3 Position3DTem_;
    Eigen::Vector3d NormalVectorTem_;
    samplePositionInMapFromDist(SourceElevationMap,
                                ElevationLayer,
                                CumProbLayer,
                                CumProbRowwiseHackLayer,
                                NormalXLayer,
                                NormalYLayer,
                                NormalZLayer,
                                Position3DTem_,
                                NormalVectorTem_);

    // Apply small random perturbation in normal direction.

    // const auto pert = uniformReal(-1, 1) * 0.1;
    // Position3DTem_[0] = Position3DTem_[0] + NormalVectorTem_[0] * pert;
    // Position3DTem_[1] = Position3DTem_[1] + NormalVectorTem_[1] * pert;
    // Position3DTem_[2] = Position3DTem_[2] + NormalVectorTem_[2] * pert;

    // Eigen::Vector3d StateSe3Tem_ = Position3DTem_;
    PositionSampledOutput = Position3DTem_;

    // // Sample rotation.
    // eulerRPY(Position3DTem_); // Abuse position to avoid memory allocation.

    // // Get roll and pitch from elevation map.

    // const Eigen::Quaterniond R_wb(Eigen::AngleAxisd(Position3DTem_[2],
    //                                                 Eigen::Vector3d::UnitZ()));

    // const auto normal_b = R_wb.inverse() * NormalVectorTem_;

    // Position3DTem_[0] = -atan2(normal_b.y(), normal_b.z()) + Position3DTem_[0] * 3.33 / M_PI_2; // Roll.
    // Position3DTem_[1] = atan2(normal_b.x(), normal_b.z()) + Position3DTem_[1] * 10.0 / M_PI_4;  // Pitch.

    // setSO3FromRPY(state_se3->rotation(), Position3DTem_);
}

double Sampler::uniformReal(double lower_bound, double upper_bound)
{
    assert(lower_bound <= upper_bound);
    return (upper_bound - lower_bound) * uniDist_(generator_) + lower_bound;
}

void Sampler::eulerRPY(Eigen::Vector3d &Value)
{
    Value[0] = boost::math::constants::pi<double>() * (-2.0 * uniDist_(generator_) + 1.0);
    Value[1] = acos(1.0 - 2.0 * uniDist_(generator_)) - boost::math::constants::pi<double>() / 2.0;
    Value[2] = boost::math::constants::pi<double>() * (-2.0 * uniDist_(generator_) + 1.0);
}

void Sampler::SampleGraphManager(std::shared_ptr<VRGraphManager> TargetGraphManager,
                                 grid_map::GridMap SourceElevationMap,
                                 std::string ElevationLayer,
                                 std::string TraversabilityLayer,
                                 std::string CumProbLayer,
                                 std::string CumProbRowwiseHackLayer,
                                 std::string NormalXLayer,
                                 std::string NormalYLayer,
                                 std::string NormalZLayer,
                                 int NewSampledVerticesNumThreshold,
                                 int ValidVerticesThreshold,
                                 int ValidEdgesThreshold,
                                 double SampleTimeThreshold,
                                 double DistanceXMax,
                                 double DistanceYMax,
                                 double DistanceZMax,
                                 int MaxCheckPoint,
                                 double RobotWidth,
                                 double TraversabilityThreshold,
                                 int ConnectEdgesMax,
                                 double EdgeLengthMin)
{
    const auto map_time = SourceElevationMap.getTimestamp();
    if (map_time == last_map_update_time_)
    {
        // Don't need to sample graph again if we query with same map.
        // TODO: Remove this spammy stuff, once we know it works.
        return;
    }
    else
    {
        last_map_update_time_ = map_time;
    }

    std::random_device rd;  // Will be used to obtain a seed for the random number engine.
    std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd().

    if (TargetGraphManager != NULL)
    {
        auto start = std::chrono::high_resolution_clock::now();
        size_t n_samples = 0;
        int n_valid_samples = 0;
        int n_valid_edges = 0;
        // unsigned int n_proc = 0;
        bool timer_exceeded = false;

        int NewSampledVerticesNumTem_ = 0;

        while (NewSampledVerticesNumTem_ < NewSampledVerticesNumThreshold &&
               n_valid_samples < ValidVerticesThreshold &&
               n_valid_edges < ValidEdgesThreshold)
        {
            // Sample new vertex.
            Eigen::Vector3d Position3DNewSampledTem_(0, 0, 0);
            do
            {
                SampleUniform(SourceElevationMap,
                              ElevationLayer,
                              CumProbLayer,
                              CumProbRowwiseHackLayer,
                              NormalXLayer,
                              NormalYLayer,
                              NormalZLayer,
                              Position3DNewSampledTem_);
                ++n_samples;
                if (n_samples % 10 == 0)
                {
                    const auto end = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<double> elapsed_seconds = end - start;
                    if (elapsed_seconds.count() > SampleTimeThreshold)
                    {
                        // std::cout << "Reached sample timer limit." << std::endl;
                        // std::cout << "ValidVerticesThreshold is " << ValidVerticesThreshold << std::endl;
                        // std::cout << "n_valid_samples is ." << n_valid_samples << std::endl;
                        timer_exceeded = true;
                        break;
                    }
                }
            } while (!IsValid(Position3DNewSampledTem_));

            if (timer_exceeded)
            {
                break;
            }
            //  Check If could connect to the GraphManager.
            // std::cout << "Position3DNewSampledTem_ is " << Position3DNewSampledTem_ << std::endl;

            int EdgesNumTem_ = 0;
            if (IfConnectToTheGraphManagerSucceed(TargetGraphManager,
                                                  Position3DNewSampledTem_,
                                                  SourceElevationMap,
                                                  ElevationLayer,
                                                  TraversabilityLayer,
                                                  DistanceXMax,
                                                  DistanceYMax,
                                                  DistanceZMax,
                                                  MaxCheckPoint,
                                                  RobotWidth,
                                                  TraversabilityThreshold,
                                                  ConnectEdgesMax,
                                                  EdgeLengthMin,
                                                  EdgesNumTem_))
            {
                // ROS_INFO("Add!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                ++n_valid_samples;
                n_valid_edges = n_valid_edges + EdgesNumTem_;
                usleep(100000 * SampleTimeThreshold / ValidVerticesThreshold);
            }
            NewSampledVerticesNumTem_++;
        }

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;

        if (false)
        {
            std::cout << "Sampled for " << std::setprecision(3) << elapsed_seconds.count() * 1000.0 << "ms" << std::endl;
            std::cout << "Added " << n_valid_samples << " new milestones from " << n_samples << " attempts." << std::endl;
            std::cout << "NewSampledVerticesNumTem_ is " << NewSampledVerticesNumTem_ << std::endl;
            std::cout << "for a total of " << TargetGraphManager->getNumVertices() << " milestones" << std::endl;
            std::cout << "and " << TargetGraphManager->getNumEdges() << " edges." << std::endl;
        }
    }
}

bool Sampler::IsValid(Eigen::Vector3d Position3D)
{
    Eigen::Vector3d FootPositionTem_ = Position3D;
    Eigen::Vector3d BodyPositionTem_ = Position3D;

    return BodyIsValid(BodyPositionTem_) && FootIsValid(FootPositionTem_);
}

bool Sampler::BodyIsValid(Eigen::Vector3d Position3D)
{
    if ((Position3D[0] == 0) &&
        (Position3D[1] == 0) &&
        (Position3D[2] == 0))
    {
        return false;
    }
    // TODO Add precise judgement. Now use the handled map layer.
    return true;
}
bool Sampler::FootIsValid(Eigen::Vector3d Position3D)
{
    if ((Position3D[0] == 0) &&
        (Position3D[1] == 0) &&
        (Position3D[2] == 0))
    {
        return false;
    }
    // TODO Add precise judgement. Now use the handled map layer.
    return true;
}

bool Sampler::IfConnectToTheGraphManagerSucceed(std::shared_ptr<VRGraphManager> TargetGraphManager,
                                                Eigen::Vector3d &PositionNeedToBeConnected,
                                                grid_map::GridMap SourceElevationMap,
                                                std::string ElevationLayer,
                                                std::string TraversabilityLayer,
                                                double DistanceXMax,
                                                double DistanceYMax,
                                                double DistanceZMax,
                                                int MaxCheckPoint,
                                                double RobotWidth,
                                                double TraversabilityThreshold,
                                                int ConnectEdgesMax,
                                                double EdgeLengthMin,
                                                int &EdgesCount)
{
    StateVec StateTem_(PositionNeedToBeConnected[0],
                       PositionNeedToBeConnected[1],
                       PositionNeedToBeConnected[2],
                       0);
    int EdgesNumTem_ = 0;
    Vertex *VertexPtrTem_;
    if (TargetGraphManager->getNearestVertex(&StateTem_, &VertexPtrTem_))
    {
        Eigen::Vector3d PointExistInGraphManagerTem_(VertexPtrTem_->state[0],
                                                     VertexPtrTem_->state[1],
                                                     VertexPtrTem_->state[2]);
        Eigen::Vector3d DistanceVectorTem_ = PositionNeedToBeConnected - PointExistInGraphManagerTem_;
        double DistanceTem_ = DistanceVectorTem_.norm();
        if (DistanceTem_ < EdgeLengthMin)
        {
            // Eigen::Vector3d NewPositionForSampledPoint = PointExistInGraphManagerTem_ +
            //                                              EdgeLengthMin * DistanceVectorTem_.normalized();
            // PositionNeedToBeConnected = NewPositionForSampledPoint;
            // DistanceTem_ = EdgeLengthMin;

            return false;
        }
        else
        {
            if (IfConnectTwoPointsSucceed(SourceElevationMap,
                                          ElevationLayer,
                                          TraversabilityLayer,
                                          RobotWidth,
                                          TraversabilityThreshold,
                                          PointExistInGraphManagerTem_,
                                          PositionNeedToBeConnected))
            {
                StateVec StateNewTem_(PositionNeedToBeConnected[0],
                                      PositionNeedToBeConnected[1],
                                      PositionNeedToBeConnected[2],
                                      0);
                Vertex *VertexNew = new Vertex(TargetGraphManager->generateVertexID(), StateNewTem_);

                VertexNew->resolution = 0.2;
                VertexNew->is_obstacle = false;
                TargetGraphManager->addVertex(VertexNew);
                TargetGraphManager->addEdge(VertexNew, VertexPtrTem_, DistanceTem_);
                EdgesNumTem_++;
                // return true;
                std::vector<Vertex *> VerticesTem_;
                VerticesTem_.clear();

                if (TargetGraphManager->getNearestVerticesInBox(&StateTem_,
                                                                DistanceXMax,
                                                                DistanceYMax,
                                                                DistanceZMax,
                                                                &VerticesTem_))
                {

                    for (size_t i = 0; (i < VerticesTem_.size()) && (i < MaxCheckPoint); i++)
                    {

                        Eigen::Vector3d PointExistInGraphManagerTem2_(VerticesTem_[i]->state[0],
                                                                      VerticesTem_[i]->state[1],
                                                                      VerticesTem_[i]->state[2]);
                        Eigen::Vector3d DistanceVectorTem2_ = PositionNeedToBeConnected - PointExistInGraphManagerTem2_;
                        double DistanceTem2_ = DistanceVectorTem2_.norm();
                        if (DistanceTem2_ < EdgeLengthMin)
                        {
                            continue;
                        }
                        Eigen::Vector2d Position2DTem_(PositionNeedToBeConnected[0],
                                                       PositionNeedToBeConnected[1]);
                        if (!SourceElevationMap.isInside(Position2DTem_))
                        {
                            continue;
                        }
                        if (VerticesTem_[i]->id == VertexNew->id)
                        {
                            continue;
                        }
                        if (VerticesTem_[i]->id == VertexPtrTem_->id)
                        {
                            continue;
                        }
                        // Check If could connect two points.
                        if (IfConnectTwoPointsSucceed(SourceElevationMap,
                                                      ElevationLayer,
                                                      TraversabilityLayer,
                                                      RobotWidth,
                                                      TraversabilityThreshold,
                                                      PointExistInGraphManagerTem2_,
                                                      PositionNeedToBeConnected))
                        {
                            TargetGraphManager->addEdge(VertexNew, VerticesTem_[i], DistanceTem2_);
                            EdgesNumTem_++;
                            if (EdgesNumTem_ >= ConnectEdgesMax)
                            {
                                EdgesCount = EdgesNumTem_;
                                return true;
                            }
                        }
                    }
                }
                else
                {
                    return false;
                }
                EdgesCount = EdgesNumTem_;
                return true;
            }
            else
            {
                return false;
            }
        }
    }
    else
    {
        return false;
    }

    // std::vector<Vertex *> VerticesTem_;
    // VerticesTem_.clear();
    // int EdgesNumTem_ = 0;
    // if (TargetGraphManager->getNearestVerticesInBox(&StateTem_,
    //                                                 DistanceXMax,
    //                                                 DistanceYMax,
    //                                                 DistanceZMax,
    //                                                 &VerticesTem_))
    // {

    //     for (size_t i = 0; (i < VerticesTem_.size()) && (i < MaxCheckPoint); i++)
    //     {

    //         Eigen::Vector3d PointExistInGraphManagerTem_(VerticesTem_[i]->state[0],
    //                                                      VerticesTem_[i]->state[1],
    //                                                      VerticesTem_[i]->state[2]);
    //         Eigen::Vector3d DistanceVectorTem_ = PositionNeedToBeConnected - PointExistInGraphManagerTem_;
    //         double DistanceTem_ = DistanceVectorTem_.norm();
    //         if (DistanceTem_ < EdgeLengthMin)
    //         {
    //             Eigen::Vector3d NewPositionForSampledPoint = PointExistInGraphManagerTem_ +
    //                                                          EdgeLengthMin * DistanceVectorTem_.normalized();
    //             PositionNeedToBeConnected = NewPositionForSampledPoint;
    //             DistanceTem_ = EdgeLengthMin;
    //             continue;
    //         }
    //         Eigen::Vector2d Position2DTem_(PositionNeedToBeConnected[0],
    //                                        PositionNeedToBeConnected[1]);
    //         if (!SourceElevationMap.isInside(Position2DTem_))
    //         {
    //             continue;
    //         }
    //         // Check If could connect two points.
    //         if (IfConnectTwoPointsSucceed(SourceElevationMap,
    //                                       ElevationLayer,
    //                                       TraversabilityLayer,
    //                                       RobotWidth,
    //                                       TraversabilityThreshold,
    //                                       PointExistInGraphManagerTem_,
    //                                       PositionNeedToBeConnected))
    //         {
    //             StateVec StateNewTem_(PositionNeedToBeConnected[0],
    //                                   PositionNeedToBeConnected[1],
    //                                   PositionNeedToBeConnected[2],
    //                                   0);
    //             Vertex *VertexNew = new Vertex(TargetGraphManager->generateVertexID(), StateNewTem_);

    //             VertexNew->resolution = 0.2;
    //             VertexNew->is_obstacle = false;
    //             TargetGraphManager->addVertex(VertexNew);
    //             TargetGraphManager->addEdge(VertexNew, VerticesTem_[i], DistanceTem_);
    //             EdgesNumTem_++;
    //             if (EdgesNumTem_ >= ConnectEdgesMax)
    //             {
    //                 break;
    //             }
    //         }
    //     }
    // }
    // else
    // {
    //     return false;
    // }
    // if (EdgesNumTem_ > 0)
    // {
    //     EdgesCount = EdgesNumTem_;
    //     return true;
    // }
    // else
    // {
    //     return false;
    // }
    return false;
}

bool Sampler::IfConnectTwoPointsSucceed(grid_map::GridMap SourceElevationMap,
                                        std::string ElevationLayer,
                                        std::string TraversabilityLayer,
                                        double RobotWidth,
                                        double TraversabilityThreshold,
                                        Eigen::Vector3d Point1,
                                        Eigen::Vector3d Point2)
{

    Eigen::Vector2d start_pos_0(Point1[0], Point1[1]);
    Eigen::Vector2d end_pos_0(Point2[0], Point2[1]);

    Eigen::Vector2d Point12DTem_(Point1[0], Point1[1]);
    Eigen::Vector2d Point22DTem_(Point2[0], Point2[1]);

    // grid_map::Size MapSizeTem_ = SourceElevationMap.getSize();
    // double MapResolutionTem_ = SourceElevationMap.getResolution();
    // grid_map::Position MapPositionTem_ = SourceElevationMap.getPosition();
    // Eigen::Vector2d MinValueTem_(MapPositionTem_[0] - (double)(MapSizeTem_[0]) * 0.5 * MapResolutionTem_,
    //                              MapPositionTem_[1] - (double)(MapSizeTem_[1]) * 0.5 * MapResolutionTem_);
    // Eigen::Vector2d MaxValueTem_(MapPositionTem_[0] + (double)(MapSizeTem_[0]) * 0.5 * MapResolutionTem_,
    //                              MapPositionTem_[1] + (double)(MapSizeTem_[1]) * 0.5 * MapResolutionTem_);

    // if (end_pos_0[0] == start_pos_0[0])
    // {
    //     if (start_pos_0[1] < MinValueTem_[1])
    //     {
    //         start_pos_0 << start_pos_0[0],
    //             MinValueTem_[1];
    //     }
    //     else if (start_pos_0[1] > MaxValueTem_[1])
    //     {
    //         start_pos_0 << start_pos_0[0],
    //             MaxValueTem_[1];
    //     }

    //     if (end_pos_0[1] < MinValueTem_[1])
    //     {
    //         end_pos_0 << end_pos_0[0],
    //             MinValueTem_[1];
    //     }
    //     else if (end_pos_0[1] > MaxValueTem_[1])
    //     {
    //         end_pos_0 << end_pos_0[0],
    //             MaxValueTem_[1];
    //     }
    // }
    // else if (end_pos_0[1] == start_pos_0[1])
    // {
    //     if (start_pos_0[0] < MinValueTem_[0])
    //     {
    //         start_pos_0 << MinValueTem_[0],
    //             start_pos_0[1];
    //     }
    //     else if (start_pos_0[0] > MaxValueTem_[0])
    //     {
    //         start_pos_0 << MaxValueTem_[0],
    //             start_pos_0[1];
    //     }

    //     if (end_pos_0[0] < MinValueTem_[0])
    //     {
    //         end_pos_0 << MinValueTem_[0],
    //             end_pos_0[1];
    //     }
    //     else if (end_pos_0[0] > MaxValueTem_[0])
    //     {
    //         end_pos_0 << MaxValueTem_[0],
    //             end_pos_0[1];
    //     }
    // }
    // else
    // {
    //     double kTem_ = (end_pos_0[1] - start_pos_0[1]) / (end_pos_0[0] - start_pos_0[0]);
    //     double bTem_ = start_pos_0[1] - kTem_ * start_pos_0[0];

    //     if (start_pos_0[0] < MinValueTem_[0])
    //     {
    //         start_pos_0 << MinValueTem_[0],
    //             kTem_ * MinValueTem_[0] + bTem_;
    //     }
    //     else if (start_pos_0[0] > MaxValueTem_[0])
    //     {
    //         start_pos_0 << MaxValueTem_[0],
    //             kTem_ * MaxValueTem_[0] + bTem_;
    //     }

    //     if (start_pos_0[1] < MinValueTem_[1])
    //     {
    //         start_pos_0 << (MinValueTem_[1] - bTem_) / kTem_,
    //             MinValueTem_[1];
    //     }
    //     else if (start_pos_0[1] > MaxValueTem_[1])
    //     {
    //         start_pos_0 << (MaxValueTem_[1] - bTem_) / kTem_,
    //             MaxValueTem_[1];
    //     }

    //     if (end_pos_0[0] < MinValueTem_[0])
    //     {
    //         end_pos_0 << MinValueTem_[0],
    //             kTem_ * MinValueTem_[0] + bTem_;
    //     }
    //     else if (end_pos_0[0] > MaxValueTem_[0])
    //     {
    //         end_pos_0 << MaxValueTem_[0],
    //             kTem_ * MaxValueTem_[0] + bTem_;
    //     }

    //     if (end_pos_0[1] < MinValueTem_[1])
    //     {
    //         end_pos_0 << (MinValueTem_[1] - bTem_) / kTem_,
    //             MinValueTem_[1];
    //     }
    //     else if (end_pos_0[1] > MaxValueTem_[1])
    //     {
    //         end_pos_0 << (MaxValueTem_[1] - bTem_) / kTem_,
    //             MaxValueTem_[1];
    //     }
    // }

    if ((!SourceElevationMap.isInside(Point12DTem_)) ||
        (!SourceElevationMap.isInside(Point22DTem_)))
    {
        return false;
    }

    // Point1[0] = start_pos_0[0];
    // Point1[1] = start_pos_0[1];
    // Point2[0] = end_pos_0[0];
    // Point2[1] = end_pos_0[1];

    int SizeInY = (int)(RobotWidth / SourceElevationMap.getResolution());
    int SizeYUsed = SizeInY / 2;

    int bad_point_count_ = 0;
    int nan_point_count_ = 0;
    double yaw_middle_ = atan((Point2[1] - Point1[1]) / (Point2[0] - Point1[0]));

    for (int i = -SizeYUsed; i <= SizeYUsed + 1; i++)
    {
        double y_change_in_robot_frame_ = i * SourceElevationMap.getResolution();
        double x_start_in_body_frame_ = 0;
        double y_start_in_body_frame = y_change_in_robot_frame_;
        double x_start_in_world_frame = Point1[0] +
                                        x_start_in_body_frame_ * cos(yaw_middle_) -
                                        y_start_in_body_frame * sin(yaw_middle_);
        double y_start_in_world_frame = Point1[1] +
                                        x_start_in_body_frame_ * sin(yaw_middle_) +
                                        y_start_in_body_frame * cos(yaw_middle_);

        Eigen::Vector2d start_middle(x_start_in_world_frame, y_start_in_world_frame);
        Eigen::Vector2d end_middle = end_pos_0 - start_pos_0 + start_middle;

        try
        {
            for (grid_map::LineIterator iterator(SourceElevationMap, start_middle, end_middle);
                 !iterator.isPastEnd(); ++iterator)
            {
                double elevation = SourceElevationMap.at(ElevationLayer.c_str(), *iterator);
                double traversability_score = SourceElevationMap.at(TraversabilityLayer.c_str(), *iterator);

                if (elevation != elevation)
                {
                    nan_point_count_++;
                    // bad_point_count_++;
                    continue;
                }
                else
                {
                }

                if (traversability_score != traversability_score)
                {
                    nan_point_count_++;
                    // bad_point_count_++;
                    continue;
                }
                else
                {
                }

                if (traversability_score < TraversabilityThreshold)
                {
                    bad_point_count_++;
                    continue;
                }
                else
                {
                }
            }
        }
        catch (const std::exception &e)
        {
            // ROS_ERROR("[Sampler_Info]: Failed to check edge as it is out of range.");
            // std::cout << "Points are "
            //           << Point1[0] << " " << Point1[1] << " " << Point1[2] << std::endl
            //           << Point2[0] << " " << Point2[1] << " " << Point2[2] << std::endl;
            // std::cerr << e.what() << '\n';
            return false;
        }
    }

    // std::cout << "angle is " << angle << " and bad_point_count_ is " << bad_point_count_ << std::endl;

    if (nan_point_count_ > std::max(abs(Point2[0] - Point1[0]) / SourceElevationMap.getResolution(),
                                    abs(Point2[1] - Point1[1]) / SourceElevationMap.getResolution()))
    {
        return false;
    }

    if (bad_point_count_ > 1)
    {
        return false;
    }
    else
    {
        return true;
    }

    return false;
}

void Sampler::InitializeAndProcessGridMap(grid_map::GridMap SourceElevationMap,
                                          grid_map::GridMap &ElevationMapOutput,
                                          std::string ElevationLayer,
                                          std::string TraversabilityLayer,
                                          double BlurRadius)
{

    // ElevationMapOutput.setFrameId(SourceElevationMap.getFrameId().c_str());
    // ElevationMapOutput.setGeometry(SourceElevationMap.getLength(),
    //                                SourceElevationMap.getResolution());
    ElevationMapOutput = SourceElevationMap;
    if (true)
    {
        std::vector<std::string> SourceLayers = SourceElevationMap.getLayers();
        for (size_t i = 0; i < SourceLayers.size(); i++)
        {
            if ((SourceLayers[i].c_str() == ElevationLayer.c_str()) ||
                (SourceLayers[i].c_str() == TraversabilityLayer.c_str()))
            {
                continue;
            }
            ElevationMapOutput.erase(SourceLayers[i].c_str());
        }
    }
    else
    {
        ElevationMapOutput.clearAll();
        ElevationMapOutput.get(ElevationLayer.c_str()) = SourceElevationMap.get(ElevationLayer.c_str());
        ElevationMapOutput.get(TraversabilityLayer.c_str()) = SourceElevationMap.get(TraversabilityLayer.c_str());
    }

    // const auto n_vertices = ElevationMapOutput.getSize();
    // int VerticesNumTem_ = n_vertices[0] * n_vertices[1];
    // grid_map::Index ind;

    ElevationMapOutput.add("n_samples", 1.0);
    auto &n_samples = ElevationMapOutput.get("n_samples");

    int filter_size_cells = 6 * BlurRadius / ElevationMapOutput.getResolution();
    const double std_dev_cells = BlurRadius / ElevationMapOutput.getResolution();
    if (filter_size_cells % 2 == 0)
    {
        filter_size_cells += 1;
    }

    n_samples = gaussianBlurMatrix(n_samples,
                                   filter_size_cells,
                                   std_dev_cells);
    if (!n_samples.isZero())
    {
        // applyBaseSampleDistribution will add a uniform distribution if this is not executed.
        ElevationMapOutput.add("sample_probability", n_samples.maxCoeff() - n_samples.array());
    }
}

grid_map::Matrix Sampler::gaussianBlurMatrix(const grid_map::Matrix &mat,
                                             int size,
                                             double std_dev)
{
    cv::Mat mat_cv(mat.cols(),
                   mat.rows(),
                   cv::DataType<grid_map::Matrix::Scalar>::type,
                   const_cast<void *>(static_cast<const void *>(mat.data())));

    cv::Mat mat_cv_blurred;

    cv::GaussianBlur(mat_cv, mat_cv_blurred, cv::Size(size, size), std_dev);

    Eigen::Map<Eigen::Matrix<grid_map::Matrix::Scalar,
                             Eigen::Dynamic,
                             Eigen::Dynamic>>
        mat_blurred(reinterpret_cast<grid_map::Matrix::Scalar *>(mat_cv_blurred.data),
                    mat.rows(),
                    mat.cols());

    return mat_blurred;
}

void Sampler::ProcessGridMapTraversabilityFilter(grid_map::GridMap &ElevationMapOutput,
                                                 std::string ElevationLayer,
                                                 std::string TraversabilityLayer)
{
    // Inpaint traversability and elevation.
    // ElevationMapOutput.get(TraversabilityLayer.c_str()) = inpaintMatrix(ElevationMapOutput.get(TraversabilityLayer.c_str()));
    // ElevationMapOutput.get(ElevationLayer.c_str()) = inpaintMatrix(ElevationMapOutput.get(ElevationLayer.c_str()));

    // estimateNormals(*ElevationMapOutput, (params_->robot.torso.length + params_->robot.torso.width) * 0.25, params_->planner.elevation_layer);

    // if (params_->planner.unknown_space_untraversable)
    // {
    //     const auto size = ElevationMapOutput->getSize();
    //     ElevationMapOutput->get(params_->planner.traversability_layer) =
    //         (ElevationMapOutput->get("observed").array() > 0.5f).select(ElevationMapOutput->get(params_->planner.traversability_layer), grid_map::Matrix::Zero(size.x(), size.y()));
    // }

    // // Compute safety features.
    // const auto mapsize = ElevationMapOutput->getSize();
    // grid_map::Matrix trav_filter =
    //     (ElevationMapOutput->get(params_->planner.traversability_layer).array() > params_->planner.traversability_thres).select(Eigen::MatrixXf::Ones(mapsize.x(), mapsize.y()), Eigen::MatrixXf::Zero(mapsize.x(), mapsize.y()));

    // ElevationMapOutput->add("traversability_thresholded_no_safety", trav_filter);

    // // Removes "safe" patches which are smaller in diameter than foothold_size.
    // const auto foothold_size = std::ceil(params_->planner.safety.foothold_size / ElevationMapOutput->getResolution());
    // //  grid_map::Matrix trav_filter_saftey = erodeAndDilateMatrix(trav_filter, foothold_size);
    // //  trav_filter = (trav_filter.array() < 0.5f).select(trav_filter, trav_filter_saftey);  // Make sure we don't validate unsafe things.

    // // Erodes traversable edges, unless they are on a small hole, without large elevation change.
    // const auto safety_margin = std::ceil(2 * params_->planner.safety.foothold_margin / ElevationMapOutput->getResolution());
    // const auto hole_size = std::floor(params_->planner.safety.foothold_margin_max_hole_size / ElevationMapOutput->getResolution());
    // grid_map::Matrix trav_filter_saftey = dilateAndErodeMatrix(trav_filter, hole_size); // Close holes.

    // // Compute height difference.
    // const grid_map::Matrix elevation = ElevationMapOutput->get(params_->planner.elevation_layer);
    // const auto safety_search_radius = std::ceil(2 * params_->planner.safety.foothold_margin_max_drop_search_radius / ElevationMapOutput->getResolution());
    // const grid_map::Matrix diff_low = elevation - erodeMatrix(elevation, safety_search_radius);
    // const auto hole_mask = diff_low.array() > params_->planner.safety.foothold_margin_max_drop;
    // trav_filter_saftey = (hole_mask).select(trav_filter, trav_filter_saftey); // Make sure we don't validate unsafe things.
    // ElevationMapOutput->add("diff_low_mask", (hole_mask).select(Eigen::MatrixXf::Ones(mapsize.x(), mapsize.y()),
    //                                                             Eigen::MatrixXf::Zero(mapsize.x(), mapsize.y())));

    // // Don't erode if we have walls, which means stepping close to them is safe.
    // const grid_map::Matrix diff_high = dilateMatrix(elevation, safety_margin) - elevation;
    // const auto wall_mask = (diff_high.array() > params_->planner.safety.foothold_margin_min_step /*&& !hole_mask*/);
    // trav_filter_saftey = (wall_mask).select(Eigen::MatrixXf::Ones(mapsize.x(), mapsize.y()), trav_filter_saftey); // Make sure we don't validate unsafe things.
    // ElevationMapOutput->add("diff_high_mask", (wall_mask).select(Eigen::MatrixXf::Ones(mapsize.x(), mapsize.y()),
    //                                                              Eigen::MatrixXf::Zero(mapsize.x(), mapsize.y())));

    // trav_filter_saftey = erodeMatrix(trav_filter_saftey, safety_margin); // Erode.
    // // Make sure we don't validate unsafe things and remove erosion along walls.
    // trav_filter_saftey = (trav_filter.array() < 0.5f || wall_mask).select(trav_filter, trav_filter_saftey);

    // // Finally, remove any new small valid patches.
    // trav_filter_saftey = erodeAndDilateMatrix(trav_filter_saftey, foothold_size);
    // trav_filter_saftey = (trav_filter.array() < 0.5f).select(trav_filter, trav_filter_saftey); // Make sure we don't validate unsafe things.

    // ElevationMapOutput->add("traversability_thresholded", trav_filter_saftey);

    // // Set untraversable region very low, so that feet are never in collision.
    // ElevationMapOutput->add("elevation_masked", -std::numeric_limits<float>::infinity());
    // ElevationMapOutput->get("elevation_masked") =
    //     (trav_filter_saftey.array() > 0.5).select(ElevationMapOutput->get(params_->planner.elevation_layer), ElevationMapOutput->get("elevation_masked"));
}

void Sampler::ProcessGridMapSampleProbability(grid_map::GridMap &ElevationMapOutput,
                                              std::string TraversabilityLayer)
{
    if (!ElevationMapOutput.exists("sample_probability"))
    {
        ElevationMapOutput.add("sample_probability", 1.0);
    }
    if (ElevationMapOutput.exists(TraversabilityLayer.c_str()))
    {
        ElevationMapOutput.get("sample_probability").array() *= ElevationMapOutput.get(TraversabilityLayer.c_str()).array();
    }
}

grid_map::Matrix Sampler::inpaintMatrix(const grid_map::Matrix &mat)
{

    // // Swap rows and cols because Eigen is column major, opencv is row major.
    // cv::Mat mat_cv(mat.cols(),
    //                mat.rows(),
    //                cv::DataType<grid_map::Matrix::Scalar>::type,
    //                const_cast<void *>(static_cast<const void *>(mat.data())));

    // const auto min = mat.minCoeffOfFinites();
    // const auto max = mat.maxCoeffOfFinites();

    // //  const cv::Mat mask = (mat_cv!=mat_cv);
    // // The expression above is buggy as of August 23, 2021. This is a workaround.
    // cv::Mat mat1 = mat_cv.clone();
    // cv::Mat mat2 = mat_cv.clone();
    // cv::patchNaNs(mat1, 128);
    // cv::patchNaNs(mat2, 200);
    // cv::Mat mask = mat1 == 128 & mat2 == 200;
    // // End of workaround.

    // Eigen::Map<Eigen::Matrix<grid_map::Matrix::Scalar,
    //                          Eigen::Dynamic,
    //                          Eigen::Dynamic>>
    //     mat_test(reinterpret_cast<grid_map::Matrix::Scalar *>(mat_cv.data),
    //              mat.rows(),
    //              mat.cols());

    // //  mat_cv = (mat_cv-min)/(max-min)*255;

    // cv::Mat mat_8u_cv;
    // mat_cv.convertTo(mat_8u_cv, CV_8U, 255 / (max - min), -min * 255 / (max - min));

    // cv::Mat mat_8u_cv_inpainted;

    // cv::inpaint(mat_8u_cv, mask, mat_8u_cv_inpainted, 3, cv::INPAINT_TELEA);
    // cv::Mat mat_cv_inpainted;
    // mat_8u_cv_inpainted.convertTo(mat_cv_inpainted, CV_32F);

    // Eigen::Map<Eigen::Matrix<grid_map::Matrix::Scalar,
    //                          Eigen::Dynamic,
    //                          Eigen::Dynamic>>
    //     mat_inpainted(reinterpret_cast<grid_map::Matrix::Scalar *>(mat_cv_inpainted.data),
    //                   mat.rows(),
    //                   mat.cols());

    // mat_inpainted *= (max - min) / 255;
    // mat_inpainted.array() += min;

    // // Inpainting fails at zero indices for some reason.
    // mat_inpainted.col(0) = mat_inpainted.col(1);
    // mat_inpainted.row(0) = mat_inpainted.row(1);

    // return mat_inpainted;
    grid_map::Matrix ceshi;
    return ceshi;
}

void Sampler::SampleGraphManager()
{
    SampleGraphManager(GSOGM_,
                       ElevationMap_,
                       ElevationLayer_,
                       TraversabilityLayer_,
                       CumProbLayer_,
                       CumProbRowwiseHackLayer_,
                       NormalXLayer_,
                       NormalYLayer_,
                       NormalZLayer_,
                       NewSampledVerticesNumThreshold_,
                       ValidVerticesThreshold_,
                       ValidEdgesThreshold_,
                       SampleTimeThreshold_,
                       DistanceXMax_,
                       DistanceYMax_,
                       DistanceZMax_,
                       MaxCheckPoint_,
                       RobotWidth_,
                       TraversabilityThreshold_,
                       ConnectEdgesMax_,
                       EdgeLengthMin_);
}

void Sampler::AddCurrentRobotStateToGraphManager(std::shared_ptr<VRGraphManager> TargetGraphManager,
                                                 std::string WorldFrame,
                                                 std::string RobotFrame,
                                                 grid_map::GridMap SourceElevationMap,
                                                 std::string ElevationLayer,
                                                 std::string TraversabilityLayer,
                                                 double DistanceXMax,
                                                 double DistanceYMax,
                                                 double DistanceZMax,
                                                 int MaxCheckPoint,
                                                 double RobotWidth,
                                                 double TraversabilityThreshold,
                                                 int ConnectEdgesMax,
                                                 double EdgeLengthMin,
                                                 int &EdgesCount)
{
    tf::TransformListener World_Robot_listener;
    tf::StampedTransform World_Robot_transform;
    try
    {
        World_Robot_listener.waitForTransform(WorldFrame.c_str(), RobotFrame.c_str(), ros::Time(0), ros::Duration(1.0));
        World_Robot_listener.lookupTransform(WorldFrame.c_str(), RobotFrame.c_str(),
                                             ros::Time(0), World_Robot_transform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("[Sampler_Info]: There might be something wrong when tryying update robot pose.");
        ROS_ERROR("%s", ex.what());
        // ros::Duration(1.0).sleep();
    }

    Eigen::Vector3d Position3DTem_(World_Robot_transform.getOrigin().x(),
                                   World_Robot_transform.getOrigin().y(),
                                   World_Robot_transform.getOrigin().z() - RobotHeight_);
    // StateVec StateVecTem_(Position3DTem_[0],
    //                       Position3DTem_[1],
    //                       Position3DTem_[2],
    //                       0);
    // std::vector<Vertex *> nearest_vertices;
    // nearest_vertices.clear();
    // if (TargetGraphManager->getNearestVerticesInBox(&StateVecTem_,
    //                                                 EdgeLengthMin_,
    //                                                 EdgeLengthMin_,
    //                                                 DistanceZMax_,
    //                                                 &nearest_vertices))
    // {
    //     return;
    // }
    // else
    // {
    //     Vertex *VertexNew = new Vertex(TargetGraphManager->generateVertexID(), StateVecTem_);
    //     VertexNew->resolution = 0.2;
    //     VertexNew->is_obstacle = false;
    //     TargetGraphManager->addVertex(VertexNew);
    //     // std::vector<Vertex *> VerticesTem_;
    //     // VerticesTem_.clear();
    //     // if (TargetGraphManager->getNearestVerticesInBox(&StateVecTem_,
    //     //                                                 DistanceXMax,
    //     //                                                 DistanceYMax,
    //     //                                                 DistanceZMax,
    //     //                                                 &VerticesTem_))
    //     // {
    //     // }
    // }

    if (IfConnectToTheGraphManagerSucceed(TargetGraphManager,
                                          Position3DTem_,
                                          SourceElevationMap,
                                          ElevationLayer,
                                          TraversabilityLayer,
                                          DistanceXMax,
                                          DistanceYMax,
                                          DistanceZMax,
                                          MaxCheckPoint,
                                          RobotWidth,
                                          TraversabilityThreshold,
                                          ConnectEdgesMax,
                                          EdgeLengthMin,
                                          EdgesCount))
    {
    }
}

void Sampler::AddCurrentRobotStateToGraphManager(std::shared_ptr<VRGraphManager> TargetGraphManager,
                                                 std::string WorldFrame,
                                                 std::string RobotFrame)
{
    int EdgesNumTem_ = 0;
    AddCurrentRobotStateToGraphManager(TargetGraphManager,
                                       WorldFrame,
                                       RobotFrame,
                                       ElevationMap_,
                                       ElevationLayer_,
                                       TraversabilityLayer_,
                                       DistanceXMax_,
                                       DistanceYMax_,
                                       DistanceZMax_,
                                       MaxCheckPoint_,
                                       RobotWidth_,
                                       TraversabilityThreshold_,
                                       ConnectEdgesMax_,
                                       EdgeLengthMin_,
                                       EdgesNumTem_);
}

void Sampler::ConvertPositionToIdAttribute(std::shared_ptr<VRGraphManager> SourceGraphManager,
                                           std::vector<Eigen::Vector4d> SourcePositionAttribute,
                                           double MaxDistanceToSourceGraphManager,
                                           bool IfContrainTheExplorationArea,
                                           Eigen::Matrix<double, 3, 2> ConstrainsOfTheExplorationArea,
                                           std::unordered_map<VOXEL_LOC, int> NotReachableTargetPositions3D,
                                           std::unordered_map<int, double> &OutputIdAttribute,
                                           std::unordered_map<int, Eigen::Vector4d> &OutputIdCenter)
{
    OutputIdAttribute.clear();
    OutputIdCenter.clear();
    for (size_t i = 0; i < SourcePositionAttribute.size(); i++)
    {
        if (IfContrainTheExplorationArea)
        {
            if ((SourcePositionAttribute[i][0] < ConstrainsOfTheExplorationArea(0, 0)) ||
                (SourcePositionAttribute[i][0] > ConstrainsOfTheExplorationArea(0, 1)) ||
                (SourcePositionAttribute[i][1] < ConstrainsOfTheExplorationArea(1, 0)) ||
                (SourcePositionAttribute[i][1] > ConstrainsOfTheExplorationArea(1, 1)) ||
                (SourcePositionAttribute[i][2] < ConstrainsOfTheExplorationArea(2, 0)) ||
                (SourcePositionAttribute[i][2] > ConstrainsOfTheExplorationArea(2, 1)))
            {
                continue;
            }
        }
        StateVec StateTem(SourcePositionAttribute[i][0],
                          SourcePositionAttribute[i][1],
                          SourcePositionAttribute[i][2],
                          0);

        VOXEL_LOC StatePosition3DTem_(StateTem[0], StateTem[1], (double)((int)(StateTem[2] / 0.5)) * 0.5);
        if (NotReachableTargetPositions3D.find(StatePosition3DTem_) != NotReachableTargetPositions3D.end())
        {
            continue;
        }
        Vertex *VertexTem_;
        // if (SourceGraphManager->getNearestVertex(&StateTem, &VertexTem_))
        if (SourceGraphManager->getNearestVertexInRange(&StateTem, MaxDistanceToSourceGraphManager, &VertexTem_))
        {
            OutputIdAttribute[VertexTem_->id] = SourcePositionAttribute[i][3];
            OutputIdCenter[VertexTem_->id] = SourcePositionAttribute[i];
        }
        else
        {
        }
    }
}

void Sampler::ConvertPositionToIdAttribute(std::shared_ptr<VRGraphManager> SourceGraphManager,
                                           std::vector<Eigen::Vector4d> SourcePositionAttribute,
                                           double MaxDistanceToSourceGraphManager,
                                           std::unordered_map<int, double> &OutputIdAttribute,
                                           std::unordered_map<int, Eigen::Vector4d> &OutputIdCenter)
{
    Eigen::Matrix<double, 3, 2> ConstrainsOfTheExplorationAreaTem_;
    ConstrainsOfTheExplorationAreaTem_ << 0.0, 0.0,
        0.0, 0.0,
        0.0, 0.0;
    std::unordered_map<VOXEL_LOC, int> NotReachableTargetPositions3DTem_;
    NotReachableTargetPositions3DTem_.clear();
    ConvertPositionToIdAttribute(SourceGraphManager,
                                 SourcePositionAttribute,
                                 MaxDistanceToSourceGraphManager,
                                 false,
                                 ConstrainsOfTheExplorationAreaTem_,
                                 NotReachableTargetPositions3DTem_,
                                 OutputIdAttribute,
                                 OutputIdCenter);
}

void Sampler::AddFrontierPointsToGraphManager(std::shared_ptr<VRGraphManager> TargetGraphManager,
                                              std::vector<Eigen::Vector4d> FrontierCenters,
                                              double MaxDistance)
{
    for (size_t i = 0; i < FrontierCenters.size(); i++)
    {
        Eigen::Vector3d Position3DTem_(FrontierCenters[i][0],
                                       FrontierCenters[i][1],
                                       FrontierCenters[i][2]);
        StateVec StateVecTem_(Position3DTem_[0],
                              Position3DTem_[1],
                              Position3DTem_[2],
                              0);
        std::vector<Vertex *> nearest_vertices;
        nearest_vertices.clear();
        if (TargetGraphManager->getNearestVerticesInBox(&StateVecTem_,
                                                        MaxDistance,
                                                        MaxDistance,
                                                        DistanceZMax_,
                                                        &nearest_vertices))
        {
            continue;
        }
        else
        {
            Vertex *VertexNew = new Vertex(TargetGraphManager->generateVertexID(), StateVecTem_);
            VertexNew->resolution = 0.2;
            VertexNew->is_obstacle = false;
            TargetGraphManager->addVertex(VertexNew);
            // if (MaxDistance >= EdgeLengthMin_)
            // {
            //     continue;
            // }
            // Update Edge.
            std::vector<Vertex *> NVFUETem_; // Nearest Vertices For Update Edge Tem_
            NVFUETem_.clear();
            if (TargetGraphManager->getNearestVerticesInBox(&StateVecTem_,
                                                            DistanceXMax_,
                                                            DistanceYMax_,
                                                            DistanceZMax_,
                                                            &NVFUETem_))
            {
                for (size_t j = 0; j < NVFUETem_.size(); j++)
                {
                    if (NVFUETem_[j]->id == VertexNew->id)
                    {
                        continue;
                    }
                    Eigen::Vector3d PositionOldTem_(NVFUETem_[j]->state[0],
                                                    NVFUETem_[j]->state[1],
                                                    NVFUETem_[j]->state[2]);
                    if (IfConnectTwoPointsSucceed(ElevationMap_,
                                                  ElevationLayer_,
                                                  TraversabilityLayer_,
                                                  RobotWidth_,
                                                  TraversabilityThreshold_,
                                                  PositionOldTem_,
                                                  Position3DTem_))
                    {
                        Eigen::Vector3d DistanceVectorTem_ = Position3DTem_ - PositionOldTem_;
                        double DistanceTem_ = DistanceVectorTem_.norm();
                        TargetGraphManager->addEdge(VertexNew, NVFUETem_[j], DistanceTem_);
                    }
                }
            }
        }
    }
}

void Sampler::AddFrontierPointsToGraphManager(std::shared_ptr<VRGraphManager> TargetGraphManager,
                                              std::vector<Eigen::Vector4d> FrontierCenters)
{
    for (size_t i = 0; i < FrontierCenters.size(); i++)
    {
        Eigen::Vector3d Position3DTem_(FrontierCenters[i][0],
                                       FrontierCenters[i][1],
                                       FrontierCenters[i][2]);
        grid_map::Position Position2DTem_(FrontierCenters[i][0],
                                          FrontierCenters[i][1]);
        if (!ElevationMap_.isInside(Position2DTem_))
        {
            continue;
        }

        int EdgesCountTem_ = 0;
        if (IfConnectToTheGraphManagerSucceed(TargetGraphManager,
                                              Position3DTem_,
                                              ElevationMap_,
                                              ElevationLayer_,
                                              TraversabilityLayer_,
                                              DistanceXMax_,
                                              DistanceYMax_,
                                              DistanceZMax_,
                                              MaxCheckPoint_,
                                              RobotWidth_,
                                              TraversabilityThreshold_,
                                              ConnectEdgesMax_,
                                              EdgeLengthMin_,
                                              EdgesCountTem_))
        {
        }
    }
}
void Sampler::UpdateEdges(grid_map::GridMap SourceElevationMap,
                          std::string ElevationLayer,
                          std::string TraversabilityLayer,
                          double RobotWidth,
                          double TraversabilityThreshold,
                          double DistanceXMax,
                          double DistanceYMax,
                          double DistanceZMax,
                          int MaxCheckPoint,
                          std::shared_ptr<VRGraphManager> TargetGraphManager)
{
    grid_map::Size MapSizeTem_ = SourceElevationMap.getSize();
    double MapResolutionTem_ = SourceElevationMap.getResolution();
    grid_map::Position GridMapPosition2DTem_ = SourceElevationMap.getPosition();
    double elevationTem_ = SourceElevationMap.atPosition(ElevationLayer, GridMapPosition2DTem_);
    // std::cout << " MapSizeTem_ is " << MapSizeTem_[0] << " " << MapSizeTem_[1] << " and GridMapPosition2DTem_ is " << GridMapPosition2DTem_[0] << " " << GridMapPosition2DTem_[1] << " and resolution is " << MapResolutionTem_ << "." << std::endl;

    StateVec StateTem_(GridMapPosition2DTem_[0],
                       GridMapPosition2DTem_[1],
                       elevationTem_,
                       0);
    std::vector<Vertex *> VerticesTem_;
    if (!TargetGraphManager->getNearestVerticesInBox(&StateTem_,
                                                     (double)(MapSizeTem_[0]) * 0.5 * MapResolutionTem_,
                                                     (double)(MapSizeTem_[1]) * 0.5 * MapResolutionTem_,
                                                     std::min(((double)(MapSizeTem_[0]) + (double)(MapSizeTem_[1])) * 0.25 * 0.5 * MapResolutionTem_, DistanceZMax),
                                                     &VerticesTem_))
    {
        ROS_ERROR("[Sampler_Info]: Can not extract vertices when try to update edges.");
        return;
    }
    std::unordered_map<int, std::set<int>> CheckedIdsTem_;
    int EdgesCountTem_ = 0;
    for (size_t i = 0; i < VerticesTem_.size(); i++)
    {
        Vertex *Vertex1Tem_ = VerticesTem_[i];
        Eigen::Vector3d Point1Tem_(Vertex1Tem_->state[0],
                                   Vertex1Tem_->state[1],
                                   Vertex1Tem_->state[2]);
        grid_map::Position Position2DTem_(Vertex1Tem_->state[0], Vertex1Tem_->state[1]);
        if (SourceElevationMap.isInside(Position2DTem_))
        {
            Eigen::Array2i IndexTem_;
            SourceElevationMap.getIndex(Position2DTem_, IndexTem_);
            double TraversabilityTem_ = SourceElevationMap.at(TraversabilityLayer, IndexTem_);
            if (TraversabilityTem_ < TraversabilityThreshold)
            {
                std::vector<Vertex *> VerticesNeedToBeRemovedTem_;
                VerticesNeedToBeRemovedTem_.push_back(Vertex1Tem_);
                TargetGraphManager->removeVerticesInBox(VerticesNeedToBeRemovedTem_,
                                                        Point1Tem_,
                                                        0.1,
                                                        0.1,
                                                        0.1);
                continue;
            }
        }

        std::vector<Vertex *> Vertices2Tem_;
        Vertices2Tem_.clear();
        if (!TargetGraphManager->getNearestVerticesInBox(&(Vertex1Tem_->state),
                                                         DistanceXMax,
                                                         DistanceYMax,
                                                         DistanceZMax,
                                                         &Vertices2Tem_))
        {
            continue;
        }
        for (size_t j = 0; j < Vertices2Tem_.size(); j++)
        {
            Vertex *Vertex2Tem_ = Vertices2Tem_[j];
            if (Vertex1Tem_->id == Vertex2Tem_->id)
            {
                continue;
            }
            int MinIdTem_ = -1;
            int MaxIdTem_ = -1;
            if (Vertex1Tem_->id < Vertex2Tem_->id)
            {
                MinIdTem_ = Vertex1Tem_->id;
                MaxIdTem_ = Vertex2Tem_->id;
            }
            else
            {
                MinIdTem_ = Vertex2Tem_->id;
                MaxIdTem_ = Vertex1Tem_->id;
            }

            if (TargetGraphManager->Graph_->adjacencyList_[MinIdTem_].find(MaxIdTem_) != TargetGraphManager->Graph_->adjacencyList_[MinIdTem_].end())
            {
                CheckedIdsTem_[MinIdTem_].insert(MaxIdTem_);
                continue;
            }

            if (CheckedIdsTem_.find(MinIdTem_) == CheckedIdsTem_.end())
            {
                std::set<int> SetTem_;
                SetTem_.clear();
                CheckedIdsTem_[MinIdTem_] = SetTem_;
                CheckedIdsTem_[MinIdTem_].insert(MaxIdTem_);
            }
            else if (CheckedIdsTem_[MinIdTem_].find(MaxIdTem_) == CheckedIdsTem_[MinIdTem_].end())
            {
                CheckedIdsTem_[MinIdTem_].insert(MaxIdTem_);
            }
            else
            {
                continue;
            }

            Eigen::Vector3d Point2Tem_(Vertex2Tem_->state[0],
                                       Vertex2Tem_->state[1],
                                       Vertex2Tem_->state[2]);

            if (IfConnectTwoPointsSucceed(SourceElevationMap,
                                          ElevationLayer,
                                          TraversabilityLayer,
                                          RobotWidth,
                                          TraversabilityThreshold,
                                          Point1Tem_,
                                          Point2Tem_))
            {
                Eigen::Vector3d Distance3DTem_ = Point2Tem_ - Point1Tem_;
                TargetGraphManager->addEdge(Vertex1Tem_, Vertex2Tem_, Distance3DTem_.norm());
                EdgesCountTem_++;
            }
        }
    }
    // ROS_INFO("[Sampler_Info]: Add %d edgs to the sample map by update edges process.", EdgesCountTem_);
}

void Sampler::UpdateEdges()
{
    UpdateEdges(ElevationMap_,
                ElevationLayer_,
                TraversabilityLayer_,
                RobotWidth_,
                TraversabilityThreshold_,
                DistanceXMax_,
                DistanceYMax_,
                DistanceZMax_,
                MaxCheckPoint_,
                GSOGM_);
}