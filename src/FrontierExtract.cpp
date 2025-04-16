#include <vrmapping/FrontierExtract.h>
FrontierExtract::FrontierExtract()
{
    FrontiersSet_.clear();
    UF_.reset(new UF(0));
    IdAndAttributeUpdatedToCenterId_.clear();
}

FrontierExtract::~FrontierExtract()
{
}

void FrontierExtract::ExtractFrontiers(grid_map::GridMap SourceElevationMap,
                                       std::string ElevationLayer,
                                       std::string TraversabilityLayer,
                                       std::string TraversabilitySupplementLayer,
                                       std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                       double ElevationMapResolution,
                                       double TraversabilityThreshold,
                                       double VRMapResolutionMax,
                                       double LimitBoxZ,
                                       std::set<Eigen::Vector3d, setcomp> &FrontiersSet)
{
    grid_map::Position Position;

    for (grid_map::GridMapIterator iterator(SourceElevationMap); !iterator.isPastEnd(); ++iterator)
    {
        double elevation = SourceElevationMap.at(ElevationLayer.c_str(), *iterator);
        double traversability_score = SourceElevationMap.at(TraversabilityLayer.c_str(), *iterator);
        double traversability_supplementary_score = SourceElevationMap.at(TraversabilitySupplementLayer.c_str(), *iterator);

        if (elevation != elevation)
        {
            continue;
        }

        if (traversability_score != traversability_score)
        {
            continue;
        }

        if (traversability_supplementary_score != traversability_supplementary_score)
        {
            continue;
        }

        traversability_score = std::max(traversability_score, traversability_supplementary_score);
        if (traversability_score < TraversabilityThreshold)
        {
            continue;
        }
        SourceElevationMap.getPosition(*iterator, Position);
        bool IsFrontier_;
        try
        {
            IsFrontier_ = !IfPointInternal(Position[0],
                                           Position[1],
                                           SourceElevationMap,
                                           ElevationMapResolution,
                                           ElevationLayer,
                                           TraversabilityLayer,
                                           TraversabilitySupplementLayer);
        }
        catch (const std::exception &e)
        {
            // std::cerr << e.what() << '\n';
            ROS_ERROR("%s", e.what());
            ROS_ERROR("[FrontierExtrack_Info]: There is something wrong when check if point is internal.");
            IsFrontier_ = true;
        }
        if (!IsFrontier_)
        {
            continue;
        }
        Eigen::Vector3d Position3D_(Position[0],
                                    Position[1],
                                    elevation);

        bool IsFrontierVertex_ = !IfVertexInternal(SourceVRGraphManager,
                                                   Position3D_,
                                                   VRMapResolutionMax,
                                                   LimitBoxZ,
                                                   8);
        if (!IsFrontierVertex_)
        {
            continue;
        }
        else
        {
            StateVec StateTem_(Position[0],
                               Position[1],
                               elevation,
                               0);
            Vertex *NearestVertexPtr_;
            if (SourceVRGraphManager->getNearestVertex(&StateTem_,
                                                       &NearestVertexPtr_))
            {
                if (NearestVertexPtr_->is_obstacle)
                {
                    continue;
                }
                Eigen::Vector3d PositionSaved_(NearestVertexPtr_->state[0],
                                               NearestVertexPtr_->state[1],
                                               NearestVertexPtr_->state[2]);
                FrontiersSet.insert(PositionSaved_);
            }
            else
            {
                ROS_ERROR("[FrontierExtract_INFO]: Can not find the nearest point.");
            }
        }
    }
}

void FrontierExtract::ExtractFrontiers(grid_map::GridMap SourceElevationMap,
                                       std::string ElevationLayer,
                                       std::string TraversabilityLayer,
                                       std::string TraversabilitySupplementLayer,
                                       std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                       double ElevationMapResolution,
                                       double TraversabilityThreshold,
                                       double VRMapResolutionMax,
                                       double VRMapResolutionMin,
                                       double LimitBoxZ,
                                       std::set<Eigen::Vector3d, setcomp> &FrontiersSet)
{
    grid_map::Position Position;
    for (grid_map::GridMapIterator iterator(SourceElevationMap); !iterator.isPastEnd(); ++iterator)
    {
        double elevation = SourceElevationMap.at(ElevationLayer.c_str(), *iterator);
        double traversability_score = SourceElevationMap.at(TraversabilityLayer.c_str(), *iterator);
        double traversability_supplementary_score = SourceElevationMap.at(TraversabilitySupplementLayer.c_str(), *iterator);

        if (elevation != elevation)
        {
            continue;
        }
        if (traversability_score != traversability_score)
        {
            continue;
        }
        if (traversability_supplementary_score != traversability_supplementary_score)
        {
            continue;
        }
        traversability_score = std::max(traversability_score, traversability_supplementary_score);
        if (traversability_score < TraversabilityThreshold)
        {
            continue;
        }
        SourceElevationMap.getPosition(*iterator, Position);
        bool IsFrontier_;
        try
        {
            IsFrontier_ = !IfPointInternal(Position[0],
                                           Position[1],
                                           SourceElevationMap,
                                           ElevationMapResolution,
                                           ElevationLayer,
                                           TraversabilityLayer,
                                           TraversabilitySupplementLayer);
        }
        catch (const std::exception &e)
        {
            // std::cerr << e.what() << '\n';
            ROS_ERROR("%s", e.what());
            ROS_ERROR("[FrontierExtrack_Info]: There is something wrong when check if point is internal.");
            IsFrontier_ = true;
        }
        if (!IsFrontier_)
        {
            continue;
        }
        Eigen::Vector3d Position3D_(Position[0],
                                    Position[1],
                                    elevation);

        // bool IsFrontierVertex_ = !IfVertexInternal(SourceVRGraphManager,
        //                                            Position3D_,
        //                                            VRMapResolutionMax,
        //                                            LimitBoxZ,
        //                                            8);

        StateVec StateTem_(Position[0],
                           Position[1],
                           elevation,
                           0);
        Vertex *NearestVertexPtr_;

        if (SourceVRGraphManager->getNearestVertex(&StateTem_,
                                                   &NearestVertexPtr_))
        {
            if (NearestVertexPtr_->is_obstacle)
            {
                continue;
            }
        }
        else
        {
            ROS_ERROR("[FrontierExtract_INFO]: Can not find the nearest point.");
        }
        Eigen::Vector3d PositionSaved_(NearestVertexPtr_->state[0],
                                       NearestVertexPtr_->state[1],
                                       NearestVertexPtr_->state[2]);
        bool IsFrontierVertex_ = !IfVertexInternalByGraph(SourceVRGraphManager,
                                                          PositionSaved_,
                                                          NearestVertexPtr_->resolution,
                                                          VRMapResolutionMax,
                                                          VRMapResolutionMin,
                                                          LimitBoxZ);
        if (!IsFrontierVertex_)
        {
            continue;
        }
        else
        {

            FrontiersSet.insert(PositionSaved_);
        }
    }
}

void FrontierExtract::ExtractFrontiers(grid_map::GridMap SourceElevationMap,
                                       std::string ElevationLayer,
                                       std::string TraversabilityLayer,
                                       std::string TraversabilitySupplementLayer,
                                       std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                       double ElevationMapResolution,
                                       double TraversabilityThreshold,
                                       double VRMapResolutionMax,
                                       double VRMapResolutionMin,
                                       double LimitBoxZ,
                                       std::set<int> &FrontiersIdSet)
{
    grid_map::Position Position;
    for (grid_map::GridMapIterator iterator(SourceElevationMap); !iterator.isPastEnd(); ++iterator)
    {
        double elevation = SourceElevationMap.at(ElevationLayer.c_str(), *iterator);
        double traversability_score = SourceElevationMap.at(TraversabilityLayer.c_str(), *iterator);
        double traversability_supplementary_score = SourceElevationMap.at(TraversabilitySupplementLayer.c_str(), *iterator);

        if (elevation != elevation)
        {
            continue;
        }
        if (traversability_score != traversability_score)
        {
            continue;
        }
        if (traversability_supplementary_score != traversability_supplementary_score)
        {
            continue;
        }
        traversability_score = std::max(traversability_score, traversability_supplementary_score);
        if (traversability_score < TraversabilityThreshold)
        {
            continue;
        }
        SourceElevationMap.getPosition(*iterator, Position);
        bool IsFrontier_;
        try
        {
            IsFrontier_ = !IfPointInternal(Position[0],
                                           Position[1],
                                           SourceElevationMap,
                                           ElevationMapResolution,
                                           ElevationLayer,
                                           TraversabilityLayer,
                                           TraversabilitySupplementLayer);
        }
        catch (const std::exception &e)
        {
            // std::cerr << e.what() << '\n';
            ROS_ERROR("%s", e.what());
            ROS_ERROR("[FrontierExtrack_Info]: There is something wrong when check if point is internal.");
            IsFrontier_ = true;
        }
        if (!IsFrontier_)
        {
            continue;
        }
        Eigen::Vector3d Position3D_(Position[0],
                                    Position[1],
                                    elevation);

        StateVec StateTem_(Position[0],
                           Position[1],
                           elevation,
                           0);
        Vertex *NearestVertexPtr_;

        if (SourceVRGraphManager->getNearestVertex(&StateTem_,
                                                   &NearestVertexPtr_))
        {
            if (NearestVertexPtr_->is_obstacle)
            {
                continue;
            }
        }
        else
        {
            ROS_ERROR("[FrontierExtract_INFO]: Can not find the nearest point.");
        }
        bool IsFrontierVertex_ = !IfVertexInternalByGraph(SourceVRGraphManager,
                                                          NearestVertexPtr_->id,
                                                          NearestVertexPtr_->resolution,
                                                          VRMapResolutionMax,
                                                          VRMapResolutionMin,
                                                          LimitBoxZ);
        if (!IsFrontierVertex_)
        {
            continue;
        }
        else
        {

            FrontiersIdSet.insert(NearestVertexPtr_->id);
        }
    }
}

void FrontierExtract::ExtractFrontiers(grid_map::GridMap SourceElevationMap,
                                       std::string ElevationLayer,
                                       std::string FrontierLayer,
                                       std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                       double VRMapResolutionMax,
                                       double VRMapResolutionMin,
                                       double LimitBoxZ,
                                       std::set<Eigen::Vector3d, setcomp> &FrontiersSet)
{
    grid_map::Position Position;
    for (grid_map::GridMapIterator iterator(SourceElevationMap); !iterator.isPastEnd(); ++iterator)
    {
        double elevation = SourceElevationMap.at(ElevationLayer.c_str(), *iterator);
        double frontier = SourceElevationMap.at(FrontierLayer.c_str(), *iterator);

        if (elevation != elevation)
        {
            continue;
        }
        if (frontier != frontier)
        {
            continue;
        }
        if (frontier == 0)
        {
            continue;
        }

        SourceElevationMap.getPosition(*iterator, Position);

        Eigen::Vector3d Position3D_(Position[0],
                                    Position[1],
                                    elevation);

        StateVec StateTem_(Position[0],
                           Position[1],
                           elevation,
                           0);
        Vertex *NearestVertexPtr_;

        if (SourceVRGraphManager->getNearestVertex(&StateTem_,
                                                   &NearestVertexPtr_))
        {
            if (NearestVertexPtr_->is_obstacle)
            {
                continue;
            }
        }
        else
        {
            ROS_ERROR("[FrontierExtract_INFO]: Can not find the nearest point.");
        }
        Eigen::Vector3d PositionSaved_(NearestVertexPtr_->state[0],
                                       NearestVertexPtr_->state[1],
                                       NearestVertexPtr_->state[2]);
        // bool IsFrontierVertex_ = !IfVertexInternalByGraph(SourceVRGraphManager,
        //                                                   PositionSaved_,
        //                                                   NearestVertexPtr_->resolution,
        //                                                   VRMapResolutionMax,
        //                                                   VRMapResolutionMin,
        //                                                   LimitBoxZ);
        bool IsFrontierVertex_ = true;
        if (!IsFrontierVertex_)
        {
            continue;
        }
        else
        {

            FrontiersSet.insert(PositionSaved_);
        }
    }
}

void FrontierExtract::ExtractFrontiers(grid_map::GridMap SourceElevationMap,
                                       std::string ElevationLayer,
                                       std::string FrontierLayer,
                                       std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                       double VRMapResolutionMax,
                                       double VRMapResolutionMin,
                                       double LimitBoxZ,
                                       std::set<int> &FrontiersIdSet)
{
    grid_map::Position Position;
    for (grid_map::GridMapIterator iterator(SourceElevationMap); !iterator.isPastEnd(); ++iterator)
    {
        double elevation = SourceElevationMap.at(ElevationLayer.c_str(), *iterator);
        double frontier = SourceElevationMap.at(FrontierLayer.c_str(), *iterator);

        if (elevation != elevation)
        {
            continue;
        }
        if (frontier != frontier)
        {
            continue;
        }
        if (frontier == 0)
        {
            continue;
        }

        SourceElevationMap.getPosition(*iterator, Position);

        Eigen::Vector3d Position3D_(Position[0],
                                    Position[1],
                                    elevation);

        StateVec StateTem_(Position[0],
                           Position[1],
                           elevation,
                           0);
        Vertex *NearestVertexPtr_;

        if (SourceVRGraphManager->getNearestVertex(&StateTem_,
                                                   &NearestVertexPtr_))
        {
            if (NearestVertexPtr_->is_obstacle)
            {
                continue;
            }
        }
        else
        {
            ROS_ERROR("[FrontierExtract_INFO]: Can not find the nearest point.");
        }
        bool IsFrontierVertex_ = !IfVertexInternalByGraph(SourceVRGraphManager,
                                                          NearestVertexPtr_->id,
                                                          NearestVertexPtr_->resolution,
                                                          VRMapResolutionMax,
                                                          VRMapResolutionMin,
                                                          LimitBoxZ);
        if (!IsFrontierVertex_)
        {
            continue;
        }
        else
        {

            FrontiersIdSet.insert(NearestVertexPtr_->id);
        }
    }
}

bool FrontierExtract::IfPointInternal(double x, double y,
                                      grid_map::GridMap SourceElevationMap,
                                      double ElevationMapResolution,
                                      std::string ElevationLayer,
                                      std::string TraversabilityLayer,
                                      std::string TraversabilitySupplementLayer)
{
    // grid_map::Position position_temp0(x, y);
    grid_map::Position position_temp1(x - ElevationMapResolution, y - ElevationMapResolution);
    grid_map::Position position_temp2(x - ElevationMapResolution, y);
    grid_map::Position position_temp3(x - ElevationMapResolution, y + ElevationMapResolution);

    grid_map::Position position_temp4(x, y - ElevationMapResolution);
    grid_map::Position position_temp5(x, y);
    grid_map::Position position_temp6(x, y + ElevationMapResolution);

    grid_map::Position position_temp7(x + ElevationMapResolution, y - ElevationMapResolution);
    grid_map::Position position_temp8(x + ElevationMapResolution, y);
    grid_map::Position position_temp9(x + ElevationMapResolution, y + ElevationMapResolution);

    std::vector<grid_map::Position> position_vector;
    position_vector.clear();
    position_vector.push_back(position_temp1);
    position_vector.push_back(position_temp2);
    position_vector.push_back(position_temp3);
    position_vector.push_back(position_temp4);
    position_vector.push_back(position_temp5);
    position_vector.push_back(position_temp6);
    position_vector.push_back(position_temp7);
    position_vector.push_back(position_temp8);
    position_vector.push_back(position_temp9);
    int inside_count = 0;
    int valid_count = 0;

    std::vector<Eigen::Array2i> index_vector;

    for (std::vector<grid_map::Position>::iterator it = position_vector.begin(); it != position_vector.end(); it++)
    {
        if (SourceElevationMap.isInside(*it))
        {
            inside_count++;
            Eigen::Array2i Index;
            SourceElevationMap.getIndex(*it, Index);
            if ((SourceElevationMap.isValid(Index, ElevationLayer.c_str())) &&
                (SourceElevationMap.isValid(Index, TraversabilityLayer.c_str())) &&
                (SourceElevationMap.isValid(Index, TraversabilitySupplementLayer.c_str())))
            {
                valid_count++;
            }
        }
    }
    if (inside_count > 8)
    {
        if (valid_count > 8)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }
    return false;
}

bool FrontierExtract::IfVertexInternal(std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                       Eigen::Vector3d VertexPosition,
                                       double VRMapResolutionMax,
                                       double LimitBoxZ,
                                       double NeighboursCountThreshold)
{
    int row = 0;
    int col = 0;

    if (VertexPosition[0] < -0.5 * VRMapResolutionMax)
    {
        row = (VertexPosition[0] + VRMapResolutionMax / 2) / VRMapResolutionMax - 1;
    }
    else
    {
        row = (VertexPosition[0] + VRMapResolutionMax / 2) / VRMapResolutionMax;
    }

    if (VertexPosition[1] < -0.5 * VRMapResolutionMax)
    {
        col = (VertexPosition[1] + VRMapResolutionMax / 2) / VRMapResolutionMax - 1;
    }
    else
    {
        col = (VertexPosition[1] + VRMapResolutionMax / 2) / VRMapResolutionMax;
    }

    int NeighboursCount_ = 0;
    for (int iRow = -1; iRow < 2; iRow++)
    {
        for (int iCol = -1; iCol < 2; iCol++)
        {
            StateVec StateTem_((row + iRow) * VRMapResolutionMax,
                               (col + iCol) * VRMapResolutionMax,
                               VertexPosition[2], 0);
            std::vector<Vertex *> nearest_vertices_MultiResolutionMap;
            nearest_vertices_MultiResolutionMap.clear();
            if (SourceVRGraphManager->getNearestVerticesInBox(&StateTem_,
                                                              VRMapResolutionMax / 2,
                                                              VRMapResolutionMax / 2,
                                                              LimitBoxZ,
                                                              &nearest_vertices_MultiResolutionMap))
            {
                NeighboursCount_++;
            }
            else
            {
            }
        }
    }
    if (NeighboursCount_ > NeighboursCountThreshold)
    {
        return true;
    }
    else
    {
        return false;
    }
    return false;
}

bool FrontierExtract::IfVertexInternalByGraph(std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                              Eigen::Vector3d VertexPosition,
                                              double VRMapResolutionMax,
                                              double VRMapResolutionMin,
                                              double LimitBoxZ)
{
    bool ExistLeftFront_ = false;
    bool ExistLeftCenter_ = false;
    bool ExistLeftBack_ = false;
    bool ExistCenterFront_ = false;
    bool ExistCenterCenter_ = false;
    bool ExistCenterBack_ = false;
    bool ExistRightFront_ = false;
    bool ExistRightCenter_ = false;
    bool ExistRightBack_ = false;

    StateVec StateTem_(VertexPosition[0],
                       VertexPosition[1],
                       VertexPosition[2], 0);
    std::vector<Vertex *> nearest_vertices_MultiResolutionMap;
    nearest_vertices_MultiResolutionMap.clear();
    if (SourceVRGraphManager->getNearestVerticesInBox(&StateTem_,
                                                      VRMapResolutionMax + 0.3 * VRMapResolutionMin,
                                                      VRMapResolutionMax + 0.3 * VRMapResolutionMin,
                                                      LimitBoxZ,
                                                      &nearest_vertices_MultiResolutionMap))
    {
        if (nearest_vertices_MultiResolutionMap.size() < 9)
        {
            return false;
        }
        for (size_t iTem_ = 0; iTem_ < nearest_vertices_MultiResolutionMap.size(); iTem_++)
        {
            int xIndex = -2;
            int yIndex = -2;
            if (nearest_vertices_MultiResolutionMap[iTem_]->state[0] >
                (VertexPosition[0] + 0.9 * VRMapResolutionMin))
            {
                xIndex = 1;
            }
            else if (nearest_vertices_MultiResolutionMap[iTem_]->state[0] <
                     (VertexPosition[0] - 0.9 * VRMapResolutionMin))
            {
                xIndex = -1;
            }
            else
            {
                xIndex = 0;
            }

            if (nearest_vertices_MultiResolutionMap[iTem_]->state[1] >
                (VertexPosition[1] + 0.2 * VRMapResolutionMin))
            {
                yIndex = 1;
            }
            else if (nearest_vertices_MultiResolutionMap[iTem_]->state[1] <
                     (VertexPosition[1] - 0.2 * VRMapResolutionMin))
            {
                yIndex = -1;
            }
            else
            {
                yIndex = 0;
            }

            switch (xIndex)
            {
            case -1:
                // back
                switch (yIndex)
                {
                case -1:
                    // right back
                    ExistRightBack_ = true;
                    break;
                case 0:
                    // center back
                    ExistCenterBack_ = true;
                    break;
                case 1:
                    // left back
                    ExistLeftBack_ = true;
                    break;

                default:
                    ROS_ERROR("[FrontierExtract_Info]: yIndex wrong when judge if vertex internal.");
                    break;
                }
                break;
            case 0:
                // center
                switch (yIndex)
                {
                case -1:
                    // right center
                    ExistRightCenter_ = true;
                    break;
                case 0:
                    // center center
                    ExistCenterCenter_ = true;
                    break;
                case 1:
                    // left center
                    ExistLeftCenter_ = true;
                    break;

                default:
                    ROS_ERROR("[FrontierExtract_Info]: yIndex wrong when judge if vertex internal.");
                    break;
                }
                break;
            case 1:
                // front
                switch (yIndex)
                {
                case -1:
                    // right front
                    ExistRightFront_ = true;
                    break;
                case 0:
                    // center front
                    ExistCenterFront_ = true;
                    break;
                case 1:
                    // left front
                    ExistLeftFront_ = true;
                    break;

                default:
                    ROS_ERROR("[FrontierExtract_Info]: yIndex wrong when judge if vertex internal.");
                    break;
                }
                break;

            default:
                ROS_ERROR("[FrontierExtract_Info]: xIndex wrong when judge if vertex internal.");
                break;
            }
            //
        }
        // std::cout << "vertex position is "
        //           << VertexPosition[0] << " " << VertexPosition[1] << " " << VertexPosition[2]
        //           << " and  exist is "
        //           << ExistLeftFront_ << ExistLeftCenter_ << ExistLeftBack_
        //           << ExistCenterFront_ << ExistCenterCenter_ << ExistCenterBack_
        //           << ExistRightFront_ << ExistRightCenter_ << ExistRightBack_ << std::endl;
        if (ExistLeftFront_ &&
            ExistLeftCenter_ &&
            ExistLeftBack_ &&
            ExistCenterFront_ &&
            ExistCenterCenter_ &&
            ExistCenterBack_ &&
            ExistRightFront_ &&
            ExistRightCenter_ &&
            ExistRightBack_)
        {
            return true;
        }
        else
        {
            return false;
        }
        // if (ExistLeftFront_ &&
        //     ExistLeftBack_ &&
        //     ExistRightFront_ &&
        //     ExistRightBack_ &&
        //     ExistCenterFront_ &&
        //     ExistCenterBack_)
        // {
        //     return true;
        // }
        // else
        // {
        //     return false;
        // }
    }
    else
    {
        return true;
        ROS_ERROR("[FrontierExtract_Info]:Can not find nearest vertices.");
    }
    return true;
}

bool FrontierExtract::IfVertexInternalByGraph(std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                              Eigen::Vector3d VertexPosition,
                                              double VertexResolution,
                                              double VRMapResolutionMax,
                                              double VRMapResolutionMin,
                                              double LimitBoxZ)
{
    bool ExistLeftFront_ = false;
    bool ExistLeftCenter_ = false;
    bool ExistLeftBack_ = false;
    bool ExistCenterFront_ = false;
    bool ExistCenterCenter_ = false;
    bool ExistCenterBack_ = false;
    bool ExistRightFront_ = false;
    bool ExistRightCenter_ = false;
    bool ExistRightBack_ = false;

    StateVec StateTem_(VertexPosition[0],
                       VertexPosition[1],
                       VertexPosition[2], 0);

    Vertex *VertexCheckedTem_;
    if (!SourceVRGraphManager->getNearestVertex(&StateTem_, &VertexCheckedTem_))
    {
        return true;
    }
    Eigen::Vector2d Vertex1PositionTem_(VertexCheckedTem_->state[0],
                                        VertexCheckedTem_->state[1]);
    double VR1Tem_ = VertexCheckedTem_->resolution;

    std::vector<Vertex *> nearest_vertices_MultiResolutionMap;
    nearest_vertices_MultiResolutionMap.clear();
    std::vector<Eigen::Vector3d> VerteicesPosition3DGeneratedTem_;
    VerteicesPosition3DGeneratedTem_.clear();
    if (SourceVRGraphManager->getNearestVerticesInBox(&StateTem_,
                                                      VRMapResolutionMax + 0.3 * VRMapResolutionMin,
                                                      VRMapResolutionMax + 0.3 * VRMapResolutionMin,
                                                      LimitBoxZ,
                                                      &nearest_vertices_MultiResolutionMap))
    {
        if (nearest_vertices_MultiResolutionMap.size() < 9)
        {
            return false;
        }
        for (size_t iTem_ = 0; iTem_ < nearest_vertices_MultiResolutionMap.size(); iTem_++)
        {
            Eigen::Vector2d Vertex2PositionTem_(nearest_vertices_MultiResolutionMap[iTem_]->state[0],
                                                nearest_vertices_MultiResolutionMap[iTem_]->state[1]);
            Eigen::Vector2d DistanceVectorMiddle_ = Vertex2PositionTem_ - Vertex1PositionTem_;
            bool IfCouldConnectTwoVertices = false;

            double VR2Tem_ = nearest_vertices_MultiResolutionMap[iTem_]->resolution;
            double DFEIX = abs(DistanceVectorMiddle_[0]); // Distance For Edge In X-axe
            double DFEIY = abs(DistanceVectorMiddle_[1]); // Distance For Edge In Y-axe
            if ((DFEIX <= (VR1Tem_ + VR2Tem_) / 2 + 0.02) &&
                (DFEIY <= (VR1Tem_ + VR2Tem_) / 2 + 0.02))
            {
                IfCouldConnectTwoVertices = true;
            }
            if (!IfCouldConnectTwoVertices)
            {
                continue;
            }
            if (nearest_vertices_MultiResolutionMap[iTem_]->resolution > VertexResolution)
            {
                double DivideTimesTem_ = (double)(nearest_vertices_MultiResolutionMap[iTem_]->resolution / VertexResolution);
                double GeneratedResolutionTem_ = nearest_vertices_MultiResolutionMap[iTem_]->resolution / DivideTimesTem_;
                double xMinTem_ = nearest_vertices_MultiResolutionMap[iTem_]->state[0] -
                                  nearest_vertices_MultiResolutionMap[iTem_]->resolution / 2 +
                                  GeneratedResolutionTem_ / 2;
                double yMinTem_ = nearest_vertices_MultiResolutionMap[iTem_]->state[1] -
                                  nearest_vertices_MultiResolutionMap[iTem_]->resolution / 2 +
                                  GeneratedResolutionTem_ / 2;
                for (int iRowTem_ = 0; iRowTem_ < DivideTimesTem_; iRowTem_++)
                {
                    for (int iColTem_ = 0; iColTem_ < DivideTimesTem_; iColTem_++)
                    {
                        Eigen::Vector3d VP3DGTem_(xMinTem_ + iRowTem_ * GeneratedResolutionTem_,
                                                  yMinTem_ + iColTem_ * GeneratedResolutionTem_,
                                                  nearest_vertices_MultiResolutionMap[iTem_]->state[2]);
                        VerteicesPosition3DGeneratedTem_.push_back(VP3DGTem_);
                    }
                }
            }
            else
            {
                Eigen::Vector3d VP3DGTem_(nearest_vertices_MultiResolutionMap[iTem_]->state[0],
                                          nearest_vertices_MultiResolutionMap[iTem_]->state[1],
                                          nearest_vertices_MultiResolutionMap[iTem_]->state[2]);
                VerteicesPosition3DGeneratedTem_.push_back(VP3DGTem_);
            }
        }
    }
    else
    {
        return true;
        ROS_ERROR("[FrontierExtract_Info]:Can not find nearest vertices.");
    }

    for (size_t iTem_ = 0; iTem_ < VerteicesPosition3DGeneratedTem_.size(); iTem_++)
    {
        int xIndex = -2;
        int yIndex = -2;
        if (VerteicesPosition3DGeneratedTem_[iTem_][0] >
            (VertexPosition[0] + VertexResolution / 2))
        {
            xIndex = 1;
        }
        else if (VerteicesPosition3DGeneratedTem_[iTem_][0] <
                 (VertexPosition[0] - VertexResolution / 2))
        {
            xIndex = -1;
        }
        else
        {
            xIndex = 0;
        }

        if (VerteicesPosition3DGeneratedTem_[iTem_][1] >
            (VertexPosition[1] + VertexResolution / 2))
        {
            yIndex = 1;
        }
        else if (VerteicesPosition3DGeneratedTem_[iTem_][1] <
                 (VertexPosition[1] - VertexResolution / 2))
        {
            yIndex = -1;
        }
        else
        {
            yIndex = 0;
        }

        switch (xIndex)
        {
        case -1:
            // back
            switch (yIndex)
            {
            case -1:
                // right back
                ExistRightBack_ = true;
                break;
            case 0:
                // center back
                ExistCenterBack_ = true;
                break;
            case 1:
                // left back
                ExistLeftBack_ = true;
                break;

            default:
                ROS_ERROR("[FrontierExtract_Info]: yIndex wrong when judge if vertex internal.");
                break;
            }
            break;
        case 0:
            // center
            switch (yIndex)
            {
            case -1:
                // right center
                ExistRightCenter_ = true;
                break;
            case 0:
                // center center
                ExistCenterCenter_ = true;
                break;
            case 1:
                // left center
                ExistLeftCenter_ = true;
                break;

            default:
                ROS_ERROR("[FrontierExtract_Info]: yIndex wrong when judge if vertex internal.");
                break;
            }
            break;
        case 1:
            // front
            switch (yIndex)
            {
            case -1:
                // right front
                ExistRightFront_ = true;
                break;
            case 0:
                // center front
                ExistCenterFront_ = true;
                break;
            case 1:
                // left front
                ExistLeftFront_ = true;
                break;

            default:
                ROS_ERROR("[FrontierExtract_Info]: yIndex wrong when judge if vertex internal.");
                break;
            }
            break;

        default:
            ROS_ERROR("[FrontierExtract_Info]: xIndex wrong when judge if vertex internal.");
            break;
        }
        //
    }
    // std::cout << "vertex position is "
    //           << VertexPosition[0] << " " << VertexPosition[1] << " " << VertexPosition[2]
    //           << " resolution is " << VertexResolution
    //           << " and  exist is "
    //           << ExistLeftFront_ << ExistLeftCenter_ << ExistLeftBack_
    //           << ExistCenterFront_ << ExistCenterCenter_ << ExistCenterBack_
    //           << ExistRightFront_ << ExistRightCenter_ << ExistRightBack_ << std::endl;
    if (ExistLeftFront_ &&
        ExistLeftCenter_ &&
        ExistLeftBack_ &&
        ExistCenterFront_ &&
        ExistCenterCenter_ &&
        ExistCenterBack_ &&
        ExistRightFront_ &&
        ExistRightCenter_ &&
        ExistRightBack_)
    {
        return true;
    }
    else
    {
        return false;
    }
    return true;
}

bool FrontierExtract::IfVertexInternalByGraph(std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                              int VertexId,
                                              double VertexResolution,
                                              double VRMapResolutionMax,
                                              double VRMapResolutionMin,
                                              double LimitBoxZ)
{
    bool ExistLeftFront_ = false;
    bool ExistLeftCenter_ = false;
    bool ExistLeftBack_ = false;
    bool ExistCenterFront_ = false;
    bool ExistCenterCenter_ = false;
    bool ExistCenterBack_ = false;
    bool ExistRightFront_ = false;
    bool ExistRightCenter_ = false;
    bool ExistRightBack_ = false;
    if (SourceVRGraphManager->vertices_map_.find(VertexId) == SourceVRGraphManager->vertices_map_.end())
    {
        return true;
    }
    Vertex *VertexCheckedTem_ = SourceVRGraphManager->vertices_map_[VertexId];
    Eigen::Vector3d VertexPosition(VertexCheckedTem_->state[0],
                                   VertexCheckedTem_->state[1],
                                   VertexCheckedTem_->state[2]);

    Eigen::Vector2d Vertex1PositionTem_(VertexCheckedTem_->state[0],
                                        VertexCheckedTem_->state[1]);
    double VR1Tem_ = VertexCheckedTem_->resolution;

    std::vector<Eigen::Vector3d> VerteicesPosition3DGeneratedTem_;
    VerteicesPosition3DGeneratedTem_.clear();
    auto NeighboursTem_ = SourceVRGraphManager->Graph_->adjacencyList_[VertexId];

    if (NeighboursTem_.size() < 9)
    {
        return false;
    }
    for (auto &IterateNeighboursTem__ : NeighboursTem_)
    {
        Vertex *NeighbourVertexTem_ = SourceVRGraphManager->getVertex(IterateNeighboursTem__.first);
        Eigen::Vector2d Vertex2PositionTem_(NeighbourVertexTem_->state[0],
                                            NeighbourVertexTem_->state[1]);
        Eigen::Vector2d DistanceVectorMiddle_ = Vertex2PositionTem_ - Vertex1PositionTem_;
        bool IfCouldConnectTwoVertices = false;

        double VR2Tem_ = NeighbourVertexTem_->resolution;
        double DFEIX = abs(DistanceVectorMiddle_[0]); // Distance For Edge In X-axe
        double DFEIY = abs(DistanceVectorMiddle_[1]); // Distance For Edge In Y-axe
        if ((DFEIX <= (VR1Tem_ + VR2Tem_) / 2 + 0.02) &&
            (DFEIY <= (VR1Tem_ + VR2Tem_) / 2 + 0.02))
        {
            IfCouldConnectTwoVertices = true;
        }
        if (!IfCouldConnectTwoVertices)
        {
            continue;
        }
        if (NeighbourVertexTem_->resolution > VertexResolution)
        {
            double DivideTimesTem_ = (double)(NeighbourVertexTem_->resolution / VertexResolution);
            double GeneratedResolutionTem_ = NeighbourVertexTem_->resolution / DivideTimesTem_;
            double xMinTem_ = NeighbourVertexTem_->state[0] -
                              NeighbourVertexTem_->resolution / 2 +
                              GeneratedResolutionTem_ / 2;
            double yMinTem_ = NeighbourVertexTem_->state[1] -
                              NeighbourVertexTem_->resolution / 2 +
                              GeneratedResolutionTem_ / 2;
            for (int iRowTem_ = 0; iRowTem_ < DivideTimesTem_; iRowTem_++)
            {
                for (int iColTem_ = 0; iColTem_ < DivideTimesTem_; iColTem_++)
                {
                    Eigen::Vector3d VP3DGTem_(xMinTem_ + iRowTem_ * GeneratedResolutionTem_,
                                              yMinTem_ + iColTem_ * GeneratedResolutionTem_,
                                              NeighbourVertexTem_->state[2]);
                    VerteicesPosition3DGeneratedTem_.push_back(VP3DGTem_);
                }
            }
        }
        else
        {
            Eigen::Vector3d VP3DGTem_(NeighbourVertexTem_->state[0],
                                      NeighbourVertexTem_->state[1],
                                      NeighbourVertexTem_->state[2]);
            VerteicesPosition3DGeneratedTem_.push_back(VP3DGTem_);
        }
    }

    for (size_t iTem_ = 0; iTem_ < VerteicesPosition3DGeneratedTem_.size(); iTem_++)
    {
        int xIndex = -2;
        int yIndex = -2;
        if (VerteicesPosition3DGeneratedTem_[iTem_][0] >
            (VertexPosition[0] + VertexResolution / 2))
        {
            xIndex = 1;
        }
        else if (VerteicesPosition3DGeneratedTem_[iTem_][0] <
                 (VertexPosition[0] - VertexResolution / 2))
        {
            xIndex = -1;
        }
        else
        {
            xIndex = 0;
        }

        if (VerteicesPosition3DGeneratedTem_[iTem_][1] >
            (VertexPosition[1] + VertexResolution / 2))
        {
            yIndex = 1;
        }
        else if (VerteicesPosition3DGeneratedTem_[iTem_][1] <
                 (VertexPosition[1] - VertexResolution / 2))
        {
            yIndex = -1;
        }
        else
        {
            yIndex = 0;
        }

        switch (xIndex)
        {
        case -1:
            // back
            switch (yIndex)
            {
            case -1:
                // right back
                ExistRightBack_ = true;
                break;
            case 0:
                // center back
                ExistCenterBack_ = true;
                break;
            case 1:
                // left back
                ExistLeftBack_ = true;
                break;

            default:
                ROS_ERROR("[FrontierExtract_Info]: yIndex wrong when judge if vertex internal.");
                break;
            }
            break;
        case 0:
            // center
            switch (yIndex)
            {
            case -1:
                // right center
                ExistRightCenter_ = true;
                break;
            case 0:
                // center center
                ExistCenterCenter_ = true;
                break;
            case 1:
                // left center
                ExistLeftCenter_ = true;
                break;

            default:
                ROS_ERROR("[FrontierExtract_Info]: yIndex wrong when judge if vertex internal.");
                break;
            }
            break;
        case 1:
            // front
            switch (yIndex)
            {
            case -1:
                // right front
                ExistRightFront_ = true;
                break;
            case 0:
                // center front
                ExistCenterFront_ = true;
                break;
            case 1:
                // left front
                ExistLeftFront_ = true;
                break;

            default:
                ROS_ERROR("[FrontierExtract_Info]: yIndex wrong when judge if vertex internal.");
                break;
            }
            break;

        default:
            ROS_ERROR("[FrontierExtract_Info]: xIndex wrong when judge if vertex internal.");
            break;
        }
        //
    }
    if (ExistLeftFront_ &&
        ExistLeftCenter_ &&
        ExistLeftBack_ &&
        ExistCenterFront_ &&
        ExistCenterCenter_ &&
        ExistCenterBack_ &&
        ExistRightFront_ &&
        ExistRightCenter_ &&
        ExistRightBack_)
    {
        return true;
    }
    else
    {
        return false;
    }
    return true;
}

void FrontierExtract::UpdateFrontiersSet(std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                         double VRMapResolutionMax,
                                         double LimitBoxZ,
                                         double NeighboursCountThreshold,
                                         std::set<Eigen::Vector3d, setcomp> &FrontiersSet)
{
    std::vector<Eigen::Vector3d> FrontiersNeedToBeDeleted_;
    FrontiersNeedToBeDeleted_.clear();
    for (auto Iterator : FrontiersSet)
    {
        StateVec StateTem_(Iterator[0],
                           Iterator[1],
                           Iterator[2],
                           0);
        Vertex *NearestVertexPtr_;
        if (SourceVRGraphManager->getNearestVertex(&StateTem_,
                                                   &NearestVertexPtr_))
        {
            if (NearestVertexPtr_->is_obstacle)
            {
                FrontiersNeedToBeDeleted_.push_back(Iterator);
                continue;
            }
            Eigen::Vector2d Distance2DTem_(NearestVertexPtr_->state[0] - Iterator[0],
                                           NearestVertexPtr_->state[1] - Iterator[1]);
            if (Distance2DTem_.norm() > 0.05)
            {
                FrontiersNeedToBeDeleted_.push_back(Iterator);
                continue;
            }
        }
        else
        {
            ROS_ERROR("[FrontierExtract_INFO]: Can not find the nearest point when update frontiers set.");
            FrontiersNeedToBeDeleted_.push_back(Iterator);
            continue;
        }

        bool IsFrontierVertex_ = !IfVertexInternal(SourceVRGraphManager,
                                                   Iterator,
                                                   VRMapResolutionMax,
                                                   LimitBoxZ,
                                                   NeighboursCountThreshold);
        if (!IsFrontierVertex_)
        {
            FrontiersNeedToBeDeleted_.push_back(Iterator);
        }
    }
    if (FrontiersNeedToBeDeleted_.size() == 0)
    {
        return;
    }
    for (size_t iTemporary_ = 0; iTemporary_ < FrontiersNeedToBeDeleted_.size(); iTemporary_++)
    {
        FrontiersSet.erase(FrontiersNeedToBeDeleted_[iTemporary_]);
    }
}

void FrontierExtract::UpdateFrontiersSetByGraph(std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                                double VRMapResolutionMax,
                                                double VRMapResolutionMin,
                                                double LimitBoxZ,
                                                bool IfConstrainTheFrontierArea,
                                                Eigen::Vector3d CenterPosition,
                                                Eigen::Matrix<double, 3, 2> ConstrainsOfFrontiers,
                                                std::set<Eigen::Vector3d, setcomp> &FrontiersSet)
{
    std::vector<Eigen::Vector3d> FrontiersNeedToBeDeleted_;
    FrontiersNeedToBeDeleted_.clear();
    Eigen::Matrix<double, 3, 2> MinAndMaxValueVectorTem_;
    MinAndMaxValueVectorTem_(0, 0) = CenterPosition[0] - ConstrainsOfFrontiers(0, 0);
    MinAndMaxValueVectorTem_(0, 1) = CenterPosition[0] + ConstrainsOfFrontiers(0, 1);
    MinAndMaxValueVectorTem_(1, 0) = CenterPosition[1] - ConstrainsOfFrontiers(1, 0);
    MinAndMaxValueVectorTem_(1, 1) = CenterPosition[1] + ConstrainsOfFrontiers(1, 1);
    MinAndMaxValueVectorTem_(2, 0) = CenterPosition[2] - ConstrainsOfFrontiers(2, 0);
    MinAndMaxValueVectorTem_(2, 1) = CenterPosition[2] + ConstrainsOfFrontiers(2, 1);
    for (auto Iterator : FrontiersSet)
    {
        if (IfConstrainTheFrontierArea)
        {
            if ((Iterator[0] < MinAndMaxValueVectorTem_(0, 0)) ||
                (Iterator[0] > MinAndMaxValueVectorTem_(0, 1)) ||
                (Iterator[1] < MinAndMaxValueVectorTem_(1, 0)) ||
                (Iterator[1] > MinAndMaxValueVectorTem_(1, 1)) ||
                (Iterator[2] < MinAndMaxValueVectorTem_(2, 0)) ||
                (Iterator[2] > MinAndMaxValueVectorTem_(2, 1)))
            {
                continue;
            }
        }
        StateVec StateTem_(Iterator[0],
                           Iterator[1],
                           Iterator[2],
                           0);
        Vertex *NearestVertexPtr_;
        if (SourceVRGraphManager->getNearestVertex(&StateTem_,
                                                   &NearestVertexPtr_))
        {
            if (NearestVertexPtr_->is_obstacle)
            {
                FrontiersNeedToBeDeleted_.push_back(Iterator);
                continue;
            }
            Eigen::Vector2d Distance2DTem_(NearestVertexPtr_->state[0] - Iterator[0],
                                           NearestVertexPtr_->state[1] - Iterator[1]);
            if (Distance2DTem_.norm() > 0.01)
            {
                FrontiersNeedToBeDeleted_.push_back(Iterator);
                continue;
            }
            Iterator[2] = NearestVertexPtr_->state[2];
        }
        else
        {
            ROS_ERROR("[FrontierExtract_INFO]: Can not find the nearest point when update frontiers set.");
            FrontiersNeedToBeDeleted_.push_back(Iterator);
            continue;
        }

        // bool IsFrontierVertex_ = !IfVertexInternal(SourceVRGraphManager,
        //                                            Iterator,
        //                                            VRMapResolutionMax,
        //                                            LimitBoxZ,
        //                                            8);
        bool IsFrontierVertex_ = !IfVertexInternalByGraph(SourceVRGraphManager,
                                                          Iterator,
                                                          NearestVertexPtr_->resolution,
                                                          VRMapResolutionMax,
                                                          VRMapResolutionMin,
                                                          LimitBoxZ);
        if (!IsFrontierVertex_)
        {
            FrontiersNeedToBeDeleted_.push_back(Iterator);
        }
    }
    if (FrontiersNeedToBeDeleted_.size() == 0)
    {
        return;
    }
    for (size_t iTemporary_ = 0; iTemporary_ < FrontiersNeedToBeDeleted_.size(); iTemporary_++)
    {
        FrontiersSet.erase(FrontiersNeedToBeDeleted_[iTemporary_]);
    }
}

void FrontierExtract::UpdateFrontiersSetByElevationMap(grid_map::GridMap SourceElevationMap,
                                                       std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                                       double VRMapResolutionMax,
                                                       double LimitBoxZ,
                                                       double NeighboursCountThreshold,
                                                       double CellResolution,
                                                       std::string ElevationLayer,
                                                       std::string TraversabilityLayer,
                                                       std::string TraversabilitySupplementLayer,
                                                       std::set<Eigen::Vector3d, setcomp> &FrontiersSet)
{
    std::vector<Eigen::Vector3d> FrontiersNeedToBeDeleted_;
    FrontiersNeedToBeDeleted_.clear();
    for (auto Iterator : FrontiersSet)
    {
        Eigen::Vector2d Position2DTem_(Iterator[0], Iterator[1]);
        if (!SourceElevationMap.isInside(Position2DTem_))
        {
            continue;
        }
        StateVec StateTem_(Iterator[0],
                           Iterator[1],
                           Iterator[2],
                           0);
        Vertex *NearestVertexPtr_;
        if (SourceVRGraphManager->getNearestVertex(&StateTem_,
                                                   &NearestVertexPtr_))
        {
            if (NearestVertexPtr_->is_obstacle)
            {
                FrontiersNeedToBeDeleted_.push_back(Iterator);
                continue;
            }
            Eigen::Vector2d Distance2DTem_(NearestVertexPtr_->state[0] - Iterator[0],
                                           NearestVertexPtr_->state[1] - Iterator[1]);
            if (Distance2DTem_.norm() > 0.05)
            {
                FrontiersNeedToBeDeleted_.push_back(Iterator);
                continue;
            }
            // Generate Cells and check if all cells are internal.
        }
        else
        {
            ROS_ERROR("[FrontierExtract_INFO]: Can not find the nearest point when update frontiers set.");
            FrontiersNeedToBeDeleted_.push_back(Iterator);
            continue;
        }

        bool IsFrontierVertex_ = !IfVertexInternal(SourceVRGraphManager,
                                                   Iterator,
                                                   VRMapResolutionMax,
                                                   LimitBoxZ,
                                                   NeighboursCountThreshold);
        if (!IsFrontierVertex_)
        {
            FrontiersNeedToBeDeleted_.push_back(Iterator);
            continue;
        }

        std::vector<Eigen::Vector2d> CellsGeneratedTem_;
        CellsGeneratedTem_.clear();
        Eigen::Vector3d CenterPositionTem_(NearestVertexPtr_->state[0],
                                           NearestVertexPtr_->state[1],
                                           NearestVertexPtr_->state[2]);
        GenerateCellsByPositionAndResolution(CellsGeneratedTem_,
                                             CenterPositionTem_,
                                             NearestVertexPtr_->resolution,
                                             CellResolution);
        bool IfAreInternalPointsTem_ = false;
        IfAreInternalPointsTem_ = IfPointsInternal(CellsGeneratedTem_,
                                                   SourceElevationMap,
                                                   CellResolution,
                                                   ElevationLayer,
                                                   TraversabilityLayer,
                                                   TraversabilitySupplementLayer);
        if (IfAreInternalPointsTem_)
        {
            FrontiersNeedToBeDeleted_.push_back(Iterator);
            continue;
        }
    }
    if (FrontiersNeedToBeDeleted_.size() == 0)
    {
        return;
    }
    for (size_t iTemporary_ = 0; iTemporary_ < FrontiersNeedToBeDeleted_.size(); iTemporary_++)
    {
        FrontiersSet.erase(FrontiersNeedToBeDeleted_[iTemporary_]);
    }
}

void FrontierExtract::VisualizeVertices(std::set<Eigen::Vector3d, setcomp> &FrontiersSet,
                                        std::shared_ptr<ros::Publisher> TopicPublisher,
                                        string ns_,
                                        string Frame)
{

    if (FrontiersSet.size() == 0)
    {
        return;
    }
    if (TopicPublisher->getNumSubscribers() < 1)
    {
        return;
    }
    // int frontier_seq_middle_ = 0;
    std::string vertex_str_ = "FrontierVertex";
    visualization_msgs::Marker vertex_marker;
    // vertex_marker.header.stamp = ros::Time::now();
    // vertex_marker.header.seq = frontier_seq_middle_;
    // vertex_marker.header.frame_id = Frame.c_str();
    // vertex_marker.id = frontier_seq_middle_;
    // vertex_marker.ns = (ns_ + vertex_str_).c_str();
    // vertex_marker.action = visualization_msgs::Marker::ADD;
    // vertex_marker.type = visualization_msgs::Marker::CUBE;
    // vertex_marker.scale.x = 0.2;
    // vertex_marker.scale.y = 0.2;
    // vertex_marker.scale.z = 0.01;
    // vertex_marker.color.r = 255.0 / 255.0;
    // vertex_marker.color.g = 0.0 / 255.0;
    // vertex_marker.color.b = 0.0 / 255.0;
    // vertex_marker.color.a = 1.0;
    // vertex_marker.lifetime = ros::Duration(5.0);
    // vertex_marker.frame_locked = false;

    vertex_marker.header.stamp = ros::Time::now();
    vertex_marker.header.seq = 0;
    vertex_marker.header.frame_id = Frame.c_str();
    vertex_marker.id = 0;
    vertex_marker.ns = (ns_ + vertex_str_).c_str();
    vertex_marker.action = visualization_msgs::Marker::ADD;
    vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    vertex_marker.scale.x = 0.2;
    vertex_marker.scale.y = 0.2;
    vertex_marker.scale.z = 0.01;
    vertex_marker.color.r = 255.0 / 255.0;
    vertex_marker.color.g = 0.0 / 255.0;
    vertex_marker.color.b = 0.0 / 255.0;
    vertex_marker.color.a = 1.0;
    vertex_marker.lifetime = ros::Duration(0.0);
    vertex_marker.frame_locked = false;

    for (auto &IterateSet_ : FrontiersSet)
    {
        geometry_msgs::Point p1;
        p1.x = IterateSet_[0];
        p1.y = IterateSet_[1];
        p1.z = IterateSet_[2] + 0.25;
        vertex_marker.points.push_back(p1);
    }
    TopicPublisher->publish(vertex_marker);
}

void FrontierExtract::DistinguishFrontiers(std::set<Eigen::Vector3d, setcomp> &SourceFrontiersSet,
                                           std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                           std::shared_ptr<UF> UFTem,
                                           double VRMapResolutionMax,
                                           double VRMapResolutionMin,
                                           double LimitBoxZ,
                                           double LimitBoxZForConnectEdge,
                                           double ElevationMapResolution,
                                           std::unordered_map<int, std::vector<int>> &SubGraphs,
                                           std::unordered_map<int, double> &OutputFrontiers)
{
    UFTem.reset(new UF(0));
    for (auto FrontierTem_ : SourceFrontiersSet)
    {
        StateVec StateVector_(FrontierTem_[0],
                              FrontierTem_[1],
                              FrontierTem_[2],
                              0);
        Vertex *NearestVertexPtr_;
        int IdFrontier_ = 0;
        Eigen::Vector2d VertexPosition;

        if (SourceVRGraphManager->getNearestVertex(&StateVector_,
                                                   &NearestVertexPtr_))
        {
            if (NearestVertexPtr_->is_obstacle)
            {
                continue;
            }
            IdFrontier_ = NearestVertexPtr_->id;
            VertexPosition << NearestVertexPtr_->state[0],
                NearestVertexPtr_->state[1];
        }
        else
        {
            ROS_ERROR("[FrontierExtract_INFO]: Can not find the nearest point when distinguish frontiers.");
        }
        UFTem->addpoint(IdFrontier_);
    }

    for (auto IteratorUF : UFTem->parent)
    {
        int IdTem_ = IteratorUF.first;
        Vertex *VertexTemPtr_ = SourceVRGraphManager->getVertex(IdTem_);
        StateVec StateTem_ = VertexTemPtr_->state;
        Eigen::Vector2d VertexPositionTem_(StateTem_[0],
                                           StateTem_[1]);
        std::vector<Vertex *> nearest_vertices_MultiResolutionMap;
        nearest_vertices_MultiResolutionMap.clear();
        if (SourceVRGraphManager->getNearestVerticesInBox(&StateTem_,
                                                          VRMapResolutionMax + 0.5 * VRMapResolutionMin,
                                                          VRMapResolutionMax + 0.5 * VRMapResolutionMin,
                                                          LimitBoxZ,
                                                          &nearest_vertices_MultiResolutionMap))
        {
            for (size_t iForVertices = 0;
                 iForVertices < nearest_vertices_MultiResolutionMap.size();
                 iForVertices++)
            {
                if (nearest_vertices_MultiResolutionMap[iForVertices]->is_obstacle)
                {
                    continue;
                }
                // double distanceForEdge_ = 0.0;
                Eigen::Vector2d VertexOldPosition_(nearest_vertices_MultiResolutionMap[iForVertices]->state[0],
                                                   nearest_vertices_MultiResolutionMap[iForVertices]->state[1]);
                Eigen::Vector2d DistanceVectorMiddle_ = VertexOldPosition_ - VertexPositionTem_;
                // distanceForEdge_ = DistanceVectorMiddle_.norm();
                bool IfCouldConnectTwoVertices = false;
                double VNR_ = VertexTemPtr_->resolution;
                double VOR_ = nearest_vertices_MultiResolutionMap[iForVertices]->resolution;
                double DFEIX = abs(DistanceVectorMiddle_[0]); // Distance For Edge In X-axe
                double DFEIY = abs(DistanceVectorMiddle_[1]); // Distance For Edge In Y-axe
                if ((DFEIX <= (VNR_ + VOR_) / 2 + ElevationMapResolution) &&
                    (DFEIY <= (VNR_ + VOR_) / 2 + ElevationMapResolution) &&
                    (abs(StateTem_[2] - nearest_vertices_MultiResolutionMap[iForVertices]->state[2]) < LimitBoxZForConnectEdge))
                {
                    IfCouldConnectTwoVertices = true;
                }
                if (IfCouldConnectTwoVertices)
                {
                    int IdNearestTem_ = nearest_vertices_MultiResolutionMap[iForVertices]->id;
                    if (UFTem->IfPointExist(IdNearestTem_))
                    {
                        UFTem->merge(IdTem_, IdNearestTem_);
                    }
                }
            }
        }
    }
    SubGraphs.clear();
    OutputFrontiers.clear();
    for (auto IteratorUFTwiceTem : UFTem->parent)
    {
        int IdTwiceTem_ = IteratorUFTwiceTem.first;
        int KindNumTem_ = UFTem->find(IdTwiceTem_);
        if (SubGraphs.find(KindNumTem_) == SubGraphs.end())
        {
            // std::vector<int> IdVectorTem_;
            // IdVectorTem_.clear();
            // SubGraphs[KindNumTem_] = IdVectorTem_;
            // OutputFrontiers[KindNumTem_] = pow(SourceVRGraphManager->getVertex(IdTwiceTem_)->resolution,
            //                                    2);
            OutputFrontiers[KindNumTem_] = 2 * SourceVRGraphManager->getVertex(IdTwiceTem_)->resolution;
            SubGraphs[KindNumTem_].push_back(IdTwiceTem_);
        }
        else
        {
            // OutputFrontiers[KindNumTem_] = pow(SourceVRGraphManager->getVertex(IdTwiceTem_)->resolution,
            //                                    2) +
            //                                OutputFrontiers[KindNumTem_];
            OutputFrontiers[KindNumTem_] = 2 * SourceVRGraphManager->getVertex(IdTwiceTem_)->resolution +
                                           OutputFrontiers[KindNumTem_];
            SubGraphs[KindNumTem_].push_back(IdTwiceTem_);
        }
    }
}

void FrontierExtract::VisualizeSubGraphsOfFrontiers(std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                                    std::shared_ptr<ros::Publisher> SGOFTopicPublisher,
                                                    std::shared_ptr<ros::Publisher> TopicPublisher,
                                                    std::unordered_map<int, std::vector<int>> &SourceSubGraphs,
                                                    std::unordered_map<int, double> InputFrontiersAttribute,
                                                    std::unordered_map<int, double> &OutputFrontiersAttribute,
                                                    std::vector<Eigen::Vector4d> &OutputCenters,
                                                    string Frame)
{
    OutputFrontiersAttribute.clear();
    OutputCenters.clear();
    visualization_msgs::MarkerArray marker_array_submap;
    marker_array_submap.markers.clear();
    visualization_msgs::MarkerArray marker_array_point_pre_exploration;
    marker_array_point_pre_exploration.markers.clear();
    int IdMarkerSub_ = 0;
    int marker_id = 0;
    int iForColor = 0;
    for (auto IterateUM_ : SourceSubGraphs)
    {
        int IdVisualize_ = IterateUM_.first;
        Vertex *VertexPtr = SourceVRGraphManager->getVertex(IdVisualize_);

        double rgb = (double)(iForColor) / (double)(SourceSubGraphs.size());
        string ns_middle = std::to_string(rgb);

        for (std::vector<int>::iterator iterator_submap_vector_ = IterateUM_.second.begin();
             iterator_submap_vector_ != IterateUM_.second.end();
             ++iterator_submap_vector_)
        {
            Vertex *vertex_submap_used_ = SourceVRGraphManager->getVertex(*iterator_submap_vector_);
            geometry_msgs::Point p_submap_;

            visualization_msgs::Marker vertex_submap_marker;
            vertex_submap_marker.header.stamp = ros::Time::now();
            vertex_submap_marker.header.seq = 0;
            vertex_submap_marker.header.frame_id = Frame.c_str();
            vertex_submap_marker.id = IdMarkerSub_;
            vertex_submap_marker.ns = ns_middle.c_str();
            vertex_submap_marker.action = visualization_msgs::Marker::ADD;
            vertex_submap_marker.type = visualization_msgs::Marker::CUBE;
            vertex_submap_marker.scale.x = vertex_submap_used_->resolution;
            vertex_submap_marker.scale.y = vertex_submap_used_->resolution;
            vertex_submap_marker.scale.z = 0.01;
            vertex_submap_marker.color.r = 1 - rgb;
            vertex_submap_marker.color.g = 1 - rgb;
            vertex_submap_marker.color.b = rgb;
            vertex_submap_marker.color.a = 1.0;
            vertex_submap_marker.lifetime = ros::Duration(0.0);
            vertex_submap_marker.frame_locked = false;

            p_submap_.x = vertex_submap_used_->state[0];
            p_submap_.y = vertex_submap_used_->state[1];
            p_submap_.z = vertex_submap_used_->state[2];
            vertex_submap_marker.pose.position = p_submap_;
            marker_array_submap.markers.push_back(vertex_submap_marker);
            IdMarkerSub_++;
        }
        iForColor++;

        bool IfGetCenterVertexSucceed_ = false;
        int IdCenter_ = 0;
        IfGetCenterVertexSucceed_ = get_central_point(IterateUM_.second,
                                                      SourceVRGraphManager,
                                                      IdCenter_);
        if (!IfGetCenterVertexSucceed_)
        {
            ROS_ERROR("[FrontierExtract_Info]: Can not get center vertex and set a random vertex in the vector.");
            OutputFrontiersAttribute[IdVisualize_] = InputFrontiersAttribute[IdVisualize_];
        }
        else
        {
            VertexPtr = SourceVRGraphManager->getVertex(IdCenter_);
            OutputFrontiersAttribute[IdCenter_] = InputFrontiersAttribute[IdVisualize_];
        }
        Eigen::Vector4d CeterTem_(VertexPtr->state[0],
                                  VertexPtr->state[1],
                                  VertexPtr->state[2],
                                  InputFrontiersAttribute[IdVisualize_]);
        OutputCenters.push_back(CeterTem_);

        visualization_msgs::Marker pre_exploration_marker;
        pre_exploration_marker.header.stamp = ros::Time::now();
        pre_exploration_marker.header.seq = 0;
        pre_exploration_marker.header.frame_id = Frame.c_str();
        pre_exploration_marker.ns = "point_pre_exploration";
        pre_exploration_marker.action = visualization_msgs::Marker::ADD;
        pre_exploration_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        pre_exploration_marker.scale.z = 0.5; // text height
        pre_exploration_marker.color.r = 18.0 / 255.0;
        pre_exploration_marker.color.g = 15.0 / 255.0;
        pre_exploration_marker.color.b = 196.0 / 255.0;
        pre_exploration_marker.color.a = 1.0;
        pre_exploration_marker.lifetime = ros::Duration(0.0);
        pre_exploration_marker.frame_locked = false;
        pre_exploration_marker.pose.position.x = VertexPtr->state[0];
        pre_exploration_marker.pose.position.y = VertexPtr->state[1];
        pre_exploration_marker.pose.position.z = VertexPtr->state[2] + 0.1;
        std::string text_display = std::to_string(IdVisualize_);

        pre_exploration_marker.text = text_display;
        pre_exploration_marker.id = marker_id;
        marker_array_point_pre_exploration.markers.push_back(pre_exploration_marker);
        marker_id++;
    }
    if (SGOFTopicPublisher->getNumSubscribers() > 0)
    {
        SGOFTopicPublisher->publish(marker_array_submap);
    }
    if (TopicPublisher->getNumSubscribers() > 0)
    {
        TopicPublisher->publish(marker_array_point_pre_exploration);
    }
}

bool FrontierExtract::get_central_point(std::vector<int> IdVector,
                                        std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                        int &IdCenterOutput)
{
    Eigen::Vector3d point_sum_(0, 0, 0);
    // std::shared_ptr<KD_TREE<PointType>> IKdTreePtrTem_;
    // IKdTreePtrTem_.reset(new KD_TREE<PointType>(0.3, 0.6, 0.16));
    // PointVector PointVectorTem_;
    // PointVectorTem_.clear();

    for (std::vector<int>::iterator iterator_ids_vector_ = IdVector.begin();
         iterator_ids_vector_ != IdVector.end();
         ++iterator_ids_vector_)
    {
        Vertex *middle = SourceVRGraphManager->getVertex(*iterator_ids_vector_);
        Eigen::Vector3d point_middle_(middle->state[0], middle->state[1], middle->state[2]);
        point_sum_ = point_sum_ + point_middle_;
        // PointType PointTem_;
        // PointTem_.x = middle->state[0];
        // PointTem_.y = middle->state[1];
        // PointTem_.z = middle->state[2];
        // PointTem_.id = middle->id;
        // PointVectorTem_.push_back(PointTem_);
    }
    point_sum_ = (double)(1.00) / (double)(IdVector.size()) * point_sum_;

    double DistanceMin = std::numeric_limits<double>::max();
    int OutputIdTem_ = 0;

    for (std::vector<int>::iterator IteratorTem2_ = IdVector.begin();
         IteratorTem2_ != IdVector.end();
         ++IteratorTem2_)
    {
        Vertex *middle = SourceVRGraphManager->getVertex(*IteratorTem2_);
        Eigen::Vector3d Point3DTem_(middle->state[0], middle->state[1], middle->state[2]);
        Eigen::Vector3d DistanceVectorTem_ = Point3DTem_ - point_sum_;
        double DistanceTem_ = DistanceVectorTem_.norm();
        if (DistanceTem_ < DistanceMin)
        {
            DistanceMin = DistanceTem_;
            OutputIdTem_ = middle->id;
        }
    }
    IdCenterOutput = OutputIdTem_;
    // IKdTreePtrTem_->Build(PointVectorTem_);

    // PointType point_target;
    // point_target.x = point_sum_[0];
    // point_target.y = point_sum_[1];
    // point_target.z = point_sum_[2];
    // PointVector SearchResultTem_;
    // vector<float> PointDistTem_;

    // IKdTreePtrTem_->Nearest_Search(point_target, 1, SearchResultTem_, PointDistTem_, 3.0);
    // if (SearchResultTem_.size() <= 0)
    // {
    //     ROS_INFO("[FrontierExtrack_Info]: Could not find the nearest vertex and extend to search in range 10m.");
    //     SearchResultTem_.clear();
    //     PointDistTem_.clear();
    //     IKdTreePtrTem_->Nearest_Search(point_target, 1, SearchResultTem_, PointDistTem_, 10.0);
    // }
    // if (SearchResultTem_.size() <= 0)
    // {
    //     ROS_INFO("[FrontierExtrack_Info]: Could not find the nearest vertex in range 10m.");
    //     return false;
    // }

    // PointType point_out;
    // point_out = SearchResultTem_[0];
    // IdCenterOutput = point_out.id;

    return true;

    // StateVec state_nearest(point_sum_[0], point_sum_[1], point_sum_[2], 0);
    // Vertex *nearest_vertices;
    // if (SourceVRGraphManager->getNearestVertex(&state_nearest, &nearest_vertices))
    // {
    //     if (nearest_vertices->id == 0)
    //     {
    //         std::vector<Vertex *> nearest_vertices_middle;
    //         SourceVRGraphManager->getNearestVertices(&state_nearest, 10, &nearest_vertices_middle);
    //         if (nearest_vertices_middle.size() > 0)
    //         {
    //             for (int index_temp = 0; index_temp < nearest_vertices_middle.size(); index_temp++)
    //             {
    //                 if (nearest_vertices_middle[index_temp]->id == 0)
    //                 {
    //                     continue;
    //                 }
    //                 else
    //                 {
    //                     IdCenterOutput = nearest_vertices_middle[index_temp]->id;
    //                     return true;
    //                 }
    //             }
    //         }
    //         else
    //         {
    //             ROS_ERROR("[FrontierExtrack_Info]: Why the central point of frontier sub graph could not be find in range of 10.");
    //         }
    //     }
    //     else
    //     {
    //         IdCenterOutput = nearest_vertices->id;
    //         return true;
    //     }
    //     // return nearest_vertices->id;
    // }
    // else
    // {
    //     ROS_INFO("[FrontierExtrack_Info]: Could not find the nearest vertex and extend to search in range 10m.");
    //     std::vector<Vertex *> nearest_vertices_middle2;
    //     SourceVRGraphManager->getNearestVertices(&state_nearest, 10, &nearest_vertices_middle2);
    //     if (nearest_vertices_middle2.size() > 0)
    //     {
    //         for (int index_temp = 0; index_temp < nearest_vertices_middle2.size(); index_temp++)
    //         {
    //             if (nearest_vertices_middle2[index_temp]->id == 0)
    //             {
    //                 continue;
    //             }
    //             else
    //             {
    //                 IdCenterOutput = nearest_vertices_middle2[index_temp]->id;
    //                 return true;
    //             }
    //         }
    //     }
    //     else
    //     {
    //         ROS_ERROR("WHAT THE HELL! Why the central point of frontier sub graph could not be find in range of 10.");
    //     }
    // }
    // ROS_ERROR("Can not find the central point and the point id is set to 0.");
    // return false;
}

void FrontierExtract::GenerateCellsByPositionAndResolution(std::vector<Eigen::Vector2d> &CellsOutPut,
                                                           Eigen::Vector3d CenterPosition,
                                                           double resolution,
                                                           double CellResolution)
{

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
        row_strat = (start_x) / CellResolution;
    }
    else
    {
        row_strat = (start_x) / CellResolution;
    }

    if (end_x < 0.0)
    {
        row_end = (end_x) / CellResolution;
    }
    else
    {
        row_end = (end_x) / CellResolution;
    }
    ////////////
    if (start_y < 0.0)
    {
        col_start = (start_y) / CellResolution;
    }
    else
    {
        col_start = (start_y) / CellResolution;
    }

    if (end_y < 0.0)
    {
        col_end = (end_y) / CellResolution;
    }
    else
    {
        col_end = (end_y) / CellResolution;
    }

    for (int iRowMiddle_ = row_strat; iRowMiddle_ <= row_end; iRowMiddle_++)
    {
        for (int iColMiddle_ = col_start; iColMiddle_ <= col_end; iColMiddle_++)
        {
            Eigen::Vector2d CellMiddle_;
            CellMiddle_ << ((double)(iRowMiddle_)-0.5) * CellResolution,
                ((double)(iColMiddle_)-0.5) * CellResolution;
            if (CellMiddle_[0] < 0)
            {
                CellMiddle_[0] = CellMiddle_[0] + CellResolution;
            }
            if (CellMiddle_[1] < 0)
            {
                CellMiddle_[1] = CellMiddle_[1] + CellResolution;
            }

            if ((CellMiddle_[0] < start_x) ||
                (CellMiddle_[0] > end_x) ||
                (CellMiddle_[1] < start_y) ||
                (CellMiddle_[1] > end_y))
            {
                continue;
            }
            CellsOutPut.push_back(CellMiddle_);
        }
    }
}

bool FrontierExtract::IfPointsInternal(std::vector<Eigen::Vector2d> Points,
                                       grid_map::GridMap SourceElevationMap,
                                       double ElevationMapResolution,
                                       std::string ElevationLayer,
                                       std::string TraversabilityLayer,
                                       std::string TraversabilitySupplementLayer)
{
    int FrontierCountTem_ = 0;
    for (size_t i = 0; i < Points.size(); i++)
    {
        bool IsFrontierTem_;
        try
        {
            IsFrontierTem_ = !IfPointInternal(Points[i][0],
                                              Points[i][1],
                                              SourceElevationMap,
                                              ElevationMapResolution,
                                              ElevationLayer,
                                              TraversabilityLayer,
                                              TraversabilitySupplementLayer);
        }
        catch (const std::exception &e)
        {
            // std::cerr << e.what() << '\n';
            ROS_ERROR("%s", e.what());
            ROS_ERROR("[FrontierExtrack_Info]: There is something wrong when check if points is internal.");
            IsFrontierTem_ = true;
        }
        if (IsFrontierTem_)
        {
            FrontierCountTem_++;
        }
    }
    if (FrontierCountTem_ > 0)
    {
        return false;
    }
    else
    {
        return true;
    }
    return true;
}

void FrontierExtract::FilterSubGraphs(std::unordered_map<int, std::vector<int>> &TargetSubGraphs,
                                      int MinSize)
{
    if (TargetSubGraphs.size() < 1)
    {
        return;
    }
    std::unordered_map<int, std::vector<int>> TargetSubGraphCopyTem_ = TargetSubGraphs;
    for (auto SubGraphsFilterIteratorTem_ : TargetSubGraphCopyTem_)
    {
        if (SubGraphsFilterIteratorTem_.second.size() < MinSize)
        {
            TargetSubGraphs.erase(SubGraphsFilterIteratorTem_.first);
        }
    }
}
