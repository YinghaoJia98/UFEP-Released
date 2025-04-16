#include <vrmapping/PathPlanner.h>
PathPlanner::PathPlanner()
{
  PathRBVPosition_.clear();
  PathRBVId_.clear();
  IdAndPrevious_.clear();
  IdAndDistance_.clear();
  NotReachableFrontierIdsInSamplerGraph_.clear();
  NotReachableTargetPositions3D_.clear();
}

PathPlanner::~PathPlanner()
{
}

bool PathPlanner::PlanPathByDijkstraMethod(std::shared_ptr<VRGraphManager> SourceVRGraphManager, int TargetId,
                                           std::vector<int>& PathIdOutput, string WorldFrame, string RobotFrame)
{
  int RobotId_ = 0;
  tf::TransformListener world_base_listener;
  tf::StampedTransform world_base_transform;
  try
  {
    world_base_listener.waitForTransform(WorldFrame.c_str(), RobotFrame.c_str(), ros::Time(0), ros::Duration(1.0));
    world_base_listener.lookupTransform(WorldFrame.c_str(), RobotFrame.c_str(), ros::Time(0), world_base_transform);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    // ros::Duration(1.0).sleep();
  }
  StateVec robot_state_(world_base_transform.getOrigin().x(), world_base_transform.getOrigin().y(),
                        world_base_transform.getOrigin().z() - RobotHeight_, 0);

  Vertex* robot_nearest_vertices;
  if (SourceVRGraphManager->getNearestVertex(&robot_state_, &robot_nearest_vertices))
  {
    RobotId_ = robot_nearest_vertices->id;
  }
  else
  {
    ROS_ERROR("[PathPlanner_Info]: Could not find the global id of robot.");
    return false;
  }
  PathIdOutput.clear();
  PathIdOutput = SourceVRGraphManager->Graph_->dijkstra(RobotId_, TargetId);
  if (PathIdOutput.size() == 0)
  {
    ROS_ERROR("[PathPlanner_Info]: Path size represented by id is 0.");
    return false;
  }
  return true;
}

bool PathPlanner::PlanPathByDijkstraMethod(std::shared_ptr<VRGraphManager> SourceVRGraphManager, int TargetId,
                                           std::vector<int>& PathIdOutput, string WorldFrame, string RobotFrame,
                                           VRGraphManager::GraphType UndirectedMap)
{
  int RobotId_ = 0;
  tf::TransformListener world_base_listener;
  tf::StampedTransform world_base_transform;
  try
  {
    world_base_listener.waitForTransform(WorldFrame.c_str(), RobotFrame.c_str(), ros::Time(0), ros::Duration(1.0));
    world_base_listener.lookupTransform(WorldFrame.c_str(), RobotFrame.c_str(), ros::Time(0), world_base_transform);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    // ros::Duration(1.0).sleep();
  }
  StateVec robot_state_(world_base_transform.getOrigin().x(), world_base_transform.getOrigin().y(),
                        world_base_transform.getOrigin().z() - RobotHeight_, 0);

  std::vector<Vertex*> RobotNearestVerticesSelectedTem_;
  RobotNearestVerticesSelectedTem_.clear();
  SourceVRGraphManager->getNearestVertices(&robot_state_, 1.0, &RobotNearestVerticesSelectedTem_);
  Eigen::Vector2d RobotPosition2DTem_(robot_state_[0], robot_state_[1]);
  for (size_t i = 0; i < RobotNearestVerticesSelectedTem_.size(); i++)
  {
    Vertex* NearestVertexTem_ = RobotNearestVerticesSelectedTem_[i];
    if (abs(NearestVertexTem_->state[2] - robot_state_[2]) > RobotHeight_)
    {
      continue;
    }
    Eigen::Vector2d VertexPosition2DTem(NearestVertexTem_->state[0], NearestVertexTem_->state[1]);
    Eigen::Vector2d Distance2DTem_ = VertexPosition2DTem - RobotPosition2DTem_;
    if (Distance2DTem_.norm() < NearestVertexTem_->resolution)
    {
      RobotId_ = NearestVertexTem_->id;
      break;
    }
  }

  if (RobotId_ == -1)
  {
    ROS_ERROR("[PathPlanner_Info]: Could not find the global id of robot.");
    return false;
  }

  // Vertex *robot_nearest_vertices;
  // if (SourceVRGraphManager->getNearestVertex(&robot_state_, &robot_nearest_vertices))
  // {
  //     RobotId_ = robot_nearest_vertices->id;
  // }
  // else
  // {
  //     ROS_ERROR("[PathPlanner_Info]: Could not find the global id of robot.");
  //     return false;
  // }
  PathIdOutput.clear();
  PathIdOutput = UndirectedMap.dijkstra(RobotId_, TargetId);
  if (PathIdOutput.size() == 0)
  {
    ROS_ERROR("[PathPlanner_Info]: Path size represented by id is 0.");
    return false;
  }
  return true;
}

bool PathPlanner::PlanPathByDijkstraMethod(std::shared_ptr<VRGraphManager> SourceVRGraphManager, int TargetId,
                                           std::vector<Eigen::Vector3d>& PathPositionOutput, string WorldFrame,
                                           string RobotFrame)
{
  PathPositionOutput.clear();
  std::vector<int> PathIdOutputForTem_;
  PathIdOutputForTem_.clear();
  bool IfSucceedPlanWithId_ = false;
  IfSucceedPlanWithId_ =
      PlanPathByDijkstraMethod(SourceVRGraphManager, TargetId, PathIdOutputForTem_, WorldFrame, RobotFrame);
  if (!IfSucceedPlanWithId_)
  {
    return false;
  }
  for (size_t iTem_ = 0; iTem_ < PathIdOutputForTem_.size(); iTem_++)
  {
    Vertex* VertexPtrTem_ = SourceVRGraphManager->getVertex(PathIdOutputForTem_[iTem_]);
    if (VertexPtrTem_ == NULL)
    {
      ROS_ERROR("[PathPlanner_Info]: Can not get the vertex from GVRGraphManager.");
      return false;
    }
    Eigen::Vector3d PositionTem_(VertexPtrTem_->state[0], VertexPtrTem_->state[1], VertexPtrTem_->state[2]);
    PathPositionOutput.push_back(PositionTem_);
  }
  if (PathPositionOutput.size() == 0)
  {
    ROS_ERROR("[PathPlanner_Info]: Path size represented by Eigen::Vector3d is 0.");
    return false;
  }
  return true;
}

bool PathPlanner::PlanPathByDijkstraMethod(std::shared_ptr<VRGraphManager> SourceVRGraphManager, int TargetId,
                                           std::vector<Eigen::Vector3d>& PathPositionOutput, string WorldFrame,
                                           string RobotFrame, VRGraphManager::GraphType UndirectedMap)
{
  PathPositionOutput.clear();
  std::vector<int> PathIdOutputForTem_;
  PathIdOutputForTem_.clear();
  bool IfSucceedPlanWithId_ = false;
  IfSucceedPlanWithId_ = PlanPathByDijkstraMethod(SourceVRGraphManager, TargetId, PathIdOutputForTem_, WorldFrame,
                                                  RobotFrame, UndirectedMap);
  if (!IfSucceedPlanWithId_)
  {
    return false;
  }
  for (size_t iTem_ = 0; iTem_ < PathIdOutputForTem_.size(); iTem_++)
  {
    Vertex* VertexPtrTem_ = SourceVRGraphManager->getVertex(PathIdOutputForTem_[iTem_]);
    if (VertexPtrTem_ == NULL)
    {
      ROS_ERROR("[PathPlanner_Info]: Can not get the vertex from GVRGraphManager.");
      return false;
    }
    Eigen::Vector3d PositionTem_(VertexPtrTem_->state[0], VertexPtrTem_->state[1], VertexPtrTem_->state[2]);
    PathPositionOutput.push_back(PositionTem_);
  }
  if (PathPositionOutput.size() == 0)
  {
    ROS_ERROR("[PathPlanner_Info]: Path size represented by Eigen::Vector3d is 0.");
    return false;
  }
  return true;
}

bool PathPlanner::DijstraProcess(std::shared_ptr<VRGraphManager> SourceVRGraphManager, string WorldFrame,
                                 string RobotFrame, std::unordered_map<int, int>& IdAndPrevious,
                                 std::unordered_map<int, double>& IdAndDistance)
{
  int RobotId_ = 0;
  tf::TransformListener world_base_listener;
  tf::StampedTransform world_base_transform;
  try
  {
    world_base_listener.waitForTransform(WorldFrame.c_str(), RobotFrame.c_str(), ros::Time(0), ros::Duration(1.0));
    world_base_listener.lookupTransform(WorldFrame.c_str(), RobotFrame.c_str(), ros::Time(0), world_base_transform);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    // ros::Duration(1.0).sleep();
  }
  StateVec robot_state_(world_base_transform.getOrigin().x(), world_base_transform.getOrigin().y(),
                        world_base_transform.getOrigin().z() - RobotHeight_, 0);

  Vertex* robot_nearest_vertices;
  if (SourceVRGraphManager->getNearestVertex(&robot_state_, &robot_nearest_vertices))
  {
    RobotId_ = robot_nearest_vertices->id;
  }
  else
  {
    ROS_ERROR("[PathPlanner_Info]: Could not find the global id of robot in DijstraProcess function.");
    return false;
  }
  SourceVRGraphManager->Graph_->DijkstraProcess(RobotId_, IdAndPrevious, IdAndDistance);
  if (IdAndPrevious.size() == 0)
  {
    ROS_ERROR("[PathPlanner_Info]: IdAndPrevious size is 0.");
    return false;
  }
  if (IdAndDistance.size() == 0)
  {
    ROS_ERROR("[PathPlanner_Info]: IdAndDistance size is 0.");
    return false;
  }
  return true;
}

bool PathPlanner::DijstraProcess(std::shared_ptr<VRGraphManager> SourceVRGraphManager, string WorldFrame,
                                 string RobotFrame, std::unordered_map<int, int>& IdAndPrevious,
                                 std::unordered_map<int, double>& IdAndDistance,
                                 VRGraphManager::GraphType UndirectedMap)
{
  int RobotId_ = 0;
  tf::TransformListener world_base_listener;
  tf::StampedTransform world_base_transform;
  try
  {
    world_base_listener.waitForTransform(WorldFrame.c_str(), RobotFrame.c_str(), ros::Time(0), ros::Duration(1.0));
    world_base_listener.lookupTransform(WorldFrame.c_str(), RobotFrame.c_str(), ros::Time(0), world_base_transform);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    // ros::Duration(1.0).sleep();
  }
  StateVec robot_state_(world_base_transform.getOrigin().x(), world_base_transform.getOrigin().y(),
                        world_base_transform.getOrigin().z() - RobotHeight_, 0);

  Vertex* robot_nearest_vertices;
  if (SourceVRGraphManager->getNearestVertex(&robot_state_, &robot_nearest_vertices))
  {
    RobotId_ = robot_nearest_vertices->id;
  }
  else
  {
    ROS_ERROR("[PathPlanner_Info]: Could not find the global id of robot in DijstraProcess function.");
    return false;
  }
  UndirectedMap.DijkstraProcess(RobotId_, IdAndPrevious, IdAndDistance);
  if (IdAndPrevious.size() == 0)
  {
    ROS_ERROR("[PathPlanner_Info]: IdAndPrevious size is 0.");
    return false;
  }
  if (IdAndDistance.size() == 0)
  {
    ROS_ERROR("[PathPlanner_Info]: IdAndDistance size is 0.");
    return false;
  }
  return true;
}

bool PathPlanner::GetDijkstraPathFromPrevious(std::shared_ptr<VRGraphManager> SourceVRGraphManager, int goal,
                                              std::unordered_map<int, int> IdAndPrevious,
                                              std::vector<int>& PathIdOutput)
{
  PathIdOutput.clear();
  SourceVRGraphManager->Graph_->GetDijkstraPathFromPrevious(goal, IdAndPrevious, PathIdOutput);
  if (PathIdOutput.size() == 0)
  {
    ROS_ERROR("[PathPlanner_Info]: PathIdOutput size is 0.");
    return false;
  }
  return true;
}

bool PathPlanner::GetDijkstraPathFromPrevious(std::shared_ptr<VRGraphManager> SourceVRGraphManager, int goal,
                                              std::unordered_map<int, int> IdAndPrevious,
                                              std::vector<Eigen::Vector3d>& PathPositionOutput)
{
  PathPositionOutput.clear();
  std::vector<int> PathIdTem_;
  PathIdTem_.clear();
  SourceVRGraphManager->Graph_->GetDijkstraPathFromPrevious(goal, IdAndPrevious, PathIdTem_);
  if (PathIdTem_.size() == 0)
  {
    ROS_ERROR("[PathPlanner_Info]: PathIdTem_ size is 0.");
    return false;
  }

  for (size_t iTem_ = 0; iTem_ < PathIdTem_.size(); iTem_++)
  {
    Vertex* VertexPtrTem_ = SourceVRGraphManager->getVertex(PathIdTem_[iTem_]);
    if (VertexPtrTem_ == NULL)
    {
      ROS_ERROR(
          "[PathPlanner_Info]: Can not get the vertex from GVRGraphManager in GetDijkstraPathFromPrevious function.");
      return false;
    }
    Eigen::Vector3d PositionTem_(VertexPtrTem_->state[0], VertexPtrTem_->state[1], VertexPtrTem_->state[2]);
    PathPositionOutput.push_back(PositionTem_);
  }
  if (PathPositionOutput.size() == 0)
  {
    ROS_ERROR(
        "[PathPlanner_Info]: Path size represented by Eigen::Vector3d is 0 in GetDijkstraPathFromPrevious function.");
    return false;
  }
  return true;
}

void PathPlanner::VisualizePath(std::vector<Eigen::Vector3d> PathVisualized,
                                std::shared_ptr<ros::Publisher> TopicPublisher, string ns_, string Frame)
{
  if (PathVisualized.size() == 0)
  {
    return;
  }

  visualization_msgs::MarkerArray Path_marker_array;

  std::string PathEdge_str_ = "PathEdge";
  visualization_msgs::Marker Path_edge_marker;
  Path_edge_marker.header.stamp = ros::Time::now();
  Path_edge_marker.header.seq = 0;
  Path_edge_marker.header.frame_id = Frame.c_str();
  Path_edge_marker.id = 0;
  Path_edge_marker.ns = (ns_ + PathEdge_str_).c_str();
  Path_edge_marker.action = visualization_msgs::Marker::ADD;
  Path_edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  Path_edge_marker.scale.x = 0.25;
  Path_edge_marker.color.r = 0.957;
  Path_edge_marker.color.g = 66.0 / 255.0;
  Path_edge_marker.color.b = 226.0 / 255.0;
  Path_edge_marker.color.a = 1.0;
  Path_edge_marker.lifetime = ros::Duration(0.0);
  Path_edge_marker.frame_locked = false;

  std::string PathPoint_str_ = "PathVertex";
  visualization_msgs::Marker Path_vertex_marker;
  Path_vertex_marker.header.stamp = ros::Time::now();
  Path_vertex_marker.header.seq = 0;
  Path_vertex_marker.header.frame_id = Frame.c_str();
  Path_vertex_marker.id = 0;
  Path_vertex_marker.ns = (ns_ + PathPoint_str_).c_str();
  Path_vertex_marker.action = visualization_msgs::Marker::ADD;
  Path_vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  Path_vertex_marker.scale.x = 0.25;
  Path_vertex_marker.scale.y = 0.25;
  Path_vertex_marker.scale.z = 0.25;
  Path_vertex_marker.color.r = 200.0 / 255.0;
  Path_vertex_marker.color.g = 100.0 / 255.0;
  Path_vertex_marker.color.b = 0.0;
  Path_vertex_marker.color.a = 1.0;
  Path_vertex_marker.lifetime = ros::Duration(0.0);
  Path_vertex_marker.frame_locked = false;

  for (size_t i = 0; i < (PathVisualized.size() - 1); ++i)
  {
    geometry_msgs::Point p1;
    p1.x = PathVisualized[i].x();
    p1.y = PathVisualized[i].y();
    p1.z = PathVisualized[i].z() + RobotHeight_;
    geometry_msgs::Point p2;
    p2.x = PathVisualized[i + 1].x();
    p2.y = PathVisualized[i + 1].y();
    p2.z = PathVisualized[i + 1].z() + RobotHeight_;
    Path_edge_marker.points.push_back(p1);
    Path_edge_marker.points.push_back(p2);
    Path_vertex_marker.points.push_back(p1);
    Eigen::Vector3d path_point_(PathVisualized[i].x(), PathVisualized[i].y(), PathVisualized[i].z());
    if (i == PathVisualized.size() - 2)
    {
      Path_vertex_marker.points.push_back(p2);
    }
  }
  Path_marker_array.markers.push_back(Path_edge_marker);
  Path_marker_array.markers.push_back(Path_vertex_marker);
  TopicPublisher->publish(Path_marker_array);
}
bool PathPlanner::TargetPointSelect(std::unordered_map<int, double> IdAndGain, int& TargetId)
{
  if (IdAndGain.size() == 0)
  {
    ROS_ERROR("[PathPlanner_Info]: The id vector where selecting target from is empty.");
    return false;
  }
  int TargetIdTem_ = 0;
  double GainMaxTem_ = 0.0;
  for (auto& IteratorTem_ : IdAndGain)
  {
    if (IteratorTem_.second > GainMaxTem_)
    {
      GainMaxTem_ = IteratorTem_.second;
      TargetIdTem_ = IteratorTem_.first;
    }
  }
  TargetId = TargetIdTem_;
  return true;
}

bool PathPlanner::TargetPointSelectByDistance(std::unordered_map<int, double> IdsPool,
                                              std::unordered_map<int, double> IdAndDistance, int& TargetId,
                                              std::set<int> NotReachableIds)
{
  if (IdsPool.size() == 0)
  {
    ROS_ERROR("[PathPlanner_Info]: The id vector where selecting target from is empty.");
    return false;
  }
  int TargetIdTem_ = -1;
  double DIstanceMinTem_ = 10000000.0;
  for (auto& IteratorTem_ : IdsPool)
  {
    int IterateIdTem_ = IteratorTem_.first;
    if (NotReachableIds.find(IterateIdTem_) != NotReachableIds.end())
    {
      continue;
    }
    double DistanceTem_ = IdAndDistance[IterateIdTem_];
    if (DistanceTem_ < DIstanceMinTem_)
    {
      DIstanceMinTem_ = DistanceTem_;
      TargetIdTem_ = IteratorTem_.first;
    }
  }
  if (TargetIdTem_ < 0)
  {
    return false;
  }
  TargetId = TargetIdTem_;
  return true;
}

bool PathPlanner::TargetPointSelectByEuclideanDistance(std::vector<Eigen::Vector4d> SourceCenters,
                                                       std::string WorldFrame, std::string BodyFrame,
                                                       bool IfContrainTheExplorationArea,
                                                       Eigen::Matrix<double, 3, 2> ConstrainsOfTheExplorationArea,
                                                       std::unordered_map<VOXEL_LOC, int> NotReachableTargetPositions3D,
                                                       Eigen::Vector3d& OutpuTargetPosition)
{
  tf::TransformListener world_base_listener;
  tf::StampedTransform world_base_transform;
  try
  {
    world_base_listener.waitForTransform(WorldFrame.c_str(), BodyFrame.c_str(), ros::Time(0), ros::Duration(1.0));
    world_base_listener.lookupTransform(WorldFrame.c_str(), BodyFrame.c_str(), ros::Time(0), world_base_transform);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    // ros::Duration(1.0).sleep();
  }
  Eigen::Vector3d RobotPosition3DTem_(world_base_transform.getOrigin().x(), world_base_transform.getOrigin().y(),
                                      world_base_transform.getOrigin().z() - RobotHeight_);
  double MinEuclideanDistanceToRobotTem_ = 9999999;
  Eigen::Vector3d TargetPosition3DTem_;
  bool IfFindTem_ = false;
  for (auto CenterTem_ : SourceCenters)
  {
    if (IfContrainTheExplorationArea)
    {
      if ((CenterTem_[0] < ConstrainsOfTheExplorationArea(0, 0)) ||
          (CenterTem_[0] > ConstrainsOfTheExplorationArea(0, 1)) ||
          (CenterTem_[1] < ConstrainsOfTheExplorationArea(1, 0)) ||
          (CenterTem_[1] > ConstrainsOfTheExplorationArea(1, 1)) ||
          (CenterTem_[2] < ConstrainsOfTheExplorationArea(2, 0)) ||
          (CenterTem_[2] > ConstrainsOfTheExplorationArea(2, 1)))
      {
        continue;
      }
    }
    VOXEL_LOC CenterPosition3DTem_(CenterTem_[0], CenterTem_[1], (double)((int)(CenterTem_[2] / 0.5)) * 0.5);
    if (NotReachableTargetPositions3D.find(CenterPosition3DTem_) != NotReachableTargetPositions3D.end())
    {
      continue;
    }
    Eigen::Vector3d Distance3DTem_(RobotPosition3DTem_[0] - CenterTem_[0], RobotPosition3DTem_[1] - CenterTem_[1],
                                   RobotPosition3DTem_[2] - CenterTem_[2]);
    if (Distance3DTem_.norm() < MinEuclideanDistanceToRobotTem_)
    {
      MinEuclideanDistanceToRobotTem_ = Distance3DTem_.norm();
      TargetPosition3DTem_ << CenterTem_[0], CenterTem_[1], CenterTem_[2];
      IfFindTem_ = true;
    }
  }
  if (IfFindTem_)
  {
    OutpuTargetPosition = TargetPosition3DTem_;
    return true;
  }
  else
  {
    return false;
  }
  return false;
}

void PathPlanner::ComputeTotalForce(const Eigen::Vector3d RobotPosition, const Eigen::Vector3d TargetPosition,
                                    const std::vector<Eigen::Vector3d> Obstacles, Eigen::Vector3d& Force,
                                    double AttractiveGain, double RepulsiveGain, double BoundRadiu)
{
  Eigen::Vector3d force_internal;
  force_internal << 0, 0, 0;
  // fx = fy = 0;
  //  Calculate attractive force towards the goal
  // double gx = 5, gy = 5; // Goal position
  double dx_att = TargetPosition[0] - RobotPosition[0];
  double dy_att = TargetPosition[1] - RobotPosition[1];
  double dz_att = TargetPosition[2] - RobotPosition[2];

  double d = std::sqrt(dx_att * dx_att + dy_att * dy_att + dz_att * dz_att);
  if (d == 0)
  {
    force_internal[0] += 0;
    force_internal[1] += 0;
    force_internal[2] += 0;
  }
  else
  {
    force_internal[0] += AttractiveGain * dx_att / d;
    force_internal[1] += AttractiveGain * dy_att / d;
    force_internal[2] += AttractiveGain * dz_att / d;
  }

  // Calculate repulsive forces from obstacles
  for (auto bound = Obstacles.begin(); bound != Obstacles.end(); ++bound)
  {
    Eigen::Vector3d obstacle_middle_ = *bound;
    double ox = obstacle_middle_[0];
    double oy = obstacle_middle_[1];
    double oz = obstacle_middle_[2];
    double radiu = BoundRadiu;

    double dx = RobotPosition[0] - ox;
    double dy = RobotPosition[1] - oy;
    double dz = RobotPosition[2] - oz;
    double d_re = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (d_re < radiu)
    {
      if (d_re == 0)
      {
        force_internal[0] += 100;
        force_internal[1] += 100;
        force_internal[2] += 100;
      }
      else
      {
        double f = RepulsiveGain * (1 / d_re - 1 / radiu) / (d_re * d_re);
        force_internal[0] += f * dx / d_re;
        force_internal[1] += f * dy / d_re;
        force_internal[2] += f * dz / d_re;
      }
    }
  }
  Force = force_internal;
}

void PathPlanner::ComputeTotalForce(const Eigen::Vector3d RobotPosition, const Eigen::Vector3d TargetPosition,
                                    const std::set<Eigen::Vector3d, setcomp> Obstacles, Eigen::Vector3d& Force,
                                    double AttractiveGain, double RepulsiveGain, double BoundRadiu)
{
  Eigen::Vector3d force_internal;
  force_internal << 0, 0, 0;
  // fx = fy = 0;
  //  Calculate attractive force towards the goal
  // double gx = 5, gy = 5; // Goal position
  double dx_att = TargetPosition[0] - RobotPosition[0];
  double dy_att = TargetPosition[1] - RobotPosition[1];
  double dz_att = TargetPosition[2] - RobotPosition[2];

  double d = std::sqrt(dx_att * dx_att + dy_att * dy_att + dz_att * dz_att);
  if (d == 0)
  {
    force_internal[0] += 0;
    force_internal[1] += 0;
    force_internal[2] += 0;
  }
  else
  {
    force_internal[0] += AttractiveGain * dx_att / d;
    force_internal[1] += AttractiveGain * dy_att / d;
    force_internal[2] += AttractiveGain * dz_att / d;
  }

  // Calculate repulsive forces from obstacles
  for (auto bound = Obstacles.begin(); bound != Obstacles.end(); ++bound)
  {
    Eigen::Vector3d obstacle_middle_ = *bound;
    double ox = obstacle_middle_[0];
    double oy = obstacle_middle_[1];
    double oz = obstacle_middle_[2];
    double radiu = BoundRadiu;

    double dx = RobotPosition[0] - ox;
    double dy = RobotPosition[1] - oy;
    double dz = RobotPosition[2] - oz;
    double d_re = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (d_re < radiu)
    {
      if (d_re == 0)
      {
        force_internal[0] += 100;
        force_internal[1] += 100;
        force_internal[2] += 100;
      }
      else
      {
        double f = RepulsiveGain * (1 / d_re - 1 / radiu) / (d_re * d_re);
        force_internal[0] += f * dx / d_re;
        force_internal[1] += f * dy / d_re;
        force_internal[2] += f * dz / d_re;
      }
    }
  }
  Force = force_internal;
}

void PathPlanner::GenerateCostMap(const std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                  std::unordered_map<int, double>& OutputCostMap, const Eigen::Vector3d TargetPosition,
                                  double AttractiveGain, double RepulsiveGain, double BoundRadiu,
                                  const std::set<Eigen::Vector3d, setcomp> ObstaclesSet)
{
  std::unordered_map<int, double> cost_unorder_map_;
  cost_unorder_map_.clear();
  OutputCostMap.clear();
  std::unordered_map<int, std::unordered_map<int, double>> MapCopyTem_;
  SourceVRGraphManager->getUndirectedMap(MapCopyTem_);
  for (auto VertexTem1_ : MapCopyTem_)
  {
    int id = VertexTem1_.first;
    const Vertex* vertex_temp_ = SourceVRGraphManager->getVertex(id);
    Eigen::Vector3d point_pos_(vertex_temp_->state[0], vertex_temp_->state[1], vertex_temp_->state[2]);
    Eigen::Vector3d total_force_;
    ComputeTotalForce(point_pos_, TargetPosition, ObstaclesSet, total_force_, AttractiveGain, RepulsiveGain,
                      BoundRadiu);

    double total_force_norm = total_force_.norm();
    cost_unorder_map_[id] = total_force_norm;
  }
  OutputCostMap = cost_unorder_map_;
}

void PathPlanner::GenerateUndirectedMapByPotentialField(const std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                                        VRGraphManager::GraphType& OuputMap, int TargetId,
                                                        double AttractiveGain, double RepulsiveGain, double BoundRadiu,
                                                        const std::set<Eigen::Vector3d, setcomp> ObstaclesSet)
{
  OuputMap.reset();
  const Vertex* TargetVertexTem_ = SourceVRGraphManager->getVertex(TargetId);
  std::unordered_map<int, double> CostMapTem_;
  Eigen::Vector3d TargetPositionTem_(TargetVertexTem_->state[0], TargetVertexTem_->state[1],
                                     TargetVertexTem_->state[2]);

  CostMapTem_.clear();
  GenerateCostMap(SourceVRGraphManager, CostMapTem_, TargetPositionTem_, AttractiveGain, RepulsiveGain, BoundRadiu,
                  ObstaclesSet);
  std::unordered_map<int, std::unordered_map<int, double>> MapCopyTem_;
  SourceVRGraphManager->getUndirectedMap(MapCopyTem_);
  for (auto VertexTem1_ : MapCopyTem_)
  {
    int IdTem1_ = VertexTem1_.first;
    double Force1Tem_ = CostMapTem_[IdTem1_];
    for (auto VertexTem2_ : VertexTem1_.second)
    {
      int IdTem2_ = VertexTem2_.first;
      double WeightOldTem_ = MapCopyTem_[IdTem1_][IdTem2_];
      double Force2Tem_ = CostMapTem_[IdTem2_];
      double TotalForceTem_ = (Force1Tem_ + Force2Tem_) / 2;
      double WeightNewTem_ = (WeightOldTem_) + (TotalForceTem_) / 2;
      MapCopyTem_[IdTem1_][IdTem2_] = WeightNewTem_;
    }
  }
  VRGraphManager::GraphType OuputMapTem_(MapCopyTem_);
  OuputMap = OuputMapTem_;
}

std::vector<int> PathPlanner::aStar(std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                    VRGraphManager::GraphType SourceUnorderedMap, int start, int goal)
{
  std::unordered_map<int, double> gScore;
  std::unordered_map<int, double> fScore;
  std::unordered_map<int, int> cameFrom;
  std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>>
      openSet;
  for (const auto& vertex : SourceUnorderedMap.adjacencyList_)
  {
    gScore[vertex.first] = std::numeric_limits<double>::max();
    cameFrom[vertex.first] = -1;
  }

  gScore[start] = 0;
  fScore[start] = heuristic(SourceVRGraphManager, start, goal);
  openSet.push({ fScore[start], start });

  while (!openSet.empty())
  {
    int current = openSet.top().second;
    openSet.pop();

    if (current == goal)
    {
      return SourceUnorderedMap.reconstructPath(cameFrom, goal);
    }

    for (const auto& neighbor : SourceUnorderedMap.adjacencyList_[current])
    {
      int neighborID = neighbor.first;
      double neighborDist = neighbor.second;

      double tentativeGScore = gScore[current] + neighborDist;
      if (gScore.find(neighborID) == gScore.end() || tentativeGScore < gScore[neighborID])
      {
        cameFrom[neighborID] = current;
        gScore[neighborID] = tentativeGScore;
        fScore[neighborID] = tentativeGScore + heuristic(SourceVRGraphManager, neighborID, goal);
        openSet.push({ fScore[neighborID], neighborID });
      }
    }
  }

  // No path found
  return std::vector<int>();
}
double PathPlanner::heuristic(std::shared_ptr<VRGraphManager> SourceVRGraphManager, int id1, int id2)
{
  Vertex* Vertex1 = SourceVRGraphManager->getVertex(id1);
  Vertex* Vertex2 = SourceVRGraphManager->getVertex(id2);
  Eigen::Vector3d Distance3DTem_(Vertex2->state[0] - Vertex1->state[0], Vertex2->state[1] - Vertex1->state[1],
                                 Vertex2->state[2] - Vertex1->state[2]);
  return Distance3DTem_.norm();
}

bool PathPlanner::PlanPathByAStarMethod(std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                        VRGraphManager::GraphType SourceUnorderedMap, int TargetId,
                                        std::vector<int>& PathIdOutput, string WorldFrame, string RobotFrame)
{
  int RobotId_ = -1;
  tf::TransformListener world_base_listener;
  tf::StampedTransform world_base_transform;
  try
  {
    world_base_listener.waitForTransform(WorldFrame.c_str(), RobotFrame.c_str(), ros::Time(0), ros::Duration(1.0));
    world_base_listener.lookupTransform(WorldFrame.c_str(), RobotFrame.c_str(), ros::Time(0), world_base_transform);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    // ros::Duration(1.0).sleep();
  }
  StateVec robot_state_(world_base_transform.getOrigin().x(), world_base_transform.getOrigin().y(),
                        world_base_transform.getOrigin().z() - RobotHeight_, 0);

  std::vector<Vertex*> RobotNearestVerticesSelectedTem_;
  RobotNearestVerticesSelectedTem_.clear();
  SourceVRGraphManager->getNearestVertices(&robot_state_, 1.0, &RobotNearestVerticesSelectedTem_);
  Eigen::Vector2d RobotPosition2DTem_(robot_state_[0], robot_state_[1]);
  for (size_t i = 0; i < RobotNearestVerticesSelectedTem_.size(); i++)
  {
    Vertex* NearestVertexTem_ = RobotNearestVerticesSelectedTem_[i];
    if (NearestVertexTem_->is_obstacle)
    {
      continue;
    }
    if (abs(NearestVertexTem_->state[2] - robot_state_[2]) > RobotHeight_)
    {
      continue;
    }
    Eigen::Vector2d VertexPosition2DTem(NearestVertexTem_->state[0], NearestVertexTem_->state[1]);
    Eigen::Vector2d Distance2DTem_ = VertexPosition2DTem - RobotPosition2DTem_;
    if (Distance2DTem_.norm() < NearestVertexTem_->resolution)
    {
      RobotId_ = NearestVertexTem_->id;
      break;
    }
  }

  if (RobotId_ == -1)
  {
    ROS_ERROR("[PathPlanner_Info]: Could not find the global id of robot.");
    return false;
  }

  // Vertex *robot_nearest_vertices;
  // if (SourceVRGraphManager->getNearestVertex(&robot_state_, &robot_nearest_vertices))
  // {
  //     RobotId_ = robot_nearest_vertices->id;
  // }
  // else
  // {
  //     ROS_ERROR("[PathPlanner_Info]: Could not find the global id of robot.");
  //     return false;
  // }
  PathIdOutput.clear();
  PathIdOutput = aStar(SourceVRGraphManager, SourceUnorderedMap, RobotId_, TargetId);
  if (PathIdOutput.size() == 0)
  {
    ROS_ERROR("[PathPlanner_Info]: Path generated by A-Star size represented by id is 0.");
    return false;
  }
  return true;
}

bool PathPlanner::PlanPathByAStarMethod(std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                        VRGraphManager::GraphType SourceUnorderedMap, int TargetId,
                                        std::vector<Eigen::Vector3d>& PathPositionOutput, string WorldFrame,
                                        string RobotFrame)
{
  PathPositionOutput.clear();
  std::vector<int> PathIdOutputForTem_;
  PathIdOutputForTem_.clear();
  bool IfSucceedPlanWithId_ = false;
  IfSucceedPlanWithId_ = PlanPathByAStarMethod(SourceVRGraphManager, SourceUnorderedMap, TargetId, PathIdOutputForTem_,
                                               WorldFrame, RobotFrame);
  if (!IfSucceedPlanWithId_)
  {
    return false;
  }
  for (size_t iTem_ = 0; iTem_ < PathIdOutputForTem_.size(); iTem_++)
  {
    Vertex* VertexPtrTem_ = SourceVRGraphManager->getVertex(PathIdOutputForTem_[iTem_]);
    if (VertexPtrTem_ == NULL)
    {
      ROS_ERROR("[PathPlanner_Info]: Can not get the vertex from SourceGraphManager.");
      return false;
    }
    Eigen::Vector3d PositionTem_(VertexPtrTem_->state[0], VertexPtrTem_->state[1], VertexPtrTem_->state[2]);
    PathPositionOutput.push_back(PositionTem_);
  }
  if (PathPositionOutput.size() == 0)
  {
    ROS_ERROR("[PathPlanner_Info]: Path generated by A-Star size represented by Eigen::Vector3d is 0.");
    return false;
  }
  return true;
}

bool PathPlanner::GenerateLocalGraphManagerForPathPlanner(
    std::shared_ptr<VRGraphManager> SourceVRGraphManager, Eigen::Vector3d StartPosition3D,
    Eigen::Vector3d TargetPosition3D, Eigen::Vector3d ExpandVector, int TargetId, double AttractiveGain,
    double RepulsiveGain, double BoundRadiu, std::shared_ptr<VRGraphManager> TargetVRGraphManager,
    VRGraphManager::GraphType& OuputUndirectedMap)
{
  // TargetVRGraphManager->ResetWithoutDelete();
  TargetVRGraphManager->ResetWithoutDeleteAndIgnoreIdPool();
  Eigen::Vector3d CenterPositionTem_ = (StartPosition3D + TargetPosition3D) / 2;
  double DistanceXTem_ = abs(TargetPosition3D[0] - CenterPositionTem_[0]) + ExpandVector[0];
  double DistanceYTem_ = abs(TargetPosition3D[1] - CenterPositionTem_[1]) + ExpandVector[1];
  double DistanceZTem_ = abs(TargetPosition3D[2] - CenterPositionTem_[2]) + ExpandVector[2];
  StateVec StateTem_(CenterPositionTem_[0], CenterPositionTem_[1], CenterPositionTem_[2], 0);
  std::vector<Vertex*> VerticesTem_;
  if (!SourceVRGraphManager->getNearestVerticesInBox(&StateTem_, DistanceXTem_, DistanceYTem_, DistanceZTem_,
                                                     &VerticesTem_))
  {
    ROS_ERROR("[PathPlanner_Info]: Can not extract vertices when generate local graph.");
    return false;
  }
  std::set<int> IdsInLocalGraphTem_;
  IdsInLocalGraphTem_.clear();
  std::unordered_map<int, std::unordered_map<int, double>> MapCopyTem_;
  SourceVRGraphManager->getUndirectedMap(MapCopyTem_);
  std::set<Eigen::Vector3d, setcomp> ObstaclesSetTem_;
  ObstaclesSetTem_.clear();
  for (size_t i = 0; i < VerticesTem_.size(); i++)
  {
    if (VerticesTem_[i]->is_obstacle)
    {
      Eigen::Vector3d ObstaclePosition3DTem_(VerticesTem_[i]->state[0], VerticesTem_[i]->state[1],
                                             VerticesTem_[i]->state[2]);
      ObstaclesSetTem_.insert(ObstaclePosition3DTem_);
      continue;
    }
    if (VerticesTem_[i]->id == 0)
    {
      Vertex* Vertedx0Tem_ = TargetVRGraphManager->getVertex(0);
      Vertedx0Tem_->state = VerticesTem_[i]->state;
      Vertedx0Tem_->is_obstacle = VerticesTem_[i]->is_obstacle;
      Vertedx0Tem_->resolution = VerticesTem_[i]->resolution;
      IdsInLocalGraphTem_.insert(0);
    }
    else
    {
      TargetVRGraphManager->addVertex(VerticesTem_[i]);
      IdsInLocalGraphTem_.insert(VerticesTem_[i]->id);
    }

    for (auto IteratorTem_ : MapCopyTem_[VerticesTem_[i]->id])
    {
      int IdToBeConnectedTem_ = IteratorTem_.first;
      if ((IdsInLocalGraphTem_.find(IdToBeConnectedTem_) != IdsInLocalGraphTem_.end()) &&
          (IdToBeConnectedTem_ != VerticesTem_[i]->id))
      {
        Vertex* VertexTem2_ = TargetVRGraphManager->getVertex(IdToBeConnectedTem_);
        Eigen::Vector3d EdgeTem_(VertexTem2_->state[0] - VerticesTem_[i]->state[0],
                                 VertexTem2_->state[1] - VerticesTem_[i]->state[1],
                                 VertexTem2_->state[2] - VerticesTem_[i]->state[2]);
        double EdgeLengthTem_ = EdgeTem_.norm();
        TargetVRGraphManager->addEdge(VerticesTem_[i], VertexTem2_, EdgeLengthTem_);
      }
    }
  }
  // GenerateUndirectedMapByPotentialField(TargetVRGraphManager,
  //                                       OuputUndirectedMap,
  //                                       TargetId,
  //                                       AttractiveGain,
  //                                       RepulsiveGain,
  //                                       BoundRadiu,
  //                                       TargetVRGraphManager->ObstaclesSet_);
  GenerateUndirectedMapByPotentialField(TargetVRGraphManager, OuputUndirectedMap, TargetId, AttractiveGain,
                                        RepulsiveGain, BoundRadiu, ObstaclesSetTem_);
  return true;
}

bool PathPlanner::GenerateLocalGraphManagerForPathPlanner(std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                                          Eigen::Vector3d TargetPosition3D,
                                                          Eigen::Vector3d ExpandVector, int TargetId,
                                                          double AttractiveGain, double RepulsiveGain,
                                                          double BoundRadiu, string WorldFrame, string RobotFrame,
                                                          std::shared_ptr<VRGraphManager> TargetVRGraphManager,
                                                          VRGraphManager::GraphType& OuputUndirectedMap)
{
  tf::TransformListener world_base_listener;
  tf::StampedTransform world_base_transform;
  try
  {
    world_base_listener.waitForTransform(WorldFrame.c_str(), RobotFrame.c_str(), ros::Time(0), ros::Duration(1.0));
    world_base_listener.lookupTransform(WorldFrame.c_str(), RobotFrame.c_str(), ros::Time(0), world_base_transform);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    return false;
    // ros::Duration(1.0).sleep();
  }
  StateVec robot_state_(world_base_transform.getOrigin().x(), world_base_transform.getOrigin().y(),
                        world_base_transform.getOrigin().z() - RobotHeight_, 0);
  Eigen::Vector3d StartPositionTem_(0, 0, 0);
  Vertex* robot_nearest_vertices;
  if (SourceVRGraphManager->getNearestVertex(&robot_state_, &robot_nearest_vertices))
  {
    StartPositionTem_ << robot_nearest_vertices->state[0], robot_nearest_vertices->state[1],
        robot_nearest_vertices->state[2];
  }
  else
  {
    ROS_ERROR("[PathPlanner_Info]: Could not find the global vertex of robot.");
    return false;
  }
  return GenerateLocalGraphManagerForPathPlanner(SourceVRGraphManager, StartPositionTem_, TargetPosition3D,
                                                 ExpandVector, TargetId, AttractiveGain, RepulsiveGain, BoundRadiu,
                                                 TargetVRGraphManager, OuputUndirectedMap);
}

bool PathPlanner::GenerateLocalGraphManagerForPathPlanner(std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                                          Eigen::Vector3d StartPosition3D,
                                                          Eigen::Vector3d TargetPosition3D,
                                                          Eigen::Vector3d ExpandVector, double ResolutionMin,
                                                          double VRMapResolutionMax, double TargetLocalBoxZConstrain,
                                                          std::shared_ptr<VRGraphManager> TargetVRGraphManager)
{
  TargetVRGraphManager->ResetWithoutDeleteAndIgnoreIdPool();
  Eigen::Vector3d CenterPositionTem_ = (StartPosition3D + TargetPosition3D) / 2;
  double DistanceXTem_ = abs(TargetPosition3D[0] - CenterPositionTem_[0]) + ExpandVector[0];
  double DistanceYTem_ = abs(TargetPosition3D[1] - CenterPositionTem_[1]) + ExpandVector[1];
  double DistanceZTem_ = abs(TargetPosition3D[2] - CenterPositionTem_[2]) + ExpandVector[2];
  StateVec StateTem_(CenterPositionTem_[0], CenterPositionTem_[1], CenterPositionTem_[2], 0);
  std::vector<Vertex*> VerticesTem_;
  if (!SourceVRGraphManager->getNearestVerticesInBox(&StateTem_, DistanceXTem_, DistanceYTem_, DistanceZTem_,
                                                     &VerticesTem_))
  {
    ROS_ERROR("[PathPlanner_Info]: Can not extract vertices when generate local graph.");
    return false;
  }
  int StartLoclaBoxRowIndexTem_;
  int StartLoclaBoxColIndexTem_;
  if (StartPosition3D[0] < -0.5 * VRMapResolutionMax)
  {
    StartLoclaBoxRowIndexTem_ = (StartPosition3D[0] + VRMapResolutionMax / 2) / VRMapResolutionMax - 1;
  }
  else
  {
    StartLoclaBoxRowIndexTem_ = (StartPosition3D[0] + VRMapResolutionMax / 2) / VRMapResolutionMax;
  }

  if (StartPosition3D[1] < -0.5 * VRMapResolutionMax)
  {
    StartLoclaBoxColIndexTem_ = (StartPosition3D[1] + VRMapResolutionMax / 2) / VRMapResolutionMax - 1;
  }
  else
  {
    StartLoclaBoxColIndexTem_ = (StartPosition3D[1] + VRMapResolutionMax / 2) / VRMapResolutionMax;
  }
  Eigen::Vector3d StartLocalBoxCenterPosition3DTem_((double)(StartLoclaBoxRowIndexTem_)*VRMapResolutionMax,
                                                    (double)(StartLoclaBoxColIndexTem_)*VRMapResolutionMax,
                                                    StartPosition3D[2]);
  Eigen::Matrix<double, 3, 2> StartBoxTem_;
  StartBoxTem_ << StartLocalBoxCenterPosition3DTem_[0] - VRMapResolutionMax,
      StartLocalBoxCenterPosition3DTem_[0] + VRMapResolutionMax,
      StartLocalBoxCenterPosition3DTem_[1] - VRMapResolutionMax,
      StartLocalBoxCenterPosition3DTem_[1] + VRMapResolutionMax,
      StartLocalBoxCenterPosition3DTem_[2] - TargetLocalBoxZConstrain,
      StartLocalBoxCenterPosition3DTem_[2] + TargetLocalBoxZConstrain;
  std::set<int> IdsInsideStartBoxTem_;
  IdsInsideStartBoxTem_.clear();
  int TargetLoclaBoxRowIndexTem_;
  int TargetLoclaBoxColIndexTem_;
  if (TargetPosition3D[0] < -0.5 * VRMapResolutionMax)
  {
    TargetLoclaBoxRowIndexTem_ = (TargetPosition3D[0] + VRMapResolutionMax / 2) / VRMapResolutionMax - 1;
  }
  else
  {
    TargetLoclaBoxRowIndexTem_ = (TargetPosition3D[0] + VRMapResolutionMax / 2) / VRMapResolutionMax;
  }

  if (TargetPosition3D[1] < -0.5 * VRMapResolutionMax)
  {
    TargetLoclaBoxColIndexTem_ = (TargetPosition3D[1] + VRMapResolutionMax / 2) / VRMapResolutionMax - 1;
  }
  else
  {
    TargetLoclaBoxColIndexTem_ = (TargetPosition3D[1] + VRMapResolutionMax / 2) / VRMapResolutionMax;
  }
  Eigen::Vector3d TargetLocalBoxCenterPosition3DTem_((double)(TargetLoclaBoxRowIndexTem_)*VRMapResolutionMax,
                                                     (double)(TargetLoclaBoxColIndexTem_)*VRMapResolutionMax,
                                                     TargetPosition3D[2]);
  Eigen::Matrix<double, 3, 2> TargetBoxTem_;
  TargetBoxTem_ << TargetLocalBoxCenterPosition3DTem_[0] - VRMapResolutionMax,
      TargetLocalBoxCenterPosition3DTem_[0] + VRMapResolutionMax,
      TargetLocalBoxCenterPosition3DTem_[1] - VRMapResolutionMax,
      TargetLocalBoxCenterPosition3DTem_[1] + VRMapResolutionMax,
      TargetLocalBoxCenterPosition3DTem_[2] - TargetLocalBoxZConstrain,
      TargetLocalBoxCenterPosition3DTem_[2] + TargetLocalBoxZConstrain;

  std::set<int> IdsInLocalGraphTem_;
  IdsInLocalGraphTem_.clear();
  std::unordered_map<int, std::unordered_map<int, double>> MapCopyTem_;
  SourceVRGraphManager->getUndirectedMap(MapCopyTem_);
  std::set<int> IdsInsideTem_;
  IdsInsideTem_.clear();
  for (size_t i = 0; i < VerticesTem_.size(); i++)
  {
    if (VerticesTem_[i]->is_obstacle)
    {
      continue;
    }
    bool InsideTargetLocalBoxTem_ = false;
    if ((VerticesTem_[i]->state[0] < TargetBoxTem_(0, 0)) || (VerticesTem_[i]->state[0] > TargetBoxTem_(0, 1)) ||
        (VerticesTem_[i]->state[1] < TargetBoxTem_(1, 0)) || (VerticesTem_[i]->state[1] > TargetBoxTem_(1, 1)) ||
        (VerticesTem_[i]->state[2] < TargetBoxTem_(2, 0)) || (VerticesTem_[i]->state[2] > TargetBoxTem_(2, 1)))
    {
      InsideTargetLocalBoxTem_ = false;
    }
    else
    {
      IdsInsideTem_.insert(VerticesTem_[i]->id);
      InsideTargetLocalBoxTem_ = true;
    }
    bool InsideStartLocalBoxTem_ = false;
    if ((VerticesTem_[i]->state[0] < StartBoxTem_(0, 0)) || (VerticesTem_[i]->state[0] > StartBoxTem_(0, 1)) ||
        (VerticesTem_[i]->state[1] < StartBoxTem_(1, 0)) || (VerticesTem_[i]->state[1] > StartBoxTem_(1, 1)) ||
        (VerticesTem_[i]->state[2] < StartBoxTem_(2, 0)) || (VerticesTem_[i]->state[2] > StartBoxTem_(2, 1)))
    {
      InsideStartLocalBoxTem_ = false;
    }
    else
    {
      IdsInsideStartBoxTem_.insert(VerticesTem_[i]->id);
      InsideStartLocalBoxTem_ = true;
    }

    if ((VerticesTem_[i]->resolution < ResolutionMin) && (!InsideTargetLocalBoxTem_) && (!InsideStartLocalBoxTem_))
    {
      continue;
    }
    if (VerticesTem_[i]->id == 0)
    {
      Vertex* Vertedx0Tem_ = TargetVRGraphManager->getVertex(0);
      Vertedx0Tem_->state = VerticesTem_[i]->state;
      Vertedx0Tem_->is_obstacle = VerticesTem_[i]->is_obstacle;
      Vertedx0Tem_->resolution = VerticesTem_[i]->resolution;
      IdsInLocalGraphTem_.insert(0);
    }
    else
    {
      TargetVRGraphManager->addVertex(VerticesTem_[i]);
      IdsInLocalGraphTem_.insert(VerticesTem_[i]->id);
    }

    for (auto IteratorTem_ : MapCopyTem_[VerticesTem_[i]->id])
    {
      int IdToBeConnectedTem_ = IteratorTem_.first;
      if ((IdsInLocalGraphTem_.find(IdToBeConnectedTem_) != IdsInLocalGraphTem_.end()) &&
          (IdToBeConnectedTem_ != VerticesTem_[i]->id))
      {
        Vertex* VertexTem2_ = TargetVRGraphManager->getVertex(IdToBeConnectedTem_);
        Eigen::Vector3d EdgeTem_(VertexTem2_->state[0] - VerticesTem_[i]->state[0],
                                 VertexTem2_->state[1] - VerticesTem_[i]->state[1],
                                 VertexTem2_->state[2] - VerticesTem_[i]->state[2]);
        double EdgeLengthTem_ = EdgeTem_.norm();
        // if ((InsideTargetLocalBoxTem_) &&
        //     ((IdsInsideTem_.find(IdToBeConnectedTem_) != IdsInsideTem_.end()) ||
        //      (IdsInsideStartBoxTem_.find(IdToBeConnectedTem_) != IdsInsideStartBoxTem_.end())) &&
        //     ((VertexTem2_->resolution < ResolutionMin) &&
        //      (VerticesTem_[i]->resolution < ResolutionMin)))
        if (((VertexTem2_->resolution < ResolutionMin) && (VerticesTem_[i]->resolution < ResolutionMin)) &&
            ((InsideTargetLocalBoxTem_) || (InsideStartLocalBoxTem_)) &&
            ((IdsInsideTem_.find(IdToBeConnectedTem_) != IdsInsideTem_.end()) ||
             (IdsInsideStartBoxTem_.find(IdToBeConnectedTem_) != IdsInsideStartBoxTem_.end())))
        {
          EdgeLengthTem_ = EdgeLengthTem_ + 10.0;
        }
        TargetVRGraphManager->addEdge(VerticesTem_[i], VertexTem2_, EdgeLengthTem_);
      }
    }
  }
  return true;
}

bool PathPlanner::GenerateLocalGraphManagerForPathPlanner(std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                                          string WorldFrame, string RobotFrame,
                                                          Eigen::Vector3d TargetPosition3D,
                                                          Eigen::Vector3d ExpandVector, double ResolutionMin,
                                                          double VRMapResolutionMax, double TargetLocalBoxZConstrain,
                                                          std::shared_ptr<VRGraphManager> TargetVRGraphManager)
{
  tf::TransformListener world_base_listener;
  tf::StampedTransform world_base_transform;
  try
  {
    world_base_listener.waitForTransform(WorldFrame.c_str(), RobotFrame.c_str(), ros::Time(0), ros::Duration(1.0));
    world_base_listener.lookupTransform(WorldFrame.c_str(), RobotFrame.c_str(), ros::Time(0), world_base_transform);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    return false;
    // ros::Duration(1.0).sleep();
  }
  StateVec robot_state_(world_base_transform.getOrigin().x(), world_base_transform.getOrigin().y(),
                        world_base_transform.getOrigin().z() - RobotHeight_, 0);
  Eigen::Vector3d StartPositionTem_(0, 0, 0);
  Vertex* robot_nearest_vertices;
  if (SourceVRGraphManager->getNearestVertex(&robot_state_, &robot_nearest_vertices))
  {
    StartPositionTem_ << robot_nearest_vertices->state[0], robot_nearest_vertices->state[1],
        robot_nearest_vertices->state[2];
  }
  else
  {
    ROS_ERROR("[PathPlanner_Info]: Could not find the global vertex of robot.");
    return false;
  }
  return GenerateLocalGraphManagerForPathPlanner(SourceVRGraphManager, StartPositionTem_, TargetPosition3D,
                                                 ExpandVector, ResolutionMin, VRMapResolutionMax,
                                                 TargetLocalBoxZConstrain, TargetVRGraphManager);
}

bool PathPlanner::GenerateLocalGraphManagerForPathPlanner(std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                                          Eigen::Vector3d TargetPosition3D, double ResolutionMin,
                                                          double VRMapResolutionMax, double TargetLocalBoxZConstrain,
                                                          std::shared_ptr<VRGraphManager> TargetVRGraphManager)
{
  TargetVRGraphManager->ResetWithoutDeleteAndIgnoreIdPool();
  if (SourceVRGraphManager->vertices_map_.size() < 1)
  {
    ROS_ERROR("[PathPlanner_Info]: Can not extract vertices when generate local graph as the size is min than 1.");
    return false;
  }
  int TargetLoclaBoxRowIndexTem_;
  int TargetLoclaBoxColIndexTem_;
  if (TargetPosition3D[0] < -0.5 * VRMapResolutionMax)
  {
    TargetLoclaBoxRowIndexTem_ = (TargetPosition3D[0] + VRMapResolutionMax / 2) / VRMapResolutionMax - 1;
  }
  else
  {
    TargetLoclaBoxRowIndexTem_ = (TargetPosition3D[0] + VRMapResolutionMax / 2) / VRMapResolutionMax;
  }

  if (TargetPosition3D[1] < -0.5 * VRMapResolutionMax)
  {
    TargetLoclaBoxColIndexTem_ = (TargetPosition3D[1] + VRMapResolutionMax / 2) / VRMapResolutionMax - 1;
  }
  else
  {
    TargetLoclaBoxColIndexTem_ = (TargetPosition3D[1] + VRMapResolutionMax / 2) / VRMapResolutionMax;
  }
  Eigen::Vector3d TargetLocalBoxCenterPosition3DTem_((double)(TargetLoclaBoxRowIndexTem_)*VRMapResolutionMax,
                                                     (double)(TargetLoclaBoxColIndexTem_)*VRMapResolutionMax,
                                                     TargetPosition3D[2]);
  Eigen::Matrix<double, 3, 2> TargetBoxTem_;
  TargetBoxTem_ << TargetLocalBoxCenterPosition3DTem_[0] - VRMapResolutionMax,
      TargetLocalBoxCenterPosition3DTem_[0] + VRMapResolutionMax,
      TargetLocalBoxCenterPosition3DTem_[1] - VRMapResolutionMax,
      TargetLocalBoxCenterPosition3DTem_[1] + VRMapResolutionMax,
      TargetLocalBoxCenterPosition3DTem_[2] - TargetLocalBoxZConstrain,
      TargetLocalBoxCenterPosition3DTem_[2] + TargetLocalBoxZConstrain;

  std::set<int> IdsInLocalGraphTem_;
  IdsInLocalGraphTem_.clear();
  std::unordered_map<int, std::unordered_map<int, double>> MapCopyTem_;
  SourceVRGraphManager->getUndirectedMap(MapCopyTem_);
  for (auto VerticesIteratorTem_ : SourceVRGraphManager->vertices_map_)
  {
    if (VerticesIteratorTem_.second->is_obstacle)
    {
      continue;
    }
    bool InsideTargetLocalBoxTem_ = false;
    if ((VerticesIteratorTem_.second->state[0] < TargetBoxTem_(0, 0)) ||
        (VerticesIteratorTem_.second->state[0] > TargetBoxTem_(0, 1)) ||
        (VerticesIteratorTem_.second->state[1] < TargetBoxTem_(1, 0)) ||
        (VerticesIteratorTem_.second->state[1] > TargetBoxTem_(1, 1)) ||
        (VerticesIteratorTem_.second->state[2] < TargetBoxTem_(2, 0)) ||
        (VerticesIteratorTem_.second->state[2] > TargetBoxTem_(2, 1)))
    {
      InsideTargetLocalBoxTem_ = false;
    }
    else
    {
      InsideTargetLocalBoxTem_ = true;
    }
    if ((VerticesIteratorTem_.second->resolution < ResolutionMin) && (!InsideTargetLocalBoxTem_))
    {
      continue;
    }
    if (VerticesIteratorTem_.second->id == 0)
    {
      Vertex* Vertedx0Tem_ = TargetVRGraphManager->getVertex(0);
      Vertedx0Tem_->state = VerticesIteratorTem_.second->state;
      Vertedx0Tem_->is_obstacle = VerticesIteratorTem_.second->is_obstacle;
      Vertedx0Tem_->resolution = VerticesIteratorTem_.second->resolution;
      IdsInLocalGraphTem_.insert(0);
    }
    else
    {
      TargetVRGraphManager->addVertex(VerticesIteratorTem_.second);
      IdsInLocalGraphTem_.insert(VerticesIteratorTem_.second->id);
    }

    for (auto IteratorTem_ : MapCopyTem_[VerticesIteratorTem_.second->id])
    {
      int IdToBeConnectedTem_ = IteratorTem_.first;
      if ((IdsInLocalGraphTem_.find(IdToBeConnectedTem_) != IdsInLocalGraphTem_.end()) &&
          (IdToBeConnectedTem_ != VerticesIteratorTem_.second->id))
      {
        Vertex* VertexTem2_ = TargetVRGraphManager->getVertex(IdToBeConnectedTem_);
        Eigen::Vector3d EdgeTem_(VertexTem2_->state[0] - VerticesIteratorTem_.second->state[0],
                                 VertexTem2_->state[1] - VerticesIteratorTem_.second->state[1],
                                 VertexTem2_->state[2] - VerticesIteratorTem_.second->state[2]);
        double EdgeLengthTem_ = EdgeTem_.norm();
        TargetVRGraphManager->addEdge(VerticesIteratorTem_.second, VertexTem2_, EdgeLengthTem_);
      }
    }
  }
  return true;
}

bool PathPlanner::GenerateLocalGraphManagerForTargetSelect(std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                                           Eigen::Vector3d StartPosition3D,
                                                           Eigen::Vector3d ExpandVector,
                                                           std::shared_ptr<VRGraphManager> TargetVRGraphManager)
{
  // TargetVRGraphManager->ResetWithoutDelete();
  TargetVRGraphManager->ResetWithoutDeleteAndIgnoreIdPool();
  Eigen::Vector3d CenterPositionTem_ = StartPosition3D;
  double DistanceXTem_ = ExpandVector[0];
  double DistanceYTem_ = ExpandVector[1];
  double DistanceZTem_ = ExpandVector[2];
  StateVec StateTem_(CenterPositionTem_[0], CenterPositionTem_[1], CenterPositionTem_[2], 0);
  std::vector<Vertex*> VerticesTem_;
  if (!SourceVRGraphManager->getNearestVerticesInBox(&StateTem_, DistanceXTem_, DistanceYTem_, DistanceZTem_,
                                                     &VerticesTem_))
  {
    ROS_ERROR("[PathPlanner_Info]: Can not extract vertices when generate local graph for selecting target.");
    return false;
  }
  std::set<int> IdsInLocalGraphTem_;
  IdsInLocalGraphTem_.clear();
  std::unordered_map<int, std::unordered_map<int, double>> MapCopyTem_;
  SourceVRGraphManager->getUndirectedMap(MapCopyTem_);
  for (size_t i = 0; i < VerticesTem_.size(); i++)
  {
    if (VerticesTem_[i]->id == 0)
    {
      Vertex* Vertedx0Tem_ = TargetVRGraphManager->getVertex(0);
      Vertedx0Tem_->state = VerticesTem_[i]->state;
      Vertedx0Tem_->is_obstacle = VerticesTem_[i]->is_obstacle;
      Vertedx0Tem_->resolution = VerticesTem_[i]->resolution;
      IdsInLocalGraphTem_.insert(0);
    }
    else
    {
      TargetVRGraphManager->addVertex(VerticesTem_[i]);
      IdsInLocalGraphTem_.insert(VerticesTem_[i]->id);
    }

    for (auto IteratorTem_ : MapCopyTem_[VerticesTem_[i]->id])
    {
      int IdToBeConnectedTem_ = IteratorTem_.first;
      if ((IdsInLocalGraphTem_.find(IdToBeConnectedTem_) != IdsInLocalGraphTem_.end()) &&
          (IdToBeConnectedTem_ != VerticesTem_[i]->id))
      {
        Vertex* VertexTem2_ = TargetVRGraphManager->getVertex(IdToBeConnectedTem_);
        // Eigen::Vector3d EdgeTem_(VertexTem2_->state[0] - VerticesTem_[i]->state[0],
        //                          VertexTem2_->state[1] - VerticesTem_[i]->state[1],
        //                          VertexTem2_->state[2] - VerticesTem_[i]->state[2]);
        // double EdgeLengthTem_ = EdgeTem_.norm();
        double EdgeLengthTem_ = IteratorTem_.second;
        TargetVRGraphManager->addEdge(VerticesTem_[i], VertexTem2_, EdgeLengthTem_);
      }
    }
  }
  return true;
}

bool PathPlanner::GenerateLocalGraphManagerForTargetSelect(std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                                           std::string WorldFrame, std::string RobotFrame,
                                                           Eigen::Vector3d ExpandVector,
                                                           std::shared_ptr<VRGraphManager> TargetVRGraphManager)
{
  tf::TransformListener world_base_listener;
  tf::StampedTransform world_base_transform;
  try
  {
    world_base_listener.waitForTransform(WorldFrame.c_str(), RobotFrame.c_str(), ros::Time(0), ros::Duration(1.0));
    world_base_listener.lookupTransform(WorldFrame.c_str(), RobotFrame.c_str(), ros::Time(0), world_base_transform);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    return false;
    // ros::Duration(1.0).sleep();
  }

  Eigen::Vector3d RobotPosition3DTem_(world_base_transform.getOrigin().x(), world_base_transform.getOrigin().y(),
                                      world_base_transform.getOrigin().z() - RobotHeight_);
  return GenerateLocalGraphManagerForTargetSelect(SourceVRGraphManager, RobotPosition3DTem_, ExpandVector,
                                                  TargetVRGraphManager);
}

bool PathPlanner::DeepCopyVRGrahManager(std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                        std::shared_ptr<VRGraphManager> TargetVRGraphManager)
{
  // std::cout << "SourceVRGraphManager usecount is " << SourceVRGraphManager.use_count() << std::endl;
  // TargetVRGraphManager->Reset();
  // auto GraphPtrTem_ = SourceVRGraphManager->Graph_.get();
  // VRGraphManager::GraphType GraphCopyTem_ = *GraphPtrTem_;
  // TargetVRGraphManager->iKdTreePtr_ = std::make_shared<KD_TREE<PointType>>(*(SourceVRGraphManager->iKdTreePtr_));
  TargetVRGraphManager->Graph_ = std::make_shared<VRGraphManager::GraphType>(*(SourceVRGraphManager->Graph_));
  TargetVRGraphManager->IdPool_ = std::make_shared<IdPool>(*(SourceVRGraphManager->IdPool_));
  TargetVRGraphManager->ObstaclesSet_ = SourceVRGraphManager->ObstaclesSet_;
  if (TargetVRGraphManager->vertices_map_.size() > 0)
  {
    for (auto IteratorTargettem_ : TargetVRGraphManager->vertices_map_)
    {
      delete IteratorTargettem_.second;
    }
    TargetVRGraphManager->vertices_map_.clear();
  }

  PointVector PointVectorTem_;
  PointVectorTem_.clear();
  for (auto IteratorVertices : SourceVRGraphManager->vertices_map_)
  {
    assert(IteratorVertices.first == IteratorVertices.second->id);

    if ((IteratorVertices.first == 0) && false)
    {
      TargetVRGraphManager->vertices_map_[0]->state = IteratorVertices.second->state;
      TargetVRGraphManager->vertices_map_[0]->is_obstacle = IteratorVertices.second->is_obstacle;
      TargetVRGraphManager->vertices_map_[0]->resolution = IteratorVertices.second->resolution;
      PointType TreePointTem_;
      TreePointTem_.x = IteratorVertices.second->state.x();
      TreePointTem_.y = IteratorVertices.second->state.y();
      TreePointTem_.z = IteratorVertices.second->state.z();
      TreePointTem_.id = IteratorVertices.second->id;
      PointVectorTem_.push_back(TreePointTem_);
    }
    else
    {
      Vertex* VertexPtrTem_ = new Vertex(IteratorVertices.second->id, IteratorVertices.second->state);
      VertexPtrTem_->is_obstacle = IteratorVertices.second->is_obstacle;
      VertexPtrTem_->resolution = IteratorVertices.second->resolution;
      TargetVRGraphManager->vertices_map_[IteratorVertices.first] = VertexPtrTem_;
      PointType TreePointTem_;
      TreePointTem_.x = IteratorVertices.second->state.x();
      TreePointTem_.y = IteratorVertices.second->state.y();
      TreePointTem_.z = IteratorVertices.second->state.z();
      TreePointTem_.id = IteratorVertices.second->id;
      PointVectorTem_.push_back(TreePointTem_);
    }
  }
  // TargetVRGraphManager->iKdTreePtr_.reset(new KD_TREE<PointType>(0.3, 0.6, 0.1));
  // TargetVRGraphManager->iKdTreePtr_->Build(PointVectorTem_);

  TargetVRGraphManager->iKdTreePtr_->Build(PointVectorTem_);
  return true;
}

bool PathPlanner::DeepCopyVRGrahManager(std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                        std::shared_ptr<VRGraphManager> TargetVRGraphManager, bool IfRebuildTree)
{
  if (IfRebuildTree)
  {
    TargetVRGraphManager->Graph_ = std::make_shared<VRGraphManager::GraphType>(*(SourceVRGraphManager->Graph_));
    TargetVRGraphManager->IdPool_ = std::make_shared<IdPool>(*(SourceVRGraphManager->IdPool_));
    TargetVRGraphManager->ObstaclesSet_ = SourceVRGraphManager->ObstaclesSet_;
    if (TargetVRGraphManager->vertices_map_.size() > 0)
    {
      for (auto IteratorTargettem_ : TargetVRGraphManager->vertices_map_)
      {
        delete IteratorTargettem_.second;
      }
      TargetVRGraphManager->vertices_map_.clear();
    }

    PointVector PointVectorTem_;
    PointVectorTem_.clear();
    for (auto IteratorVertices : SourceVRGraphManager->vertices_map_)
    {
      assert(IteratorVertices.first == IteratorVertices.second->id);

      if ((IteratorVertices.first == 0) && false)
      {
        TargetVRGraphManager->vertices_map_[0]->state = IteratorVertices.second->state;
        TargetVRGraphManager->vertices_map_[0]->is_obstacle = IteratorVertices.second->is_obstacle;
        TargetVRGraphManager->vertices_map_[0]->resolution = IteratorVertices.second->resolution;
        PointType TreePointTem_;
        TreePointTem_.x = IteratorVertices.second->state.x();
        TreePointTem_.y = IteratorVertices.second->state.y();
        TreePointTem_.z = IteratorVertices.second->state.z();
        TreePointTem_.id = IteratorVertices.second->id;
        PointVectorTem_.push_back(TreePointTem_);
      }
      else
      {
        Vertex* VertexPtrTem_ = new Vertex(IteratorVertices.second->id, IteratorVertices.second->state);
        VertexPtrTem_->is_obstacle = IteratorVertices.second->is_obstacle;
        VertexPtrTem_->resolution = IteratorVertices.second->resolution;
        TargetVRGraphManager->vertices_map_[IteratorVertices.first] = VertexPtrTem_;
        PointType TreePointTem_;
        TreePointTem_.x = IteratorVertices.second->state.x();
        TreePointTem_.y = IteratorVertices.second->state.y();
        TreePointTem_.z = IteratorVertices.second->state.z();
        TreePointTem_.id = IteratorVertices.second->id;
        PointVectorTem_.push_back(TreePointTem_);
      }
    }
    TargetVRGraphManager->iKdTreePtr_->Build(PointVectorTem_);
  }
  else
  {
    TargetVRGraphManager->Graph_ = std::make_shared<VRGraphManager::GraphType>(*(SourceVRGraphManager->Graph_));
    TargetVRGraphManager->IdPool_ = std::make_shared<IdPool>(*(SourceVRGraphManager->IdPool_));
    TargetVRGraphManager->ObstaclesSet_ = SourceVRGraphManager->ObstaclesSet_;
    if (TargetVRGraphManager->vertices_map_.size() > 0)
    {
      for (auto IteratorTargettem_ : TargetVRGraphManager->vertices_map_)
      {
        delete IteratorTargettem_.second;
      }
      TargetVRGraphManager->vertices_map_.clear();
    }

    for (auto IteratorVertices : SourceVRGraphManager->vertices_map_)
    {
      assert(IteratorVertices.first == IteratorVertices.second->id);

      if ((IteratorVertices.first == 0) && false)
      {
        TargetVRGraphManager->vertices_map_[0]->state = IteratorVertices.second->state;
        TargetVRGraphManager->vertices_map_[0]->is_obstacle = IteratorVertices.second->is_obstacle;
        TargetVRGraphManager->vertices_map_[0]->resolution = IteratorVertices.second->resolution;
      }
      else
      {
        Vertex* VertexPtrTem_ = new Vertex(IteratorVertices.second->id, IteratorVertices.second->state);
        VertexPtrTem_->is_obstacle = IteratorVertices.second->is_obstacle;
        VertexPtrTem_->resolution = IteratorVertices.second->resolution;
        TargetVRGraphManager->vertices_map_[IteratorVertices.first] = VertexPtrTem_;
      }
    }
    TargetVRGraphManager->iKdTreePtr_.reset(new KD_TREE<PointType>(0.3, 0.6, 0.1));
  }

  return true;
}

double PathPlanner::ComputeExplorationGain(double Size, double LambdaSize, double Distance, double LambdaDistance,
                                           double ObstaclesCount, double LambdaObstacle, double ZDistance,
                                           double LambdaZDistance)
{
  double SizeRevisedTem_;
  if (Size > 5.0)
  {
    SizeRevisedTem_ = 5.0;
  }
  else
  {
    SizeRevisedTem_ = Size;
  }
  return LambdaSize * SizeRevisedTem_ * exp((-LambdaDistance) * Distance) * exp((-LambdaObstacle) * ObstaclesCount) *
         exp((-LambdaZDistance) * ZDistance);
}

bool PathPlanner::TargetPointSelectByExplorationGain(std::shared_ptr<VRGraphManager> SourceGlobalVRGraphManager,
                                                     std::unordered_map<int, Eigen::Vector4d> IdAndPositionAndSize,
                                                     std::unordered_map<int, double> IdAndDistance,
                                                     std::set<int> NotReachableIds, Eigen::Vector3d RobotPosition,
                                                     double LambdaSize, double LambdaDistance, double LambdaObstacle,
                                                     double LambdaZDistance, Eigen::Vector3d& OutputTargetPosition)
{
  Eigen::Vector3d RobotStandPosition3DTem_(RobotPosition[0], RobotPosition[1], RobotPosition[2] - RobotHeight_);
  Eigen::Vector3d OutputTargetPosition3DTem_;
  double ExplorationGainMaxValueTem_ = -1;
  bool IfFindTem_ = false;
  // std::cout << "Size of IdAndPositionAndSize is " << IdAndPositionAndSize.size() << std::endl;
  // for (auto IteratorCentersTem_ : IdAndPositionAndSize)
  // {
  //     std::cout << "Id is " << IteratorCentersTem_.first << " and center is " << std::endl
  //               << IteratorCentersTem_.second << std::endl;
  // }

  for (auto IteratorTem_ : IdAndPositionAndSize)
  {
    double ObstaclesCountTem_ = 0;
    double ExplorationGaintem_ = ComputeExplorationGain(
        IteratorTem_.second[3], LambdaSize, IdAndDistance[IteratorTem_.first], LambdaDistance, ObstaclesCountTem_,
        LambdaObstacle, abs(IteratorTem_.second[2] - RobotStandPosition3DTem_[2]), LambdaZDistance);

    // ROS_INFO("[PathPlanner_Info]: Potential target position is %f, %f, %f and corresponding exploration gain is %f",
    //          IteratorTem_.second[0],
    //          IteratorTem_.second[1],
    //          IteratorTem_.second[2],
    //          ExplorationGaintem_);
    if (ExplorationGaintem_ > ExplorationGainMaxValueTem_)
    {
      ExplorationGainMaxValueTem_ = ExplorationGaintem_;
      OutputTargetPosition3DTem_ << IteratorTem_.second[0], IteratorTem_.second[1], IteratorTem_.second[2];
      if (!IfFindTem_)
      {
        IfFindTem_ = true;
      }
    }
  }
  if (IfFindTem_)
  {
    OutputTargetPosition = OutputTargetPosition3DTem_;
    // ROS_INFO("[PathPlanner_Info]: Position of selected target is %f, %f, %f and corresponding exploration gain is
    // %f",
    //          OutputTargetPosition[0],
    //          OutputTargetPosition[1],
    //          OutputTargetPosition[2],
    //          ExplorationGainMaxValueTem_);
    return true;
  }
  else
  {
    return false;
  }
  return false;
}

bool PathPlanner::TargetPointSelectByExplorationGain(std::shared_ptr<VRGraphManager> SourceGlobalVRGraphManager,
                                                     std::unordered_map<int, Eigen::Vector4d> IdAndPositionAndSize,
                                                     std::unordered_map<int, double> IdAndDistance,
                                                     std::set<int> NotReachableIds, std::string WorldFrame,
                                                     std::string RobotFrame, double LambdaSize, double LambdaDistance,
                                                     double LambdaObstacle, double LambdaZDistance,
                                                     Eigen::Vector3d& OutputTargetPosition)
{
  tf::TransformListener world_base_listener;
  tf::StampedTransform world_base_transform;
  try
  {
    world_base_listener.waitForTransform(WorldFrame.c_str(), RobotFrame.c_str(), ros::Time(0), ros::Duration(1.0));
    world_base_listener.lookupTransform(WorldFrame.c_str(), RobotFrame.c_str(), ros::Time(0), world_base_transform);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    return false;
    // ros::Duration(1.0).sleep();
  }

  Eigen::Vector3d RobotPositionTem_(world_base_transform.getOrigin().x(), world_base_transform.getOrigin().y(),
                                    world_base_transform.getOrigin().z());
  return TargetPointSelectByExplorationGain(SourceGlobalVRGraphManager, IdAndPositionAndSize, IdAndDistance,
                                            NotReachableIds, RobotPositionTem_, LambdaSize, LambdaDistance,
                                            LambdaObstacle, LambdaZDistance, OutputTargetPosition);
}