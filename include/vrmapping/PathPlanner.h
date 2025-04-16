#ifndef PATHPLANNER_H_
#define PATHPLANNER_H_
#include <vrmapping/VRGraphManager.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
class PathPlanner
{
public:
  PathPlanner();
  ~PathPlanner();
  bool PlanPathByDijkstraMethod(std::shared_ptr<VRGraphManager> SourceVRGraphManager, int TargetId,
                                std::vector<int>& PathIdOutput, string WorldFrame, string RobotFrame);
  bool PlanPathByDijkstraMethod(std::shared_ptr<VRGraphManager> SourceVRGraphManager, int TargetId,
                                std::vector<int>& PathIdOutput, string WorldFrame, string RobotFrame,
                                VRGraphManager::GraphType UndirectedMap);

  bool PlanPathByDijkstraMethod(std::shared_ptr<VRGraphManager> SourceVRGraphManager, int TargetId,
                                std::vector<Eigen::Vector3d>& PathPositionOutput, string WorldFrame, string RobotFrame);

  bool PlanPathByDijkstraMethod(std::shared_ptr<VRGraphManager> SourceVRGraphManager, int TargetId,
                                std::vector<Eigen::Vector3d>& PathPositionOutput, string WorldFrame, string RobotFrame,
                                VRGraphManager::GraphType UndirectedMap);

  bool DijstraProcess(std::shared_ptr<VRGraphManager> SourceVRGraphManager, string WorldFrame, string RobotFrame,
                      std::unordered_map<int, int>& IdAndPrevious, std::unordered_map<int, double>& IdAndDistance);

  bool DijstraProcess(std::shared_ptr<VRGraphManager> SourceVRGraphManager, string WorldFrame, string RobotFrame,
                      std::unordered_map<int, int>& IdAndPrevious, std::unordered_map<int, double>& IdAndDistance,
                      VRGraphManager::GraphType UndirectedMap);

  bool GetDijkstraPathFromPrevious(std::shared_ptr<VRGraphManager> SourceVRGraphManager, int goal,
                                   std::unordered_map<int, int> IdAndPrevious, std::vector<int>& PathIdOutput);

  bool GetDijkstraPathFromPrevious(std::shared_ptr<VRGraphManager> SourceVRGraphManager, int goal,
                                   std::unordered_map<int, int> IdAndPrevious,
                                   std::vector<Eigen::Vector3d>& PathPositionOutput);

  void VisualizePath(std::vector<Eigen::Vector3d> PathVisualized, std::shared_ptr<ros::Publisher> TopicPublisher,
                     string ns_, string Frame);

  bool TargetPointSelect(std::unordered_map<int, double> IdAndGain, int& TargetId);

  bool TargetPointSelectByDistance(std::unordered_map<int, double> IdsPool,
                                   std::unordered_map<int, double> IdAndDistance, int& TargetId,
                                   std::set<int> NotReachableIds);

  bool TargetPointSelectByEuclideanDistance(std::vector<Eigen::Vector4d> SourceCenters, std::string WorldFrame,
                                            std::string BodyFrame, bool IfContrainTheExplorationArea,
                                            Eigen::Matrix<double, 3, 2> ConstrainsOfTheExplorationArea,
                                            std::unordered_map<VOXEL_LOC, int> NotReachableTargetPositions3D,
                                            Eigen::Vector3d& OutpuTargetPosition);

  void GenerateCostMap(const std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                       std::unordered_map<int, double>& OutputCostMap, const Eigen::Vector3d TargetPosition,
                       double AttractiveGain, double RepulsiveGain, double BoundRadiu,
                       const std::set<Eigen::Vector3d, setcomp> ObstaclesSet);

  void ComputeTotalForce(const Eigen::Vector3d RobotPosition, const Eigen::Vector3d TargetPosition,
                         const std::vector<Eigen::Vector3d> Obstacles, Eigen::Vector3d& Force, double AttractiveGain,
                         double RepulsiveGain, double BoundRadiu);

  void ComputeTotalForce(const Eigen::Vector3d RobotPosition, const Eigen::Vector3d TargetPosition,
                         const std::set<Eigen::Vector3d, setcomp> Obstacles, Eigen::Vector3d& Force,
                         double AttractiveGain, double RepulsiveGain, double BoundRadiu);

  void GenerateUndirectedMapByPotentialField(const std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                             VRGraphManager::GraphType& OuputMap, int TargetId, double AttractiveGain,
                                             double RepulsiveGain, double BoundRadiu,
                                             const std::set<Eigen::Vector3d, setcomp> ObstaclesSet);

  std::vector<int> aStar(std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                         VRGraphManager::GraphType SourceUnorderedMap, int start, int goal);
  double heuristic(std::shared_ptr<VRGraphManager> SourceVRGraphManager, int id1, int id2);

  bool PlanPathByAStarMethod(std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                             VRGraphManager::GraphType SourceUnorderedMap, int TargetId, std::vector<int>& PathIdOutput,
                             string WorldFrame, string RobotFrame);

  bool PlanPathByAStarMethod(std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                             VRGraphManager::GraphType SourceUnorderedMap, int TargetId,
                             std::vector<Eigen::Vector3d>& PathPositionOutput, string WorldFrame, string RobotFrame);

  bool GenerateLocalGraphManagerForPathPlanner(std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                               Eigen::Vector3d StartPosition3D, Eigen::Vector3d TargetPosition3D,
                                               Eigen::Vector3d ExpandVector, int TargetId, double AttractiveGain,
                                               double RepulsiveGain, double BoundRadiu,
                                               std::shared_ptr<VRGraphManager> TargetVRGraphManager,
                                               VRGraphManager::GraphType& OuputUndirectedMap);

  bool GenerateLocalGraphManagerForPathPlanner(std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                               Eigen::Vector3d TargetPosition3D, Eigen::Vector3d ExpandVector,
                                               int TargetId, double AttractiveGain, double RepulsiveGain,
                                               double BoundRadiu, string WorldFrame, string RobotFrame,
                                               std::shared_ptr<VRGraphManager> TargetVRGraphManager,
                                               VRGraphManager::GraphType& OuputUndirectedMap);

  bool GenerateLocalGraphManagerForPathPlanner(std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                               Eigen::Vector3d StartPosition3D, Eigen::Vector3d TargetPosition3D,
                                               Eigen::Vector3d ExpandVector, double ResolutionMin,
                                               double VRMapResolutionMax, double TargetLocalBoxZConstrain,
                                               std::shared_ptr<VRGraphManager> TargetVRGraphManager);

  bool GenerateLocalGraphManagerForPathPlanner(std::shared_ptr<VRGraphManager> SourceVRGraphManager, string WorldFrame,
                                               string RobotFrame, Eigen::Vector3d TargetPosition3D,
                                               Eigen::Vector3d ExpandVector, double ResolutionMin,
                                               double VRMapResolutionMax, double TargetLocalBoxZConstrain,
                                               std::shared_ptr<VRGraphManager> TargetVRGraphManager);

  bool GenerateLocalGraphManagerForPathPlanner(std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                               Eigen::Vector3d TargetPosition3D, double ResolutionMin,
                                               double VRMapResolutionMax, double TargetLocalBoxZConstrain,
                                               std::shared_ptr<VRGraphManager> TargetVRGraphManager);

  bool GenerateLocalGraphManagerForTargetSelect(std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                                Eigen::Vector3d StartPosition3D, Eigen::Vector3d ExpandVector,
                                                std::shared_ptr<VRGraphManager> TargetVRGraphManager);

  bool GenerateLocalGraphManagerForTargetSelect(std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                                std::string WorldFrame, std::string RobotFrame,
                                                Eigen::Vector3d ExpandVector,
                                                std::shared_ptr<VRGraphManager> TargetVRGraphManager);

  bool DeepCopyVRGrahManager(std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                             std::shared_ptr<VRGraphManager> TargetVRGraphManager);

  bool DeepCopyVRGrahManager(std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                             std::shared_ptr<VRGraphManager> TargetVRGraphManager, bool IfRebuildTree);

  double ComputeExplorationGain(double Size, double LambdaSize, double Distance, double LambdaDistance,
                                double ObstaclesCount, double LambdaObstacle, double ZDistance, double LambdaZDistance);

  bool TargetPointSelectByExplorationGain(std::shared_ptr<VRGraphManager> SourceGlobalVRGraphManager,
                                          std::unordered_map<int, Eigen::Vector4d> IdAndPositionAndSize,
                                          std::unordered_map<int, double> IdAndDistance, std::set<int> NotReachableIds,
                                          Eigen::Vector3d RobotPosition, double LambdaSize, double LambdaDistance,
                                          double LambdaObstacle, double LambdaZDistance,
                                          Eigen::Vector3d& OutputTargetPosition);

  bool TargetPointSelectByExplorationGain(std::shared_ptr<VRGraphManager> SourceGlobalVRGraphManager,
                                          std::unordered_map<int, Eigen::Vector4d> IdAndPositionAndSize,
                                          std::unordered_map<int, double> IdAndDistance, std::set<int> NotReachableIds,
                                          std::string WorldFrame, std::string RobotFrame, double LambdaSize,
                                          double LambdaDistance, double LambdaObstacle, double LambdaZDistance,
                                          Eigen::Vector3d& OutputTargetPosition);

  std::vector<Eigen::Vector3d> PathRBVPosition_;  // Path Represented By Point Vertex Position.
  std::vector<int> PathRBVId_;                    // Path Represented By Point Vertex Id.
  std::unordered_map<int, int> IdAndPrevious_;
  std::unordered_map<int, double> IdAndDistance_;
  int TargetId_;
  VRGraphManager::GraphType UndirectedMap_;

  bool IfPlanPathByAStar_;  // true for A-Star and false for Dijkstra.
  double RobotHeight_;

  double LambdaSize_;
  double LambdaDistance_;
  double LambdaObstacle_;
  double LambdaZDistance_;

  // avoid obstacles and delete the target from map.
  std::set<int> NotReachableFrontierIdsInSamplerGraph_;

  std::unordered_map<VOXEL_LOC, int> NotReachableTargetPositions3D_;

private:
};

#endif /* PATHPLANNER_H_ */
