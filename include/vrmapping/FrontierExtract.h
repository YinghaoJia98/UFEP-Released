#ifndef FRONTIEREXTRACT_H_
#define FRONTIEREXTRACT_H_
#include <grid_map_ros/grid_map_ros.hpp>
#include <vrmapping/VRGraphManager.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vrmapping/union_find.h>

class FrontierExtract
{

public:
    FrontierExtract();
    ~FrontierExtract();
    void ExtractFrontiers(grid_map::GridMap SourceElevationMap,
                          std::string ElevationLayer,
                          std::string TraversabilityLayer,
                          std::string TraversabilitySupplementLayer,
                          std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                          double ElevationMapResolution,
                          double TraversabilityThreshold,
                          double VRMapResolutionMax,
                          double LimitBoxZ,
                          std::set<Eigen::Vector3d, setcomp> &FrontiersSet);
    void ExtractFrontiers(grid_map::GridMap SourceElevationMap,
                          std::string ElevationLayer,
                          std::string TraversabilityLayer,
                          std::string TraversabilitySupplementLayer,
                          std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                          double ElevationMapResolution,
                          double TraversabilityThreshold,
                          double VRMapResolutionMax,
                          double VRMapResolutionMin,
                          double LimitBoxZ,
                          std::set<Eigen::Vector3d, setcomp> &FrontiersSet);
    void ExtractFrontiers(grid_map::GridMap SourceElevationMap,
                          std::string ElevationLayer,
                          std::string TraversabilityLayer,
                          std::string TraversabilitySupplementLayer,
                          std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                          double ElevationMapResolution,
                          double TraversabilityThreshold,
                          double VRMapResolutionMax,
                          double VRMapResolutionMin,
                          double LimitBoxZ,
                          std::set<int> &FrontiersIdSet);
    void ExtractFrontiers(grid_map::GridMap SourceElevationMap,
                          std::string ElevationLayer,
                          std::string FrontierLayer,
                          std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                          double VRMapResolutionMax,
                          double VRMapResolutionMin,
                          double LimitBoxZ,
                          std::set<Eigen::Vector3d, setcomp> &FrontiersSet);
    void ExtractFrontiers(grid_map::GridMap SourceElevationMap,
                          std::string ElevationLayer,
                          std::string FrontierLayer,
                          std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                          double VRMapResolutionMax,
                          double VRMapResolutionMin,
                          double LimitBoxZ,
                          std::set<int> &FrontiersIdSet);
    bool IfPointInternal(double x, double y,
                         grid_map::GridMap SourceElevationMap,
                         double ElevationMapResolution,
                         std::string ElevationLayer,
                         std::string TraversabilityLayer,
                         std::string TraversabilitySupplementLayer);

    bool IfVertexInternal(std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                          Eigen::Vector3d VertexPosition,
                          double VRMapResolutionMax,
                          double LimitBoxZ,
                          double NeighboursCountThreshold);
    bool IfVertexInternalByGraph(std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                 Eigen::Vector3d VertexPosition,
                                 double VRMapResolutionMax,
                                 double VRMapResolutionMin,
                                 double LimitBoxZ);
    bool IfVertexInternalByGraph(std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                 Eigen::Vector3d VertexPosition,
                                 double VertexResolution,
                                 double VRMapResolutionMax,
                                 double VRMapResolutionMin,
                                 double LimitBoxZ);
    bool IfVertexInternalByGraph(std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                 int VertexId,
                                 double VertexResolution,
                                 double VRMapResolutionMax,
                                 double VRMapResolutionMin,
                                 double LimitBoxZ);
    void UpdateFrontiersSet(std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                            double VRMapResolutionMax,
                            double LimitBoxZ,
                            double NeighboursCountThreshold,
                            std::set<Eigen::Vector3d, setcomp> &FrontiersSet);
    void UpdateFrontiersSetByGraph(std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                   double VRMapResolutionMax,
                                   double VRMapResolutionMin,
                                   double LimitBoxZ,
                                   bool IfConstrainTheFrontierArea,
                                   Eigen::Vector3d CenterPosition,
                                   Eigen::Matrix<double, 3, 2> ConstrainsOfFrontiers,
                                   std::set<Eigen::Vector3d, setcomp> &FrontiersSet);
    void UpdateFrontiersSetByElevationMap(grid_map::GridMap SourceElevationMap,
                                          std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                          double VRMapResolutionMax,
                                          double LimitBoxZ,
                                          double NeighboursCountThreshold,
                                          double CellResolution,
                                          std::string ElevationLayer,
                                          std::string TraversabilityLayer,
                                          std::string TraversabilitySupplementLayer,
                                          std::set<Eigen::Vector3d, setcomp> &FrontiersSet);

    void VisualizeVertices(std::set<Eigen::Vector3d, setcomp> &FrontiersSet,
                           std::shared_ptr<ros::Publisher> TopicPublisher,
                           string ns_,
                           string Frame);

    void DistinguishFrontiers(std::set<Eigen::Vector3d, setcomp> &SourceFrontiersSet,
                              std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                              std::shared_ptr<UF> UFTem,
                              double VRMapResolutionMax,
                              double VRMapResolutionMin,
                              double LimitBoxZ,
                              double LimitBoxZForConnectEdge,
                              double ElevationMapResolution,
                              std::unordered_map<int, std::vector<int>> &SubGraphs,
                              std::unordered_map<int, double> &OutputFrontiers);
    void VisualizeSubGraphsOfFrontiers(std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                       std::shared_ptr<ros::Publisher> SGOFTopicPublisher,
                                       std::shared_ptr<ros::Publisher> TopicPublisher,
                                       std::unordered_map<int, std::vector<int>> &SourceSubGraphs,
                                       std::unordered_map<int, double> InputFrontiersAttribute,
                                       std::unordered_map<int, double> &OutputFrontiersAttribute,
                                       std::vector<Eigen::Vector4d> &OutputCenters,
                                       string Frame);

    bool get_central_point(std::vector<int> IdVector,
                           std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                           int &IdCenter);

    void GenerateCellsByPositionAndResolution(std::vector<Eigen::Vector2d> &CellsOutPut,
                                              Eigen::Vector3d CenterPosition,
                                              double resolution,
                                              double CellResolution);
    bool IfPointsInternal(std::vector<Eigen::Vector2d> Points,
                          grid_map::GridMap SourceElevationMap,
                          double ElevationMapResolution,
                          std::string ElevationLayer,
                          std::string TraversabilityLayer,
                          std::string TraversabilitySupplementLayer);

    void FilterSubGraphs(std::unordered_map<int, std::vector<int>> &TargetSubGraphs,
                         int MinSize);

    std::set<Eigen::Vector3d, setcomp> FrontiersSet_;
    std::set<int> FrontiersIdSet_;
    std::shared_ptr<UF> UF_;

    std::unordered_map<int, double> IdAndAttributeUpdatedToCenterId_;
    std::vector<Eigen::Vector4d> Centers_;

    grid_map::GridMap ElevationMap_;
    std::string FrontierLayer_;

private:
    // std::set<Eigen::Vector3d, setcomp> FrontiersSet_;
    // std::shared_ptr<UF> UF_;
};

#endif /* FRONTIEREXTRACT_H_ */
