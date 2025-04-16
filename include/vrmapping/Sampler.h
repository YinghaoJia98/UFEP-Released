#ifndef SAMPLER_H_
#define SAMPLER_H_
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <random>
#include <boost/math/constants/constants.hpp>
#include <vrmapping/VRGraphManager.h>
#include <tf/transform_listener.h>
class Sampler
{
public:
    Sampler();
    ~Sampler();
    bool SetGridMap(grid_map::GridMap ElevationMap);
    void samplePositionInMapFromDist(grid_map::GridMap SourceElevationMap,
                                     std::string ElevationLayer,
                                     std::string CumProbLayer,
                                     std::string CumProbRowwiseHackLayer,
                                     std::string NormalXLayer,
                                     std::string NormalYLayer,
                                     std::string NormalZLayer,
                                     Eigen::Vector3d &Position3DOutput,
                                     Eigen::Vector3d &NormalVectorOutput);
    void SampleUniform(grid_map::GridMap SourceElevationMap,
                       std::string ElevationLayer,
                       std::string CumProbLayer,
                       std::string CumProbRowwiseHackLayer,
                       std::string NormalXLayer,
                       std::string NormalYLayer,
                       std::string NormalZLayer,
                       Eigen::Vector3d &PositionSampledOutput);

    void SampleGraphManager(std::shared_ptr<VRGraphManager> TargetGraphManager,
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
                            double EdgeLengthMin);

    bool IfConnectToTheGraphManagerSucceed(std::shared_ptr<VRGraphManager> TargetGraphManager,
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
                                           int &EdgesCount);

    bool IfConnectTwoPointsSucceed(grid_map::GridMap SourceElevationMap,
                                   std::string ElevationLayer,
                                   std::string TraversabilityLayer,
                                   double RobotWidth,
                                   double TraversabilityThreshold,
                                   Eigen::Vector3d Point1,
                                   Eigen::Vector3d Point2);

    void InitializeAndProcessGridMap(grid_map::GridMap SourceElevationMap,
                                     grid_map::GridMap &ElevationMapOutput,
                                     std::string ElevationLayer,
                                     std::string TraversabilityLayer,
                                     double BlurRadius);

    void ProcessGridMapTraversabilityFilter(grid_map::GridMap &ElevationMapOutput,
                                            std::string ElevationLayer,
                                            std::string TraversabilityLayer);

    void ProcessGridMapSampleProbability(grid_map::GridMap &ElevationMapOutput,
                                         std::string TraversabilityLayer);

    bool IsValid(Eigen::Vector3d Position3D);
    bool BodyIsValid(Eigen::Vector3d Position3D);
    bool FootIsValid(Eigen::Vector3d Position3D);

    grid_map::Matrix gaussianBlurMatrix(const grid_map::Matrix &mat,
                                        int size,
                                        double std_dev);
    grid_map::Matrix inpaintMatrix(const grid_map::Matrix &mat);

    double uniformReal(double lower_bound, double upper_bound);
    void eulerRPY(Eigen::Vector3d &Value);

    void SampleGraphManager();

    void AddCurrentRobotStateToGraphManager(std::shared_ptr<VRGraphManager> TargetGraphManager,
                                            std::string WorldFrame,
                                            std::string RobotFrame);

    void AddCurrentRobotStateToGraphManager(std::shared_ptr<VRGraphManager> TargetGraphManager,
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
                                            int &EdgesCount);

    void ConvertPositionToIdAttribute(std::shared_ptr<VRGraphManager> SourceGraphManager,
                                      std::vector<Eigen::Vector4d> SourcePositionAttribute,
                                      double MaxDistanceToSourceGraphManager,
                                      bool IfContrainTheExplorationArea,
                                      Eigen::Matrix<double, 3, 2> ConstrainsOfTheExplorationArea,
                                      std::unordered_map<VOXEL_LOC, int> NotReachableTargetPositions3D,
                                      std::unordered_map<int, double> &OutputIdAttribute,
                                      std::unordered_map<int, Eigen::Vector4d> &OutputIdCenter);
    void ConvertPositionToIdAttribute(std::shared_ptr<VRGraphManager> SourceGraphManager,
                                      std::vector<Eigen::Vector4d> SourcePositionAttribute,
                                      double MaxDistanceToSourceGraphManager,
                                      std::unordered_map<int, double> &OutputIdAttribute,
                                      std::unordered_map<int, Eigen::Vector4d> &OutputIdCenter);

    void AddFrontierPointsToGraphManager(std::shared_ptr<VRGraphManager> TargetGraphManager,
                                         std::vector<Eigen::Vector4d> FrontierCenters,
                                         double MaxDistance);

    void AddFrontierPointsToGraphManager(std::shared_ptr<VRGraphManager> TargetGraphManager,
                                         std::vector<Eigen::Vector4d> FrontierCenters);

    void UpdateEdges(grid_map::GridMap SourceElevationMap,
                     std::string ElevationLayer,
                     std::string TraversabilityLayer,
                     double RobotWidth,
                     double TraversabilityThreshold,
                     double DistanceXMax,
                     double DistanceYMax,
                     double DistanceZMax,
                     int MaxCheckPoint,
                     std::shared_ptr<VRGraphManager> TargetGraphManager);

    void UpdateEdges();

    std::shared_ptr<VRGraphManager> GSOGM_; // Global Sampled Output Graph Manager
    std::shared_ptr<VRGraphManager> LSOGM_; // Local Sampled Output Graph Manager
    grid_map::GridMap ElevationMap_;
    std::string ElevationLayer_;
    std::string TraversabilityLayer_;
    std::string CumProbLayer_;
    std::string CumProbRowwiseHackLayer_;
    std::string NormalXLayer_;
    std::string NormalYLayer_;
    std::string NormalZLayer_;
    int NewSampledVerticesNumThreshold_;
    int ValidVerticesThreshold_;
    int ValidEdgesThreshold_;
    double SampleTimeThreshold_;
    double DistanceXMax_;
    double DistanceYMax_;
    double DistanceZMax_;
    int MaxCheckPoint_;
    double RobotWidth_;
    double TraversabilityThreshold_;
    int ConnectEdgesMax_;
    double EdgeLengthMin_;

    double RobotHeight_;

private:
    grid_map_msgs::GridMap ElevationMapMsg_;
    grid_map::GridMap ElevationMapProcessed_;
    double last_map_update_time_;
    std::uniform_real_distribution<> uniDist_{0, 1};
    std::mt19937 generator_;
};

#endif /* SAMPLER_H_ */
