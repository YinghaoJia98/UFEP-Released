#ifndef VRMAPPING_H_
#define VRMAPPING_H_

#include <ros/ros.h>
#include <vrmapping/VRGraphManager.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <vrmapping/partition_helper.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <vrmapping/FrontierExtract.h>
#include <vrmapping/PathPlanner.h>
#include <vrmapping_msgs/VarmappingPubTarget.h>
#include <vrmapping/PathTracker.h>
#include <vrmapping/Sampler.h>
#include <std_srvs/Trigger.h>
#include <mutex>
#include <thread>
#include <chrono>
#include <vrmapping/Integrator/Integrator.h>

class VRMap
{
public:
    VRMap(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    void Initialize();
    void ReadParameters();
    void ReadSamplerParameters(std::shared_ptr<Sampler> TargetSampler);
    void RegisterSubscribers();
    void RegisterTimers();
    void RegisterServers();
    void IterateElevationMap(grid_map::GridMap SourceElevationMap,
                             std::shared_ptr<VRGraphManager> LocalVariableResolutionGraphManager,
                             std::shared_ptr<VRGraphManager> TargetGlobalVariableResolutionGraphManager,
                             bool IfResetTargetGlobalVariableResolutionGraphManager);
    void IterElevationMapAndIntegrate(grid_map::GridMap SourceElevationMap);
    bool IfVertexExistInGraphManager(StateVec state_new,
                                     std::vector<Vertex *> *nearest_vertices,
                                     std::shared_ptr<VRGraphManager> SourceVRGraphManager,
                                     double MapResolution);

    void RebuildVertexWithObstacleGridsAndNormalGrids(int GlobalIdInMultiResolutionMap,
                                                      std::vector<PlannerGrid> ObstacleGridsNeededByRebuild,
                                                      std::vector<PlannerGrid> NormalGridsNeededByRebuild,
                                                      std::shared_ptr<VRGraphManager> GlobalGraphForMultiResolutionMap,
                                                      std::vector<PlannerCell> &CellsGenerated_);
    void GenerateCellsByPositionAndResolution(std::vector<PlannerCell> &CellsOutPut,
                                              Eigen::Vector3d CenterPosition,
                                              double resolution,
                                              double CellResolution,
                                              bool IfFillWithTraversableGrid);
    void AddGridsToGraphManager(std::vector<PlannerGrid> Grids,
                                std::shared_ptr<VRGraphManager> GlobalGraphForMultiResolutionMap,
                                bool IfTraversable);

    void VisualizeVRGraphManager(std::shared_ptr<VRGraphManager> GraphManager,
                                 std::shared_ptr<ros::Publisher> TopicPublisher,
                                 string ns_);
    void VisualizeVRGraphManager(std::shared_ptr<VRGraphManager> GraphManager,
                                 std::shared_ptr<ros::Publisher> TopicPublisher,
                                 string ns_,
                                 bool IfPubEdge,
                                 std::shared_ptr<ros::Publisher> EdgeTopicPublisher);

    void PlanToTarget3DPosition(Eigen::Vector3d TargetPosition3D,
                                std::shared_ptr<VRGraphManager> SourceGlobalVariableResolutionGraphManager,
                                std::shared_ptr<VRGraphManager> SourceLocalVariableResolutionGraphManager,
                                std::shared_ptr<PathPlanner> SourcePathPlanner,
                                int MaxTimeOfExpandGraph,
                                std::shared_ptr<Ppath_Tracker> SourcePathTracker);

    void PlanToTarget3DPosition(Eigen::Vector3d TargetPosition3D,
                                std::shared_ptr<VRGraphManager> SourceGlobalVariableResolutionGraphManager,
                                std::shared_ptr<VRGraphManager> SourceLocalVariableResolutionGraphManager,
                                std::shared_ptr<PathPlanner> SourcePathPlanner,
                                int MaxTimeOfExpandGraph,
                                std::shared_ptr<Ppath_Tracker> SourcePathTracker,
                                bool IfReplacePotentialFiledByResolution,
                                double MinResolutionValue,
                                bool IfExplorationMode,
                                double MinDistance);

    void UpdateTargetWithoutSampleGraph();

    void SetupIntegrator();
    bool CheckCudaDevice();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    std::shared_ptr<VRGraphManager> LVRGM_;
    std::shared_ptr<VRGraphManager> GVRGM_;    // Global Variable Resolution Graph Manager.
    std::shared_ptr<VRGraphManager> GFGVVRGM_; // Global For Graph Visualize Variable Resolution Graph Manager.

    // frontier start
    std::shared_ptr<FrontierExtract> FrontierExtractor_;
    std::shared_ptr<ros::Publisher> FSPublisher_;     // Frontiers Set Publisher
    std::shared_ptr<ros::Publisher> SGOFPublisher_;   // Sub Grpahs Of Frontiers Publisher
    std::shared_ptr<ros::Publisher> COSGOFPublisher_; // Center Of Sub Grpahs Of Frontiers Publisher
    std::shared_ptr<VRGraphManager> GFFEVRGM_;        // Global For Frontier Extractor Variable Resolution Graph Manager.
    // frontier end

    // pathplanner start
    ros::Timer UpdateTargetTimer_;
    void UpdateTargetTimerCallback(const ros::TimerEvent &event);
    std::shared_ptr<PathPlanner> PathPlanner_;
    std::shared_ptr<ros::Publisher> PathPublisher_;
    std::string RobotFrame_;
    double AttractiveGain_;
    double RepulsiveGain_;
    double BoundRadiu_;
    std::shared_ptr<VRGraphManager> LFPPVRGM_;           // Local For Path Planner Variable Resolution Graph Manager.
    std::shared_ptr<VRGraphManager> GFPPVRGM_;           // Global For Path Planner Variable Resolution Graph Manager.
    std::shared_ptr<ros::Publisher> GFPPVRGMPublisher_;  // Global For Path Planner Variable Resolution Graph Manager Publisher
    std::shared_ptr<ros::Publisher> GFPPVRGMEPublisher_; // Global For Path Planner Variable Resolution Graph Manager Edges Publisher
    double ExpandGraphX_;
    double ExpandGraphY_;
    double ExpandGraphZ_;
    int ExpandGraphTime_;
    bool ICTEA_;                        // If Contrain The Exploration Area
    Eigen::Matrix<double, 3, 2> COTEA_; // Constrains Of The Exploration Area
    bool IfExplore_;
    double ExpandSampledGraphForTargetSelectionX_;
    double ExpandSampledGraphForTargetSelectionY_;
    double ExpandSampledGraphForTargetSelectionZ_;
    int MaxExpandSampledGraphTime_;
    bool IfReplacePotentialFiledByResolution_;
    double MinTraResolution_;
    // pathplanner end

    // path tracker start
    std::shared_ptr<Ppath_Tracker> GlobalPathTracker_;
    // path tracker end

    // Sampler start
    std::shared_ptr<Sampler> GlobalSampler_;
    std::shared_ptr<ros::Publisher> GSGMPublisher_;  // Global Sampler Graph Manager Publisher
    std::shared_ptr<ros::Publisher> GSGMEPublisher_; // Global Sampler Graph Manager Edges Publisher
    ros::Timer SampleTimer_;
    void SampleTimerCallback(const ros::TimerEvent &event);
    ros::Timer SampleAddRobotStateTimer_;
    void SampleAddRobotStateTimerCallback(const ros::TimerEvent &event);
    // Sampler end

    std::string ElevationMapTopic_;
    ros::Subscriber ElevationMapSubscriber_;
    void ElevationMap_Callback(const grid_map_msgs::GridMap &msg);
    grid_map_msgs::GridMap ElevationMapMsg_;
    grid_map::GridMap ElevationMap_;
    bool IfElevationMapInitialized_;
    double ElevationMapResolution_;

    std::string ElevationLayer_, TraversabilityLayer_, TraversabilitySupplementaryLayer_;
    std::string WorldFrame_;

    std::shared_ptr<ros::Publisher> GVRGMPublisher_;  // Global Variable Resolution Graph Manager.
    std::shared_ptr<ros::Publisher> GVRGMEPublisher_; // Global Variable Resolution Graph Manager Edge.

    double TraversabilityThreshold_;

    double VRMapResolutionMax_;
    double VRMapResolutionMin_;

    double LimitBoxZ_;
    double LimitBoxZForConnectEdge_;
    double ValidThresholdForMultiMapResolution_;

    bool IfUpdateMapByMultiThreads_;
    std::shared_ptr<IntegratorNS::IntegratorBase> IntegratorPtr_;

    ros::Timer IterateElevationMapTimer_;
    void IterateElevationMapTimerCallback(const ros::TimerEvent &event);

    ros::Timer VisualizeGraphTimer_;
    void VisualizeGraphTimerCallback(const ros::TimerEvent &event);

    ros::Timer ExtractFrontierTimer_;
    void ExtractFrontierTimerCallback(const ros::TimerEvent &event);

    ros::ServiceServer PlanningByTargetPositonServer_;
    bool PlanningByTargetPositonCallback(vrmapping_msgs::VarmappingPubTarget::Request &req,
                                         vrmapping_msgs::VarmappingPubTarget::Response &res);

    ros::ServiceServer InitializationServer_;
    bool InitializationCallback(std_srvs::Trigger::Request &req,
                                std_srvs::Trigger::Response &res);

    ros::ServiceServer StartExplorationServer_;
    bool StartExplorationCallback(std_srvs::Trigger::Request &req,
                                  std_srvs::Trigger::Response &res);

    ros::ServiceServer StopExplorationServer_;
    bool StopExplorationCallback(std_srvs::Trigger::Request &req,
                                 std_srvs::Trigger::Response &res);

    mutable std::mutex ElevationMapMutex_;
    mutable std::mutex GVRGMMutex_;
    mutable std::mutex EFMutex_;   // Extract Frontier Mutex.
    mutable std::mutex PPMutex_;   // Path Planner Mutex.
    mutable std::mutex GSMutex_;   // Global Sampler Mutex.
    mutable std::mutex TOUTMutex_; // Timer Of Update Target Mutex.
};

#endif /* VRMAPPING_H_ */
