#include <vrmapping/vrmapping.h>
VRMap::VRMap(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) : nh_(nh), nh_private_(nh_private)
{
  Initialize();
  ReadParameters();
  RegisterSubscribers();
  RegisterTimers();
  RegisterServers();
  ROS_INFO("Hello VRMap.");
  // LOG(INFO) << "I am INFO!";
}
void VRMap::Initialize()
{
  LVRGM_.reset(new VRGraphManager());
  GVRGM_.reset(new VRGraphManager());
  GFGVVRGM_.reset(new VRGraphManager());
  FrontierExtractor_.reset(new FrontierExtract());
  GFFEVRGM_.reset(new VRGraphManager());
  PathPlanner_.reset(new PathPlanner());
  LFPPVRGM_.reset(new VRGraphManager());
  GFPPVRGM_.reset(new VRGraphManager());

  IfElevationMapInitialized_ = false;
  ElevationMapResolution_ = 0;
  IfExplore_ = false;
  IfUpdateMapByMultiThreads_ = true;

  GVRGMPublisher_.reset(
      new ros::Publisher(nh_.advertise<visualization_msgs::MarkerArray>("vrmapping/GlobalGraph", 10000)));
  GVRGMEPublisher_.reset(
      new ros::Publisher(nh_.advertise<visualization_msgs::Marker>("vrmapping/GlobalGraphEdge", 10000)));

  FSPublisher_.reset(new ros::Publisher(nh_.advertise<visualization_msgs::Marker>("vrmapping/Frontier", 10000)));
  SGOFPublisher_.reset(
      new ros::Publisher(nh_.advertise<visualization_msgs::MarkerArray>("vrmapping/FrontierSubGraph", 10000)));
  COSGOFPublisher_.reset(
      new ros::Publisher(nh_.advertise<visualization_msgs::MarkerArray>("vrmapping/CentersOfFrontierSubGraph", 10000)));

  PathPublisher_.reset(new ros::Publisher(nh_.advertise<visualization_msgs::MarkerArray>("vrmapping/Path", 10000)));

  GlobalPathTracker_.reset(new Ppath_Tracker(nh_, nh_private_));

  GlobalSampler_.reset(new Sampler());
  GSGMPublisher_.reset(
      new ros::Publisher(nh_.advertise<visualization_msgs::MarkerArray>("vrmapping/SamplerGlobalGraph", 10000)));
  GSGMEPublisher_.reset(
      new ros::Publisher(nh_.advertise<visualization_msgs::Marker>("vrmapping/SamplerGlobalGraphManagerEdge", 10000)));

  GFPPVRGMPublisher_.reset(
      new ros::Publisher(nh_.advertise<visualization_msgs::MarkerArray>("vrmapping/GlobalPlannerGraph", 10000)));
  GFPPVRGMEPublisher_.reset(
      new ros::Publisher(nh_.advertise<visualization_msgs::Marker>("vrmapping/GlobalPlannerGraphEdge", 10000)));
}
void VRMap::ReadParameters()
{
  nh_.param<std::string>("/vrmapping/vrmapping_settings/ElevationLayer", ElevationLayer_, std::string("elevation"));
  nh_.param<std::string>("/vrmapping/vrmapping_settings/TraversabilityLayer", TraversabilityLayer_,
                         std::string("step"));
  nh_.param<std::string>("/vrmapping/vrmapping_settings/TraversabilitySupplementaryLayer",
                         TraversabilitySupplementaryLayer_, std::string("traversability"));

  nh_.param<double>("/vrmapping/vrmapping_settings/VRMapResolutionMax", VRMapResolutionMax_, 1.28);

  nh_.param<double>("/vrmapping/vrmapping_settings/VRMapResolutionMin", VRMapResolutionMin_, 0.16);

  nh_.param<double>("/vrmapping/vrmapping_settings/TraversabilityThreshold", TraversabilityThreshold_, 0.70);

  nh_.param<double>("/vrmapping/vrmapping_settings/LimitBoxZ", LimitBoxZ_, 0.5);

  nh_.param<double>("/vrmapping/vrmapping_settings/LimitBoxZForConnectEdge", LimitBoxZForConnectEdge_, 0.25);

  nh_.param<double>("/vrmapping/vrmapping_settings/ValidThresholdForMultiMapResolution",
                    ValidThresholdForMultiMapResolution_, 0.9);

  nh_.param<std::string>("/vrmapping/vrmapping_settings/WorldFrame", WorldFrame_, std::string("world"));

  nh_.param<std::string>("/vrmapping/vrmapping_settings/RobotFrame", RobotFrame_, std::string("base_link"));

  nh_.param<double>("/vrmapping/vrmapping_settings/AttractiveGain", AttractiveGain_, 0.0);

  nh_.param<double>("/vrmapping/vrmapping_settings/RepulsiveGain", RepulsiveGain_, 0.01);

  nh_.param<double>("/vrmapping/vrmapping_settings/BoundRadiu", BoundRadiu_, 0.6);

  nh_.param<double>("/vrmapping/vrmapping_settings/ExpandGraphX", ExpandGraphX_, 3.0);
  nh_.param<double>("/vrmapping/vrmapping_settings/ExpandGraphY", ExpandGraphY_, 3.0);
  nh_.param<double>("/vrmapping/vrmapping_settings/ExpandGraphZ", ExpandGraphZ_, 0.5);
  nh_.param<int>("/vrmapping/vrmapping_settings/ExpandGraphTime", ExpandGraphTime_, 3);
  nh_.param<bool>("/vrmapping/vrmapping_settings/IfContrainTheExplorationArea", ICTEA_, false);
  nh_.param<double>("/vrmapping/vrmapping_settings/ConstrainsOfTheExplorationAreaXMin", COTEA_(0, 0), 0.0);
  nh_.param<double>("/vrmapping/vrmapping_settings/ConstrainsOfTheExplorationAreaXMax", COTEA_(0, 1), 0.0);
  nh_.param<double>("/vrmapping/vrmapping_settings/ConstrainsOfTheExplorationAreaYMin", COTEA_(1, 0), 0.0);
  nh_.param<double>("/vrmapping/vrmapping_settings/ConstrainsOfTheExplorationAreaYMax", COTEA_(1, 1), 0.0);
  nh_.param<double>("/vrmapping/vrmapping_settings/ConstrainsOfTheExplorationAreaZMin", COTEA_(2, 0), 0.0);
  nh_.param<double>("/vrmapping/vrmapping_settings/ConstrainsOfTheExplorationAreaZMax", COTEA_(2, 1), 0.0);

  nh_.param<double>("/vrmapping/vrmapping_settings/ExpandSampledGraphForTargetSelectionX",
                    ExpandSampledGraphForTargetSelectionX_, 10.0);
  nh_.param<double>("/vrmapping/vrmapping_settings/ExpandSampledGraphForTargetSelectionY",
                    ExpandSampledGraphForTargetSelectionY_, 10.0);
  nh_.param<double>("/vrmapping/vrmapping_settings/ExpandSampledGraphForTargetSelectionZ",
                    ExpandSampledGraphForTargetSelectionZ_, 3.0);
  nh_.param<int>("/vrmapping/vrmapping_settings/MaxExpandSampledGraphTime", MaxExpandSampledGraphTime_, 3);

  nh_.param<bool>("/vrmapping/vrmapping_settings/IfReplacePotentialFiledByResolution",
                  IfReplacePotentialFiledByResolution_, true);
  nh_.param<double>("/vrmapping/vrmapping_settings/MinTraResolution", MinTraResolution_, 0.3);

  nh_.param<std::string>("/vrmapping/vrmapping_settings/FrontierLayer", FrontierExtractor_->FrontierLayer_,
                         std::string("frontiers"));

  nh_.param<bool>("/vrmapping/vrmapping_settings/IfPlanPathByAStar", PathPlanner_->IfPlanPathByAStar_, true);
  nh_.param<double>("/vrmapping/vrmapping_settings/RobotHeight", PathPlanner_->RobotHeight_, 0.235);
  nh_.param<double>("/vrmapping/vrmapping_settings/LambdaSize", PathPlanner_->LambdaSize_, 1000.11);
  nh_.param<double>("/vrmapping/vrmapping_settings/LambdaDistance", PathPlanner_->LambdaDistance_, 0.101666666);
  nh_.param<double>("/vrmapping/vrmapping_settings/LambdaObstacle", PathPlanner_->LambdaObstacle_, 0.0101);
  nh_.param<double>("/vrmapping/vrmapping_settings/LambdaZDistance", PathPlanner_->LambdaZDistance_, 6.1);
  ReadSamplerParameters(GlobalSampler_);

  nh_.param<bool>("/vrmapping/vrmapping_settings/IfUpdateMapByMultiThreads",
                  IfUpdateMapByMultiThreads_, true);

  if (IfUpdateMapByMultiThreads_)
  {
    SetupIntegrator();
  }
  else
  {
    IntegratorPtr_ = nullptr;
  }
}

void VRMap::ReadSamplerParameters(std::shared_ptr<Sampler> TargetSampler)
{
  TargetSampler->ElevationLayer_ = ElevationLayer_;
  TargetSampler->TraversabilityLayer_ = TraversabilityLayer_;
  nh_.param<std::string>("/vrmapping/sampler_settings/CumProbLayer", TargetSampler->CumProbLayer_,
                         std::string("cum_prob"));
  nh_.param<std::string>("/vrmapping/sampler_settings/CumProbRowwiseHackLayer", TargetSampler->CumProbRowwiseHackLayer_,
                         std::string("cum_prob_rowwise_hack"));
  nh_.param<std::string>("/vrmapping/sampler_settings/NormalXLayer", TargetSampler->NormalXLayer_,
                         std::string("normal_x"));
  nh_.param<std::string>("/vrmapping/sampler_settings/NormalYLayer", TargetSampler->NormalYLayer_,
                         std::string("normal_y"));
  nh_.param<std::string>("/vrmapping/sampler_settings/NormalZLayer", TargetSampler->NormalZLayer_,
                         std::string("normal_z"));
  nh_.param<int>("/vrmapping/sampler_settings/NewSampledVerticesNumThreshold",
                 TargetSampler->NewSampledVerticesNumThreshold_, 1000);
  nh_.param<int>("/vrmapping/sampler_settings/ValidVerticesThreshold", TargetSampler->ValidVerticesThreshold_, 100);
  nh_.param<int>("/vrmapping/sampler_settings/ValidEdgesThreshold", TargetSampler->ValidEdgesThreshold_, 500);
  nh_.param<double>("/vrmapping/sampler_settings/SampleTimeThreshold", TargetSampler->SampleTimeThreshold_, 1.0);
  nh_.param<double>("/vrmapping/sampler_settings/DistanceXMax", TargetSampler->DistanceXMax_, 2.0);
  nh_.param<double>("/vrmapping/sampler_settings/DistanceYMax", TargetSampler->DistanceYMax_, 2.0);
  nh_.param<double>("/vrmapping/sampler_settings/DistanceZMax", TargetSampler->DistanceZMax_, 0.50);
  nh_.param<int>("/vrmapping/sampler_settings/MaxCheckPoint", TargetSampler->MaxCheckPoint_, 5);
  nh_.param<double>("/vrmapping/sampler_settings/RobotWidth", TargetSampler->RobotWidth_, 0.5);
  nh_.param<double>("/vrmapping/sampler_settings/TraversabilityThreshold", TargetSampler->TraversabilityThreshold_,
                    0.6);
  nh_.param<int>("/vrmapping/sampler_settings/ConnectEdgesMax", TargetSampler->ConnectEdgesMax_, 5);
  nh_.param<double>("/vrmapping/sampler_settings/EdgeLengthMin", TargetSampler->EdgeLengthMin_, 0.6);

  nh_.param<double>("/vrmapping/vrmapping_settings/RobotHeight", TargetSampler->RobotHeight_, 0.235);
}

void VRMap::RegisterSubscribers()
{
  nh_.param<std::string>("/vrmapping/vrmapping_settings/ElevationMapTopic", ElevationMapTopic_,
                         std::string("/elevation_mapping/elevation_map_raw"));
  ElevationMapSubscriber_ = nh_.subscribe(ElevationMapTopic_.c_str(), 1, &VRMap::ElevationMap_Callback, this);
}
void VRMap::RegisterTimers()
{
  if (1)
  {
    double IteratorElevationMapFps_;
    nh_.param<double>("/vrmapping/vrmapping_settings/IteratorElevationMapFps", IteratorElevationMapFps_, 1.0);
    double duration = 1.0 / (IteratorElevationMapFps_ + 0.00001);
    IterateElevationMapTimer_ =
        nh_.createTimer(ros::Duration(duration), &VRMap::IterateElevationMapTimerCallback, this);
  }

  if (1)
  {
    double VisualizeFreshFps_;
    nh_.param<double>("/vrmapping/vrmapping_settings/VisualizeFreshFps", VisualizeFreshFps_, 1.0);
    double duration = 1.0 / (VisualizeFreshFps_ + 0.00001);
    VisualizeGraphTimer_ = nh_.createTimer(ros::Duration(duration), &VRMap::VisualizeGraphTimerCallback, this);
  }

  if (1)
  {
    double ExtractFrontierFps_;
    nh_.param<double>("/vrmapping/vrmapping_settings/ExtractFrontierFps", ExtractFrontierFps_, 1.0);
    double duration = 1.0 / (ExtractFrontierFps_ + 0.00001);
    ExtractFrontierTimer_ = nh_.createTimer(ros::Duration(duration), &VRMap::ExtractFrontierTimerCallback, this);
  }

  if (1)
  {
    double SampleFps_;
    nh_.param<double>("/vrmapping/sampler_settings/SampleFps", SampleFps_, 1.0);
    double duration = 1.0 / (SampleFps_ + 0.00001);
    SampleTimer_ = nh_.createTimer(ros::Duration(duration), &VRMap::SampleTimerCallback, this);
  }

  if (0)
  {
    double SampleAddRobotStateFps_;
    nh_.param<double>("/vrmapping/sampler_settings/SampleAddRobotStateFps", SampleAddRobotStateFps_, 1.0);
    double duration = 1.0 / (SampleAddRobotStateFps_ + 0.00001);
    SampleAddRobotStateTimer_ =
        nh_.createTimer(ros::Duration(duration), &VRMap::SampleAddRobotStateTimerCallback, this);
  }

  if (1)
  {
    double UpdateTargetFps_;
    nh_.param<double>("/vrmapping/vrmapping_settings/UpdateTargetFps", UpdateTargetFps_, 1.0);
    double duration = 1.0 / (UpdateTargetFps_ + 0.00001);
    UpdateTargetTimer_ = nh_.createTimer(ros::Duration(duration), &VRMap::UpdateTargetTimerCallback, this);
  }
}

void VRMap::RegisterServers()
{
  PlanningByTargetPositonServer_ =
      nh_.advertiseService("/VRMap/PlannerByPosition", &VRMap::PlanningByTargetPositonCallback, this);

  InitializationServer_ = nh_.advertiseService("/VRMap/Initialization", &VRMap::InitializationCallback, this);

  StartExplorationServer_ = nh_.advertiseService("/VRMap/StartExploration", &VRMap::StartExplorationCallback, this);

  StopExplorationServer_ = nh_.advertiseService("/VRMap/StopExploration", &VRMap::StopExplorationCallback, this);
}

void VRMap::ElevationMap_Callback(const grid_map_msgs::GridMap &msg)
{
  std::lock_guard<std::mutex> lock1(ElevationMapMutex_);
  ElevationMapMsg_ = msg;
  grid_map::GridMapRosConverter::fromMessage(ElevationMapMsg_, ElevationMap_);
  // GlobalSampler_->ElevationMap_ = ElevationMap_;

  if (ElevationMapResolution_ != ElevationMapMsg_.info.resolution)
  {
    ROS_ERROR("Traversability parameters ElevationMapResolution_ might be wrong and the lucky is that would be fixed");
    std::cout << "ElevationMapResolution_ is " << ElevationMapResolution_
              << " and the ElevationMapMsg_.info.resolution is " << ElevationMapMsg_.info.resolution << std::endl;
    ElevationMapResolution_ = ElevationMapMsg_.info.resolution;
    if (IfUpdateMapByMultiThreads_)
    {
      IntegratorPtr_->setElevationMapResolutionParam(ElevationMapResolution_);
    }
  }

  if (!IfElevationMapInitialized_)
  {
    IfElevationMapInitialized_ = true;
  }
}
void VRMap::IterateElevationMapTimerCallback(const ros::TimerEvent &event)
{
  // ROS_INFO("[iteratorMapTimer_INFO]:START.");
  if (!IfElevationMapInitialized_)
  {
    return;
  }
  std::lock_guard<std::mutex> lock2(GVRGMMutex_);
  // std::unique_lock<std::mutex> lock1(ElevationMapMutex_);
  grid_map::GridMap ElevationMapCopyForIteratorTem_ = ElevationMap_;
  // lock1.unlock();

  ROSTIME TStart_;
  START_TIME(TStart_);
  // ROS_INFO("ITERATOR CALLBACK");
  // ros::Time t_qian = ros::Time::now();
  // iterator_map(traversability_map);
  if (IfUpdateMapByMultiThreads_)
  {
    IterElevationMapAndIntegrate(ElevationMapCopyForIteratorTem_);
  }
  else
  {
    IterateElevationMap(ElevationMapCopyForIteratorTem_, LVRGM_, GVRGM_, false);
  }

  // IterateElevationMap(ElevationMapCopyForIteratorTem_, LVRGM_, GVRGM_, false);

  float TimeUsed1_ = GET_ELAPSED_TIME(TStart_);
  if (TimeUsed1_ > 0.0)
  {
    ROS_INFO("Time used for iterator map solely is %f", TimeUsed1_);
  }

  // float TimeUsed_ = GET_ELAPSED_TIME(TStart_);
  // if (TimeUsed_ > 0.5)
  // {
  //     ROS_INFO("Iterator map without map callback and time used for iterator map is %f", TimeUsed_);
  // }

  bool code_ceshi = false;
  if (code_ceshi)
  {
    //  test_fun();
  }

  // ROS_INFO("[iteratorMapTimer_INFO]:END.");
}

void VRMap::ExtractFrontierTimerCallback(const ros::TimerEvent &event)
{
  if (!IfElevationMapInitialized_)
  {
    return;
  }
  std::lock_guard<std::mutex> lock3(EFMutex_);
  ROSTIME TStart_;
  START_TIME(TStart_);
  // std::unique_lock<std::mutex> lock1(ElevationMapMutex_);
  FrontierExtractor_->ElevationMap_ = ElevationMap_;
  // lock1.unlock();
  std::unique_lock<std::mutex> lock2(GVRGMMutex_);
  // GVRGM_;
  if (!PathPlanner_->DeepCopyVRGrahManager(GVRGM_, GFFEVRGM_))
  {
    ROS_ERROR("[VRMapping_Info]: Deep Copy GVRGM_ To GFFEVRGM_ Faild.");
    return;
  }
  lock2.unlock();

  if (false)
  {
    FrontierExtractor_->ExtractFrontiers(FrontierExtractor_->ElevationMap_, ElevationLayer_, TraversabilityLayer_,
                                         TraversabilitySupplementaryLayer_, GFFEVRGM_, ElevationMapResolution_,
                                         TraversabilityThreshold_, VRMapResolutionMax_, VRMapResolutionMin_, LimitBoxZ_,
                                         FrontierExtractor_->FrontiersSet_);
  }
  else
  {
    FrontierExtractor_->ExtractFrontiers(FrontierExtractor_->ElevationMap_, ElevationLayer_,
                                         FrontierExtractor_->FrontierLayer_, GFFEVRGM_, VRMapResolutionMax_,
                                         VRMapResolutionMin_, LimitBoxZ_, FrontierExtractor_->FrontiersSet_);
  }

  // float TimeUsed1_ = GET_ELAPSED_TIME(TStart_);
  // ROS_INFO("TimeUsed1_ is %f", TimeUsed1_);

  // std::cout << "FrontiersSet_ size is " << FrontierExtractor_->FrontiersSet_.size() << std::endl;
  bool IfConstrainTheFrontierAreaTem_ = true;
  Eigen::Vector2d CenterPosition2DTem_ = FrontierExtractor_->ElevationMap_.getPosition();
  double ElevationValueTem_ = FrontierExtractor_->ElevationMap_.atPosition(ElevationLayer_, CenterPosition2DTem_);
  Eigen::Vector3d CenterPosition3DTem_(CenterPosition2DTem_[0], CenterPosition2DTem_[1], ElevationValueTem_);
  double CheckAreaXTem_ =
      (double)((int)((FrontierExtractor_->ElevationMap_.getLength().x() * 0.5) / VRMapResolutionMax_) + 1.5) *
      VRMapResolutionMax_;
  double CheckAreaYTem_ =
      (double)((int)((FrontierExtractor_->ElevationMap_.getLength().y() * 0.5) / VRMapResolutionMax_) + 1.5) *
      VRMapResolutionMax_;
  double CheckAreaZTem_ = 1.5 * LimitBoxZ_;
  Eigen::Matrix<double, 3, 2> ConstrainsOfFrontiersAreaTem_;
  ConstrainsOfFrontiersAreaTem_ << CheckAreaXTem_, CheckAreaXTem_, CheckAreaYTem_, CheckAreaYTem_, CheckAreaZTem_,
      CheckAreaZTem_;
  FrontierExtractor_->UpdateFrontiersSetByGraph(GFFEVRGM_, VRMapResolutionMax_, VRMapResolutionMin_, LimitBoxZ_,
                                                IfConstrainTheFrontierAreaTem_, CenterPosition3DTem_,
                                                ConstrainsOfFrontiersAreaTem_, FrontierExtractor_->FrontiersSet_);
  // float TimeUsed2_ = GET_ELAPSED_TIME(TStart_);
  // ROS_INFO("TimeUsed2_ is %f", TimeUsed2_);
  // float TimeUsed2_ = GET_ELAPSED_TIME(TStart_);

  string FNS_ = "Global";
  FrontierExtractor_->VisualizeVertices(FrontierExtractor_->FrontiersSet_, FSPublisher_, FNS_, WorldFrame_);
  // float TimeUsed3_ = GET_ELAPSED_TIME(TStart_);
  // ROS_INFO("TimeUsed3_ is %f", TimeUsed3_);

  std::unordered_map<int, std::vector<int>> SubGraphs_;
  SubGraphs_.clear();
  std::unordered_map<int, double> IdAndAttribute_;

  IdAndAttribute_.clear();
  FrontierExtractor_->DistinguishFrontiers(
      FrontierExtractor_->FrontiersSet_, GFFEVRGM_, FrontierExtractor_->UF_, VRMapResolutionMax_, VRMapResolutionMin_,
      LimitBoxZ_, LimitBoxZForConnectEdge_, ElevationMapResolution_, SubGraphs_, IdAndAttribute_);
  // float TimeUsed4_ = GET_ELAPSED_TIME(TStart_);
  // ROS_INFO("TimeUsed4_ is %f", TimeUsed4_);
  if (false)
  {
    FrontierExtractor_->FilterSubGraphs(SubGraphs_, 2);
  }

  FrontierExtractor_->IdAndAttributeUpdatedToCenterId_.clear();
  // TODO
  FrontierExtractor_->VisualizeSubGraphsOfFrontiers(
      GFFEVRGM_, SGOFPublisher_, COSGOFPublisher_, SubGraphs_, IdAndAttribute_,
      FrontierExtractor_->IdAndAttributeUpdatedToCenterId_, FrontierExtractor_->Centers_, WorldFrame_);
  // float TimeUsed5_ = GET_ELAPSED_TIME(TStart_);
  // ROS_INFO("TimeUsed5_ is %f", TimeUsed5_);

  float TimeUsed_ = GET_ELAPSED_TIME(TStart_);
  if (TimeUsed_ > 0.5)
  {
    ROS_INFO("Time used for extract frontier is %f", TimeUsed_);
  }
}

void VRMap::SampleTimerCallback(const ros::TimerEvent &event)
{
  if (!IfElevationMapInitialized_)
  {
    return;
  }
  std::lock_guard<std::mutex> lock5(GSMutex_);
  // std::unique_lock<std::mutex> lock1(ElevationMapMutex_);
  GlobalSampler_->ElevationMap_ = ElevationMap_;
  // lock1.unlock();
  // ROS_INFO("Hello, Sampler.");
  ROSTIME TStart_;
  START_TIME(TStart_);

  std::vector<Eigen::Vector4d> FrontiersCentersTem_;
  FrontiersCentersTem_ = FrontierExtractor_->Centers_;

  // ROS_INFO("Hello, Sampler1.");
  GlobalSampler_->SampleGraphManager();

  GlobalSampler_->AddCurrentRobotStateToGraphManager(GlobalSampler_->GSOGM_, WorldFrame_, RobotFrame_);

  GlobalSampler_->AddFrontierPointsToGraphManager(GlobalSampler_->GSOGM_, FrontiersCentersTem_);

  GlobalSampler_->UpdateEdges();

  // GlobalSampler_->AddFrontierPointsToGraphManager(GlobalSampler_->GSOGM_,
  //                                                 FrontiersCentersTem_,
  //                                                 GlobalSampler_->EdgeLengthMin_);
  // for (auto IteratorTem_ : GlobalSampler_->GSOGM_->vertices_map_)
  // {
  //     std::cout << "id is " << IteratorTem_.first << " and state is "
  //               << IteratorTem_.second->state[0] << " "
  //               << IteratorTem_.second->state[1] << " "
  //               << IteratorTem_.second->state[2] << " "
  //               << std::endl;
  // }

  // ROS_INFO("Hello, Sampler2.");
  std::string GSGMNS_ = "GlobalSampler";
  // TODO
  VisualizeVRGraphManager(GlobalSampler_->GSOGM_, GSGMPublisher_, GSGMNS_, true, GSGMEPublisher_);
  float TimeUsed1_ = GET_ELAPSED_TIME(TStart_);
  if (TimeUsed1_ > 0.5)
  {
    ROS_INFO("Time used for sampler is %f", TimeUsed1_);
  }
  // ROS_INFO("End, Sampler.");
}

void VRMap::SampleAddRobotStateTimerCallback(const ros::TimerEvent &event)
{
  std::lock_guard<std::mutex> lock5(GSMutex_);
  GlobalSampler_->AddCurrentRobotStateToGraphManager(GlobalSampler_->GSOGM_, WorldFrame_, RobotFrame_);
}

void VRMap::UpdateTargetTimerCallback(const ros::TimerEvent &event)
{
  std::lock_guard<std::mutex> lock6(TOUTMutex_);
  if (!IfExplore_)
  {
    return;
  }
  if ((GlobalPathTracker_->getStatus()) && (!GlobalPathTracker_->IfNeedToUpdateTarget()))
  {
    return;
  }

  // if (GlobalPathTracker_->getStatus())
  // {
  //     return;
  // }
  ros::TimerEvent TimerEventTem_;
  // usleep(50000);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  IterateElevationMapTimerCallback(TimerEventTem_);
  ExtractFrontierTimerCallback(TimerEventTem_);
  ExtractFrontierTimerCallback(TimerEventTem_);
  // ExtractFrontierTimerCallback(TimerEventTem_);
  SampleTimerCallback(TimerEventTem_);
  SampleTimerCallback(TimerEventTem_);

  ROSTIME TStart_;
  START_TIME(TStart_);
  std::unique_lock<std::mutex> lock2(GVRGMMutex_);
  ROSTIME TStartCopy_;
  START_TIME(TStartCopy_);

  if (!PathPlanner_->DeepCopyVRGrahManager(GVRGM_, GFPPVRGM_))
  {
    ROS_ERROR("[VRMapping_Info]: Deep Copy GVRGM_ To GFPPVRGM_ Faild.");
    return;
  }
  std::string GFPPVRGMNS_ = "GlobalPlanner";
  VisualizeVRGraphManager(GFPPVRGM_, GFPPVRGMPublisher_, GFPPVRGMNS_, true, GFPPVRGMEPublisher_);

  lock2.unlock();
  std::unordered_map<int, int> IdAndPreviousTem_;
  IdAndPreviousTem_.clear();
  std::unordered_map<int, double> IdAndDistanceTem_;
  IdAndDistanceTem_.clear();
  std::unordered_map<int, double> IdAndAttributeTem_;
  IdAndAttributeTem_.clear();
  std::unordered_map<int, Eigen::Vector4d> IdCenterTem_;
  IdCenterTem_.clear();
  bool IfProcessedTem_ = false;

  Eigen::Vector3d SampledGraphExpandVector3DTem_(0, 0, 0);

  bool IfUseGlobalSampledGraphTem_ = false;
  float TimeUsedTem_ = GET_ELAPSED_TIME(TStart_);
  ROS_INFO(" Time used for prepare select targetis %f", TimeUsedTem_);

  for (int ITem_ = 0; ITem_ < MaxExpandSampledGraphTime_; ITem_++)
  {
    // ROS_INFO("[VRMapping_Info]: The %d th expand Local Sampled Target Selection Graph.", ITem_);
    SampledGraphExpandVector3DTem_[0] = SampledGraphExpandVector3DTem_[0] + ExpandSampledGraphForTargetSelectionX_;
    SampledGraphExpandVector3DTem_[1] = SampledGraphExpandVector3DTem_[1] + ExpandSampledGraphForTargetSelectionY_;
    SampledGraphExpandVector3DTem_[2] = SampledGraphExpandVector3DTem_[2] + ExpandSampledGraphForTargetSelectionZ_;
    if (PathPlanner_->GenerateLocalGraphManagerForTargetSelect(GlobalSampler_->GSOGM_, WorldFrame_, RobotFrame_,
                                                               SampledGraphExpandVector3DTem_, GlobalSampler_->LSOGM_))
    {
      GlobalSampler_->ConvertPositionToIdAttribute(
          GlobalSampler_->LSOGM_, FrontierExtractor_->Centers_, GlobalSampler_->EdgeLengthMin_ + 0.05, ICTEA_, COTEA_,
          PathPlanner_->NotReachableTargetPositions3D_, IdAndAttributeTem_, IdCenterTem_);
      if (IdAndAttributeTem_.size() > 0)
      {
        break;
      }
    }
    else
    {
      continue;
    }
  }

  if (IdAndAttributeTem_.size() < 1)
  {
    ROS_ERROR(
        "[VRMapping_Info]: Can not find frontiers in local sampled graphs corresponding to frontiers in global graph "
        "by %d times expand.  And try to convert by global sampled graph.",
        MaxExpandSampledGraphTime_);
    GlobalSampler_->ConvertPositionToIdAttribute(
        GlobalSampler_->GSOGM_, FrontierExtractor_->Centers_, GlobalSampler_->EdgeLengthMin_ + 0.05, ICTEA_, COTEA_,
        PathPlanner_->NotReachableTargetPositions3D_, IdAndAttributeTem_, IdCenterTem_);
    IfUseGlobalSampledGraphTem_ = true;
  }

  TimeUsedTem_ = GET_ELAPSED_TIME(TStart_);
  ROS_INFO(" Time used for convert target into sample graph is %f", TimeUsedTem_);

  if ((IdAndAttributeTem_.size() < 1) || false)
  {
    ROS_ERROR(
        "[VRMapping_Info]: The Id Vector to be selected is empty and we choose to select the target nearest in "
        "Euclidean distance.");
    Eigen::Vector3d TargetPosition3DTem_;
    if (PathPlanner_->TargetPointSelectByEuclideanDistance(FrontierExtractor_->Centers_, WorldFrame_, RobotFrame_,
                                                           ICTEA_, COTEA_, PathPlanner_->NotReachableTargetPositions3D_,
                                                           TargetPosition3DTem_))
    {
      PlanToTarget3DPosition(TargetPosition3DTem_, GFPPVRGM_, LFPPVRGM_, PathPlanner_, ExpandGraphTime_,
                             GlobalPathTracker_, IfReplacePotentialFiledByResolution_, MinTraResolution_, true,
                             GlobalPathTracker_->GetGoalReachingThresholdValue());
    }
    else
    {
      ROS_ERROR("[VRMapping_Info]: Can not select target by Euclidean distance.");
    }
    // ROS_INFO("[VRMapping_Info]: Target 3D Position Selected is %f, %f, %f.", TargetPosition3DTem_[0],
    // TargetPosition3DTem_[1], TargetPosition3DTem_[2]);
  }
  else
  {
    if (IfUseGlobalSampledGraphTem_)
    {
      IfProcessedTem_ = PathPlanner_->DijstraProcess(GlobalSampler_->GSOGM_, WorldFrame_, RobotFrame_,
                                                     IdAndPreviousTem_, IdAndDistanceTem_);
    }
    else
    {
      IfProcessedTem_ = PathPlanner_->DijstraProcess(GlobalSampler_->LSOGM_, WorldFrame_, RobotFrame_,
                                                     IdAndPreviousTem_, IdAndDistanceTem_);
    }

    if (IfProcessedTem_)
    {
      if (true)
      {
        TimeUsedTem_ = GET_ELAPSED_TIME(TStart_);
        ROS_INFO(" Time used for dijkstra process sample graph is %f", TimeUsedTem_);
        // std::cout << "Size of IdCenterTem_ is " << IdCenterTem_.size() << std::endl;
        // for (auto IteratorCentersTem_ : IdCenterTem_)
        // {
        //     std::cout << "Id is " << IteratorCentersTem_.first << " and center is " << std::endl
        //               << IteratorCentersTem_.second << std::endl;
        // }
        Eigen::Vector3d TargetPosition3DTem_;
        if (PathPlanner_->TargetPointSelectByExplorationGain(
                GFPPVRGM_, IdCenterTem_, IdAndDistanceTem_, PathPlanner_->NotReachableFrontierIdsInSamplerGraph_,
                WorldFrame_, RobotFrame_, PathPlanner_->LambdaSize_, PathPlanner_->LambdaDistance_,
                PathPlanner_->LambdaObstacle_, PathPlanner_->LambdaZDistance_, TargetPosition3DTem_))
        {
          TimeUsedTem_ = GET_ELAPSED_TIME(TStart_);
          ROS_INFO(" Time used for select by exploration gain is %f", TimeUsedTem_);
          PlanToTarget3DPosition(TargetPosition3DTem_, GFPPVRGM_, LFPPVRGM_, PathPlanner_, ExpandGraphTime_,
                                 GlobalPathTracker_, IfReplacePotentialFiledByResolution_, MinTraResolution_, true,
                                 GlobalPathTracker_->GetGoalReachingThresholdValue());
          TimeUsedTem_ = GET_ELAPSED_TIME(TStart_);
          ROS_INFO(" Time used for plan path is %f", TimeUsedTem_);
        }
        else
        {
          ROS_ERROR(
              "[VRMapping_Info]: Can not get the target Id by select max exploration gain and we choose to select the "
              "target nearest in Euclidean distance.");
          if (PathPlanner_->TargetPointSelectByEuclideanDistance(
                  FrontierExtractor_->Centers_, WorldFrame_, RobotFrame_, ICTEA_, COTEA_,
                  PathPlanner_->NotReachableTargetPositions3D_, TargetPosition3DTem_))
          {
            PlanToTarget3DPosition(TargetPosition3DTem_, GFPPVRGM_, LFPPVRGM_, PathPlanner_, ExpandGraphTime_,
                                   GlobalPathTracker_, IfReplacePotentialFiledByResolution_, MinTraResolution_, true,
                                   GlobalPathTracker_->GetGoalReachingThresholdValue());
          }
          else
          {
            ROS_ERROR("[VRMapping_Info]: Can not select target by Euclidean distance.");
          }
        }
      }
      else
      {
        int TargetIdInSamplerGraphTem_ = 0;
        if (PathPlanner_->TargetPointSelectByDistance(IdAndAttributeTem_, IdAndDistanceTem_, TargetIdInSamplerGraphTem_,
                                                      PathPlanner_->NotReachableFrontierIdsInSamplerGraph_))
        {
          Eigen::Vector4d TargetCenterTem_ = IdCenterTem_[TargetIdInSamplerGraphTem_];
          Eigen::Vector3d TargetPosition3DTem_(TargetCenterTem_[0], TargetCenterTem_[1], TargetCenterTem_[2]);
          // ROS_INFO("[VRMapping_Info]: Target 3D Position Selected is %f, %f, %f.", TargetPosition3DTem_[0],
          // TargetPosition3DTem_[1], TargetPosition3DTem_[2]);
          PlanToTarget3DPosition(TargetPosition3DTem_, GFPPVRGM_, LFPPVRGM_, PathPlanner_, ExpandGraphTime_,
                                 GlobalPathTracker_, IfReplacePotentialFiledByResolution_, MinTraResolution_, true,
                                 GlobalPathTracker_->GetGoalReachingThresholdValue());
          if (PathPlanner_->PathRBVPosition_.size() < 1)
          {
            ROS_ERROR("[VRMapping_Info]: Can not generate path to target position %f, %f, %f in sample graph.",
                      TargetPosition3DTem_[0], TargetPosition3DTem_[1], TargetPosition3DTem_[2]);

            // PathPlanner_->NotReachableFrontierIdsInSamplerGraph_.insert(TargetIdInSamplerGraphTem_);
            return;
          }
        }
        else
        {
          ROS_ERROR(
              "[VRMapping_Info]: Can not get the target Id by select min distance and we choose to select the target "
              "nearest in Euclidean distance.");
          Eigen::Vector3d TargetPosition3DTem_;
          if (PathPlanner_->TargetPointSelectByEuclideanDistance(
                  FrontierExtractor_->Centers_, WorldFrame_, RobotFrame_, ICTEA_, COTEA_,
                  PathPlanner_->NotReachableTargetPositions3D_, TargetPosition3DTem_))
          {
            PlanToTarget3DPosition(TargetPosition3DTem_, GFPPVRGM_, LFPPVRGM_, PathPlanner_, ExpandGraphTime_,
                                   GlobalPathTracker_, IfReplacePotentialFiledByResolution_, MinTraResolution_, true,
                                   GlobalPathTracker_->GetGoalReachingThresholdValue());
          }
          else
          {
            ROS_ERROR("[VRMapping_Info]: Can not select target by Euclidean distance.");
          }
        }
      }
    }
    else
    {
      ROS_ERROR("[VRMapping_Info]: Use Dijstra method to process sampled graph failed.");
    }
  }
  float TimeUsed_ = GET_ELAPSED_TIME(TStart_);
  ROS_INFO(" Time used for select target and plan path is %f", TimeUsed_);
}

void VRMap::IterateElevationMap(grid_map::GridMap SourceElevationMap,
                                std::shared_ptr<VRGraphManager> LocalVariableResolutionGraphManager,
                                std::shared_ptr<VRGraphManager> TargetGlobalVariableResolutionGraphManager,
                                bool IfResetTargetGlobalVariableResolutionGraphManager)
{
  std::unordered_map<VOXEL_LOC, std::vector<PlannerCell>> LocalGridClusterVectorTem_;
  LocalGridClusterVectorTem_.clear();
  // if (LocalVariableResolutionGraphManager != NULL)
  // {
  //     LocalVariableResolutionGraphManager->Reset();
  // }
  if (IfResetTargetGlobalVariableResolutionGraphManager)
  {
    TargetGlobalVariableResolutionGraphManager->Reset();
  }

  ROSTIME StartTimeTem_;
  START_TIME(StartTimeTem_);

  for (grid_map::GridMapIterator iterator(SourceElevationMap); !iterator.isPastEnd(); ++iterator)
  {
    grid_map::Position GridPositionTem_;
    SourceElevationMap.getPosition(*iterator, GridPositionTem_);
    double elevation = SourceElevationMap.at(ElevationLayer_.c_str(), *iterator);
    double traversability_score = SourceElevationMap.at(TraversabilityLayer_.c_str(), *iterator);
    double traversability_supplementary_score =
        SourceElevationMap.at(TraversabilitySupplementaryLayer_.c_str(), *iterator);
    // remove the point whose value is Nan;
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
    // traversability_score = traversability_score - 1.0;
    //  if (traversability_score > traversability_score_threshold_local_graph)
    //  {
    int row;
    int col;
    // PPlannerMapResolutionMax_ pplaner_map_resolution
    if (GridPositionTem_[0] < -0.5 * VRMapResolutionMax_)
    {
      row = (GridPositionTem_[0] + VRMapResolutionMax_ / 2) / VRMapResolutionMax_ - 1;
    }
    else
    {
      row = (GridPositionTem_[0] + VRMapResolutionMax_ / 2) / VRMapResolutionMax_;
    }

    if (GridPositionTem_[1] < -0.5 * VRMapResolutionMax_)
    {
      col = (GridPositionTem_[1] + VRMapResolutionMax_ / 2) / VRMapResolutionMax_ - 1;
    }
    else
    {
      col = (GridPositionTem_[1] + VRMapResolutionMax_ / 2) / VRMapResolutionMax_;
    }

    // pplaner_map_resolution PPlannerMapResolutionMax_
    StateVec state_new(row * VRMapResolutionMax_, col * VRMapResolutionMax_, elevation, 0);
    PlannerCell PointSavedTem_;
    // X,Y,Z,Traversability.
    if (traversability_score < TraversabilityThreshold_)
    {
      PointSavedTem_ << GridPositionTem_[0], GridPositionTem_[1], state_new[2], 1;
    }
    else
    {
      PointSavedTem_ << GridPositionTem_[0], GridPositionTem_[1], state_new[2], 0;
    }

    VOXEL_LOC LocalPositionTem_(state_new[0], state_new[1], 0);

    LocalGridClusterVectorTem_[LocalPositionTem_].push_back(PointSavedTem_);
  }
  // float TimeUsedTem_ = GET_ELAPSED_TIME(StartTimeTem_);
  // ROS_INFO(" Time used for iterate local elevation map is %f", TimeUsedTem_);
  if ((LocalGridClusterVectorTem_.size() < 1) || false)
  {
    return;
  }

  for (auto &IterateGridVectorMapMultiResolutionMap__ : LocalGridClusterVectorTem_)
  {
    std::vector<PlannerGrid> GridsForMultiResolutionMap_;
    GridsForMultiResolutionMap_.clear();
    std::vector<PlannerGrid> ObstacleGridsForMultiResolutionMap_;
    ObstacleGridsForMultiResolutionMap_.clear();
    Eigen::Vector2d CurrentCenterPosition_(IterateGridVectorMapMultiResolutionMap__.first.x,
                                           IterateGridVectorMapMultiResolutionMap__.first.y);

    // ROSTIME StartTimeTem2_;
    // START_TIME(StartTimeTem2_);
    GridsForMultiResolutionMap_ =
        partition_helper::partitionGrid(IterateGridVectorMapMultiResolutionMap__.second, VRMapResolutionMax_,
                                        VRMapResolutionMin_, CurrentCenterPosition_, ElevationMapResolution_,
                                        ValidThresholdForMultiMapResolution_, ObstacleGridsForMultiResolutionMap_);
    // float TimeUsedTem2_ = GET_ELAPSED_TIME(StartTimeTem2_);
    // ROS_INFO(" Time used for partition once is %f", TimeUsedTem2_);
    std::vector<Vertex *> nearest_vertices_GlobalMultiResolutionMap;
    nearest_vertices_GlobalMultiResolutionMap.clear();
    // if (vertex_exist(state_new, &nearest_vertices))
    double zTem_ = 0.0;
    double zSumTem_ = 0.0;
    for (size_t i = 0; i < IterateGridVectorMapMultiResolutionMap__.second.size(); i++)
    {
      zSumTem_ = zSumTem_ + IterateGridVectorMapMultiResolutionMap__.second[i][2];
    }
    zTem_ = zSumTem_ / IterateGridVectorMapMultiResolutionMap__.second.size();
    StateVec state_in_LocalMultiResolutionMap(IterateGridVectorMapMultiResolutionMap__.first.x,
                                              IterateGridVectorMapMultiResolutionMap__.first.y, zTem_, 0);

    if (IfVertexExistInGraphManager(state_in_LocalMultiResolutionMap, &nearest_vertices_GlobalMultiResolutionMap,
                                    TargetGlobalVariableResolutionGraphManager, VRMapResolutionMax_))
    {
      // return;
      //  ROS_ERROR("ERROR FOR DEBUG");
      std::vector<PlannerCell> PlannerCellsAll_;
      PlannerCellsAll_.clear();
      PlannerCellsAll_.insert(PlannerCellsAll_.end(), IterateGridVectorMapMultiResolutionMap__.second.begin(),
                              IterateGridVectorMapMultiResolutionMap__.second.end());
      for (size_t id_GlobalMultiResolutionMap_middle = 0;
           id_GlobalMultiResolutionMap_middle < nearest_vertices_GlobalMultiResolutionMap.size();
           id_GlobalMultiResolutionMap_middle++)
      {
        std::vector<PlannerCell> PlannerCellsMiddle_;
        PlannerCellsMiddle_.clear();
        // ROSTIME StartTimeTem3_;
        // START_TIME(StartTimeTem3_);
        RebuildVertexWithObstacleGridsAndNormalGrids(
            nearest_vertices_GlobalMultiResolutionMap[id_GlobalMultiResolutionMap_middle]->id,
            ObstacleGridsForMultiResolutionMap_, GridsForMultiResolutionMap_,
            TargetGlobalVariableResolutionGraphManager, PlannerCellsMiddle_);
        // float TimeUsedTem3_ = GET_ELAPSED_TIME(StartTimeTem3_);
        // ROS_INFO(" Time used for rebuild vertex once is %f", TimeUsedTem3_);
        PlannerCellsAll_.insert(PlannerCellsAll_.end(), PlannerCellsMiddle_.begin(), PlannerCellsMiddle_.end());
      }
      // TargetGlobalVariableResolutionGraphManager->removeVertices(nearest_vertices_GlobalMultiResolutionMap);
      Eigen::Vector3d CenterPosition3D(IterateGridVectorMapMultiResolutionMap__.first.x,
                                       IterateGridVectorMapMultiResolutionMap__.first.y, zTem_);
      // std::cout << "CenterPosition3D is " << CenterPosition3D[0] << " " << CenterPosition3D[1] << " " <<
      // CenterPosition3D[2] << "." << std::endl;
      TargetGlobalVariableResolutionGraphManager->removeVerticesInBox(nearest_vertices_GlobalMultiResolutionMap,
                                                                      CenterPosition3D, VRMapResolutionMax_ / 2,
                                                                      VRMapResolutionMax_ / 2, LimitBoxZ_);
      nearest_vertices_GlobalMultiResolutionMap.clear();
      std::vector<PlannerGrid> GridsForMultiResolutionMapToGlobalGraph_;
      GridsForMultiResolutionMapToGlobalGraph_.clear();
      std::vector<PlannerGrid> ObstacleGridsForMultiResolutionMapToGlobalGraph_;
      ObstacleGridsForMultiResolutionMapToGlobalGraph_.clear();
      // ROSTIME StartTimeTem4_;
      // START_TIME(StartTimeTem4_);
      GridsForMultiResolutionMapToGlobalGraph_ = partition_helper::partitionGrid(
          PlannerCellsAll_, VRMapResolutionMax_, VRMapResolutionMin_, CurrentCenterPosition_, ElevationMapResolution_,
          ValidThresholdForMultiMapResolution_, ObstacleGridsForMultiResolutionMapToGlobalGraph_);
      // float TimeUsedTem4_ = GET_ELAPSED_TIME(StartTimeTem4_);
      // ROS_INFO(" Time used for later partition once is %f", TimeUsedTem4_);
      // ROSTIME StartTimeTem5_;
      // START_TIME(StartTimeTem5_);
      AddGridsToGraphManager(GridsForMultiResolutionMapToGlobalGraph_, TargetGlobalVariableResolutionGraphManager,
                             true);
      AddGridsToGraphManager(ObstacleGridsForMultiResolutionMapToGlobalGraph_,
                             TargetGlobalVariableResolutionGraphManager, false);
      // float TimeUsedTem5_ = GET_ELAPSED_TIME(StartTimeTem5_);
      // ROS_INFO(" Time used for add grids once is %f", TimeUsedTem5_);
    }
    else
    {
      // add new point in AllGrids one by one;
      AddGridsToGraphManager(GridsForMultiResolutionMap_, TargetGlobalVariableResolutionGraphManager, true);
      AddGridsToGraphManager(ObstacleGridsForMultiResolutionMap_, TargetGlobalVariableResolutionGraphManager, false);
    }
  }
}

void VRMap::IterElevationMapAndIntegrate(grid_map::GridMap SourceElevationMap)
{
  // ROSTIME StartTimeTem_;
  // START_TIME(StartTimeTem_);
  std::unordered_map<VOXEL_LOC, std::vector<PlannerCell>> LocalGridClusterVectorTem_;
  LocalGridClusterVectorTem_.clear();
  for (grid_map::GridMapIterator iterator(SourceElevationMap); !iterator.isPastEnd(); ++iterator)
  {
    grid_map::Position GridPositionTem_;
    SourceElevationMap.getPosition(*iterator, GridPositionTem_);
    double elevation = SourceElevationMap.at(ElevationLayer_.c_str(), *iterator);
    double traversability_score = SourceElevationMap.at(TraversabilityLayer_.c_str(), *iterator);
    double traversability_supplementary_score =
        SourceElevationMap.at(TraversabilitySupplementaryLayer_.c_str(), *iterator);
    // remove the point whose value is Nan;
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
    // traversability_score = traversability_score - 1.0;
    //  if (traversability_score > traversability_score_threshold_local_graph)
    //  {
    int row;
    int col;
    // PPlannerMapResolutionMax_ pplaner_map_resolution
    if (GridPositionTem_[0] < -0.5 * VRMapResolutionMax_)
    {
      row = (GridPositionTem_[0] + VRMapResolutionMax_ / 2) / VRMapResolutionMax_ - 1;
    }
    else
    {
      row = (GridPositionTem_[0] + VRMapResolutionMax_ / 2) / VRMapResolutionMax_;
    }

    if (GridPositionTem_[1] < -0.5 * VRMapResolutionMax_)
    {
      col = (GridPositionTem_[1] + VRMapResolutionMax_ / 2) / VRMapResolutionMax_ - 1;
    }
    else
    {
      col = (GridPositionTem_[1] + VRMapResolutionMax_ / 2) / VRMapResolutionMax_;
    }

    // pplaner_map_resolution PPlannerMapResolutionMax_
    StateVec state_new(row * VRMapResolutionMax_, col * VRMapResolutionMax_, elevation, 0);
    PlannerCell PointSavedTem_;
    // X,Y,Z,Traversability.
    if (traversability_score < TraversabilityThreshold_)
    {
      PointSavedTem_ << GridPositionTem_[0], GridPositionTem_[1], state_new[2], 1;
    }
    else
    {
      PointSavedTem_ << GridPositionTem_[0], GridPositionTem_[1], state_new[2], 0;
    }

    VOXEL_LOC LocalPositionTem_(state_new[0], state_new[1], 0);

    LocalGridClusterVectorTem_[LocalPositionTem_].push_back(PointSavedTem_);
  }

  if ((LocalGridClusterVectorTem_.size() < 1) || false)
  {
    return;
  }
  // float TimeUsedTem_ = GET_ELAPSED_TIME(StartTimeTem_);
  // ROS_INFO(" Time used for iterate local elevation map is %f", TimeUsedTem_);

  IntegratorPtr_->integrateGridCells(LocalGridClusterVectorTem_);
  // float TimeUsedTem2_ = GET_ELAPSED_TIME(StartTimeTem_);
  // ROS_INFO(" Time used for integrating grid cells is %f", TimeUsedTem2_ - TimeUsedTem_);
}

bool VRMap::IfVertexExistInGraphManager(StateVec state_new, std::vector<Vertex *> *nearest_vertices,
                                        std::shared_ptr<VRGraphManager> SourceVRGraphManager, double MapResolution)
{
  if (SourceVRGraphManager->getNearestVerticesInBox(&state_new, MapResolution / 2, MapResolution / 2, LimitBoxZ_,
                                                    nearest_vertices))
  {
    return true;
  }
  else
  {
    return false;
  }
}

void VRMap::RebuildVertexWithObstacleGridsAndNormalGrids(
    int GlobalIdInMultiResolutionMap, std::vector<PlannerGrid> ObstacleGridsNeededByRebuild,
    std::vector<PlannerGrid> NormalGridsNeededByRebuild,
    std::shared_ptr<VRGraphManager> GlobalGraphForMultiResolutionMap, std::vector<PlannerCell> &CellsGenerated_)
{
  Vertex *VertexNeedChanged_ = GlobalGraphForMultiResolutionMap->getVertex(GlobalIdInMultiResolutionMap);
  if (VertexNeedChanged_ == NULL)
  {
    ROS_ERROR("Vertex needing to be changed is NULL.");
    return;
  }
  // generate grid cells in the area of vertex.
  // std::vector<PlannerCell> CellsGenerated_;
  Eigen::Vector3d CenterPosition_(VertexNeedChanged_->state[0], VertexNeedChanged_->state[1],
                                  VertexNeedChanged_->state[2]);
  double VertexResolution_ = VertexNeedChanged_->resolution;
  bool IfTraversable_ = !VertexNeedChanged_->is_obstacle;
  GenerateCellsByPositionAndResolution(CellsGenerated_, CenterPosition_, VertexResolution_, ElevationMapResolution_,
                                       IfTraversable_);
  if (IfTraversable_)
  {
    // check obstacle.
    for (std::vector<PlannerCell>::iterator iterCellsVector_ = CellsGenerated_.begin();
         iterCellsVector_ != CellsGenerated_.end(); ++iterCellsVector_)
    {
      Eigen::Vector3d CellPositionMiddle_(iterCellsVector_->x(), iterCellsVector_->y(), iterCellsVector_->z());
      bool IfNeedSetToObstacle_ = false;
      for (std::vector<PlannerGrid>::iterator iterObstacleGridsVectorMiddle_ = ObstacleGridsNeededByRebuild.begin();
           iterObstacleGridsVectorMiddle_ != ObstacleGridsNeededByRebuild.end(); ++iterObstacleGridsVectorMiddle_)
      {
        Eigen::Vector3d GridPositionMiddle_(iterObstacleGridsVectorMiddle_->centerX,
                                            iterObstacleGridsVectorMiddle_->centerY,
                                            iterObstacleGridsVectorMiddle_->centerZ);
        Eigen::Vector3d DistanceVectorMiddle_ = GridPositionMiddle_ - CellPositionMiddle_;
        if (abs(DistanceVectorMiddle_[2]) > LimitBoxZForConnectEdge_)
        {
        }
        Eigen::Vector2d DistanceVectorMiddleFor2D_(DistanceVectorMiddle_[0], DistanceVectorMiddle_[1]);
        // double Distance2DMiddle_ = DistanceVectorMiddleFor2D_.norm();
        double GridSizeMiddle_ = (iterObstacleGridsVectorMiddle_->size) / 2;
        // if (Distance2DMiddle_ < GridSizeMiddle_)
        if ((abs(DistanceVectorMiddle_[0]) < (GridSizeMiddle_)) && (abs(DistanceVectorMiddle_[1]) < (GridSizeMiddle_)))
        {
          // iterCellsVector_->x() = GridPositionMiddle_[0];
          // iterCellsVector_->y() = GridPositionMiddle_[1];
          iterCellsVector_->z() = GridPositionMiddle_[2];
          IfNeedSetToObstacle_ = true;
          break;
        }
      }
      if (IfNeedSetToObstacle_)
      {
        iterCellsVector_->w() = 1;
        continue;
        // TODO Might need change to pointer.
      }
      for (std::vector<PlannerGrid>::iterator iterNormalGridsVectorMiddle_ = NormalGridsNeededByRebuild.begin();
           iterNormalGridsVectorMiddle_ != NormalGridsNeededByRebuild.end(); ++iterNormalGridsVectorMiddle_)
      {
        Eigen::Vector3d GridPositionMiddle_(iterNormalGridsVectorMiddle_->centerX,
                                            iterNormalGridsVectorMiddle_->centerY,
                                            iterNormalGridsVectorMiddle_->centerZ);
        Eigen::Vector3d DistanceVectorMiddle_ = GridPositionMiddle_ - CellPositionMiddle_;
        if (abs(DistanceVectorMiddle_[2]) > LimitBoxZForConnectEdge_)
        {
        }
        Eigen::Vector2d DistanceVectorMiddleFor2D_(DistanceVectorMiddle_[0], DistanceVectorMiddle_[1]);
        // double Distance2DMiddle_ = DistanceVectorMiddleFor2D_.norm();
        double GridSizeMiddle_ = (iterNormalGridsVectorMiddle_->size) / 2;
        // if (Distance2DMiddle_ < GridSizeMiddle_)
        if ((abs(DistanceVectorMiddle_[0]) < (GridSizeMiddle_)) && (abs(DistanceVectorMiddle_[1]) < (GridSizeMiddle_)))
        {
          // iterCellsVector_->x() = GridPositionMiddle_[0];
          // iterCellsVector_->y() = GridPositionMiddle_[1];
          iterCellsVector_->z() = GridPositionMiddle_[2];
          break;
        }
      }
    }
  }
  else
  {
    // check normal
    for (std::vector<PlannerCell>::iterator iterCellsVector_ = CellsGenerated_.begin();
         iterCellsVector_ != CellsGenerated_.end(); ++iterCellsVector_)
    {
      Eigen::Vector3d CellPositionMiddle_(iterCellsVector_->x(), iterCellsVector_->y(), iterCellsVector_->z());
      bool IfNeedSetToNormal_ = false;

      for (std::vector<PlannerGrid>::iterator iterNormalGridsVectorMiddle_ = NormalGridsNeededByRebuild.begin();
           iterNormalGridsVectorMiddle_ != NormalGridsNeededByRebuild.end(); ++iterNormalGridsVectorMiddle_)
      {
        Eigen::Vector3d GridPositionMiddle_(iterNormalGridsVectorMiddle_->centerX,
                                            iterNormalGridsVectorMiddle_->centerY,
                                            iterNormalGridsVectorMiddle_->centerZ);
        Eigen::Vector3d DistanceVectorMiddle_ = GridPositionMiddle_ - CellPositionMiddle_;
        if (abs(DistanceVectorMiddle_[2]) > LimitBoxZForConnectEdge_)
        {
        }
        Eigen::Vector2d DistanceVectorMiddleFor2D_(DistanceVectorMiddle_[0], DistanceVectorMiddle_[1]);
        // double Distance2DMiddle_ = DistanceVectorMiddleFor2D_.norm();
        double GridSizeMiddle_ = (iterNormalGridsVectorMiddle_->size) / 2;
        // if (Distance2DMiddle_ < GridSizeMiddle_)
        if ((abs(DistanceVectorMiddle_[0]) < (GridSizeMiddle_)) && (abs(DistanceVectorMiddle_[1]) < (GridSizeMiddle_)))
        {
          // iterCellsVector_->x() = GridPositionMiddle_[0];
          // iterCellsVector_->y() = GridPositionMiddle_[1];
          iterCellsVector_->z() = GridPositionMiddle_[2];
          IfNeedSetToNormal_ = true;
          break;
        }
      }
      if (IfNeedSetToNormal_)
      {
        iterCellsVector_->w() = 0;
        continue;
        // TODO Might need change to pointer.
      }

      for (std::vector<PlannerGrid>::iterator iterObstacleGridsVectorMiddle_ = ObstacleGridsNeededByRebuild.begin();
           iterObstacleGridsVectorMiddle_ != ObstacleGridsNeededByRebuild.end(); ++iterObstacleGridsVectorMiddle_)
      {
        Eigen::Vector3d GridPositionMiddle_(iterObstacleGridsVectorMiddle_->centerX,
                                            iterObstacleGridsVectorMiddle_->centerY,
                                            iterObstacleGridsVectorMiddle_->centerZ);
        Eigen::Vector3d DistanceVectorMiddle_ = GridPositionMiddle_ - CellPositionMiddle_;
        if (abs(DistanceVectorMiddle_[2]) > LimitBoxZForConnectEdge_)
        {
        }
        Eigen::Vector2d DistanceVectorMiddleFor2D_(DistanceVectorMiddle_[0], DistanceVectorMiddle_[1]);
        // double Distance2DMiddle_ = DistanceVectorMiddleFor2D_.norm();
        double GridSizeMiddle_ = (iterObstacleGridsVectorMiddle_->size) / 2;
        // if (Distance2DMiddle_ < GridSizeMiddle_)
        if ((abs(DistanceVectorMiddle_[0]) < (GridSizeMiddle_)) && (abs(DistanceVectorMiddle_[1]) < (GridSizeMiddle_)))
        {
          // iterCellsVector_->x() = GridPositionMiddle_[0];
          // iterCellsVector_->y() = GridPositionMiddle_[1];
          iterCellsVector_->z() = GridPositionMiddle_[2];
          break;
        }
      }
    }
  }
}

void VRMap::GenerateCellsByPositionAndResolution(std::vector<PlannerCell> &CellsOutPut, Eigen::Vector3d CenterPosition,
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

  // if (start_x < -0.5 * CellResolution)
  // {
  //     row_strat = (start_x + CellResolution / 2) / CellResolution - 1;
  // }
  // else
  // {
  //     row_strat = (start_x + CellResolution / 2) / CellResolution;
  // }

  // if (end_x < -0.5 * CellResolution)
  // {
  //     row_end = (end_x + CellResolution / 2) / CellResolution - 1;
  // }
  // else
  // {
  //     row_end = (end_x + CellResolution / 2) / CellResolution;
  // }
  // ////////////
  // if (start_y < -0.5 * CellResolution)
  // {
  //     col_start = (start_y + CellResolution / 2) / CellResolution - 1;
  // }
  // else
  // {
  //     col_start = (start_y + CellResolution / 2) / CellResolution;
  // }

  // if (end_y < -0.5 * CellResolution)
  // {
  //     col_end = (end_y + CellResolution / 2) / CellResolution - 1;
  // }
  // else
  // {
  //     col_end = (end_y + CellResolution / 2) / CellResolution;
  // }

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

  // std::cout << "//////////******INFO******//////////" << std::endl;
  // std::cout << "CenterPosition is " << CenterPosition[0] << " " << CenterPosition[1]
  //           << " " << CenterPosition[2] << std::endl
  //           << " Resolution is " << resolution << std::endl
  //           << " start x is " << start_x << " and end x is " << end_x << std::endl
  //           << " start y is " << start_y << " and end y is " << end_y << std::endl;

  for (int iRowMiddle_ = row_strat; iRowMiddle_ <= row_end; iRowMiddle_++)
  {
    for (int iColMiddle_ = col_start; iColMiddle_ <= col_end; iColMiddle_++)
    {
      // PlannerCell CellMiddle_((double)(iRowMiddle_ - 0.5) * CellResolution,
      //                         (double)(iColMiddle_ - 0.5) * CellResolution,
      //                         CenterPosition[2],
      //                         traversability);
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

void VRMap::AddGridsToGraphManager(std::vector<PlannerGrid> Grids,
                                   std::shared_ptr<VRGraphManager> GlobalGraphForMultiResolutionMap, bool IfTraversable)
{
  for (std::vector<PlannerGrid>::iterator iterGridsVector_ = Grids.begin(); iterGridsVector_ != Grids.end();
       ++iterGridsVector_)
  {
    StateVec StateNew_(iterGridsVector_->centerX, iterGridsVector_->centerY, iterGridsVector_->centerZ, 0);
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
          &StateNew_, VRMapResolutionMax_ + 0.5 * VRMapResolutionMin_, VRMapResolutionMax_ + 0.5 * VRMapResolutionMin_,
          LimitBoxZForConnectEdge_, &NearestVerticesForConnectingEdge_);
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
        if ((DFEIX <= (VNR_ + VOR_) / 2 + ElevationMapResolution_) &&
            (DFEIY <= (VNR_ + VOR_) / 2 + ElevationMapResolution_))
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

void VRMap::VisualizeGraphTimerCallback(const ros::TimerEvent &event)
{
  std::unique_lock<std::mutex> lock2(GVRGMMutex_);
  // std::lock_guard<std::mutex> lock3(EFMutex_);
  ROSTIME TStart_;
  START_TIME(TStart_);
  string GVRGMNS_ = "Global";
  if (GVRGM_.use_count() > 2)
  {
    ROS_ERROR("FBI Warning!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    std::cout << "////////////////////////When Visualize the count of GVRGM_ is " << GVRGM_.use_count() << std::endl;
  }

  // if (!PathPlanner_->DeepCopyVRGrahManager(GVRGM_, GFGVVRGM_))
  if (!PathPlanner_->DeepCopyVRGrahManager(GVRGM_, GFGVVRGM_, false))
  {
    ROS_ERROR("[VRMapping_Info]: Deep Copy GVRGM_ To GFGVVRGM_ Faild.");
    return;
  }
  lock2.unlock();

  VisualizeVRGraphManager(GFGVVRGM_, GVRGMPublisher_, GVRGMNS_, true, GVRGMEPublisher_);

  float TimeUsed_ = GET_ELAPSED_TIME(TStart_);
  if (TimeUsed_ > 0.5)
  {
    ROS_INFO("Time used for visualize map is %f", TimeUsed_);
  }
}

void VRMap::VisualizeVRGraphManager(std::shared_ptr<VRGraphManager> GraphManager,
                                    std::shared_ptr<ros::Publisher> TopicPublisher, string ns_)
{
  std::unordered_map<int, Vertex *> &v_map_visualize = GraphManager->vertices_map_;

  if (GraphManager->getNumVertices() == 0)
  {
    return;
  }
  if (TopicPublisher->getNumSubscribers() < 1)
  {
    return;
  }

  visualization_msgs::MarkerArray MarkerArrayPublished_;
  MarkerArrayPublished_.markers.clear();

  int normal_seq_middle_ = 0;
  int obstacle_seq_middle_ = 0;

  for (auto &IterateMap_ : GraphManager->vertices_map_)
  {
    int id = IterateMap_.first;
    geometry_msgs::Point p1;
    p1.x = v_map_visualize[id]->state[0];
    p1.y = v_map_visualize[id]->state[1];
    p1.z = v_map_visualize[id]->state[2];
    if (v_map_visualize[id]->is_obstacle)
    {
      std::string vertex_obstacle_str_ = "vertex_obstacle";
      visualization_msgs::Marker vertex_obstacles_marker;
      vertex_obstacles_marker.header.stamp = ros::Time::now();
      vertex_obstacles_marker.header.seq = obstacle_seq_middle_;
      vertex_obstacles_marker.header.frame_id = WorldFrame_.c_str();
      vertex_obstacles_marker.id = obstacle_seq_middle_;
      vertex_obstacles_marker.ns = (ns_ + vertex_obstacle_str_).c_str();
      vertex_obstacles_marker.action = visualization_msgs::Marker::ADD;
      vertex_obstacles_marker.type = visualization_msgs::Marker::CUBE;
      vertex_obstacles_marker.scale.x = v_map_visualize[id]->resolution;
      vertex_obstacles_marker.scale.y = v_map_visualize[id]->resolution;
      vertex_obstacles_marker.scale.z = 0.01;
      vertex_obstacles_marker.color.r = 255.0 / 255.0;
      vertex_obstacles_marker.color.g = 0.0 / 255.0;
      vertex_obstacles_marker.color.b = 255.0 / 255.0;
      vertex_obstacles_marker.color.a = 1.0;
      vertex_obstacles_marker.lifetime = ros::Duration(5.0);
      vertex_obstacles_marker.frame_locked = false;
      // vertex_obstacles_marker.points.push_back(p1);
      vertex_obstacles_marker.pose.position = p1;
      obstacle_seq_middle_++;
      MarkerArrayPublished_.markers.push_back(vertex_obstacles_marker);
    }
    else
    {
      std::string vertex_str_ = "vertex";
      visualization_msgs::Marker vertex_marker;
      // visualization_msgs::Marker vertex_marker;
      vertex_marker.header.stamp = ros::Time::now();
      vertex_marker.header.seq = normal_seq_middle_;
      vertex_marker.header.frame_id = WorldFrame_.c_str();
      vertex_marker.id = normal_seq_middle_;
      vertex_marker.ns = (ns_ + vertex_str_).c_str();
      vertex_marker.action = visualization_msgs::Marker::ADD;
      vertex_marker.type = visualization_msgs::Marker::CUBE;
      vertex_marker.scale.x = v_map_visualize[id]->resolution;
      vertex_marker.scale.y = v_map_visualize[id]->resolution;
      vertex_marker.scale.z = 0.01;
      vertex_marker.color.r = 0.0 / 255.0;
      vertex_marker.color.g = 255.0 / 255.0;
      vertex_marker.color.b = 0.0 / 255.0;
      vertex_marker.color.a = 1.0;
      vertex_marker.lifetime = ros::Duration(5.0);
      vertex_marker.frame_locked = false;
      // vertex_marker.points.push_back(p1);
      vertex_marker.pose.position = p1;
      normal_seq_middle_++;
      MarkerArrayPublished_.markers.push_back(vertex_marker);
    }
  }

  // Plot all edges
  std::string edge_str_ = "edge";
  visualization_msgs::Marker edge_marker;
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.header.seq = 1;
  edge_marker.header.frame_id = WorldFrame_.c_str();
  edge_marker.id = 1;
  edge_marker.ns = (ns_ + edge_str_).c_str();
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.scale.x = 0.02;
  edge_marker.color.r = 200.0 / 255.0;
  edge_marker.color.g = 100.0 / 255.0;
  edge_marker.color.b = 0.0;
  edge_marker.color.a = 1.0;
  edge_marker.lifetime = ros::Duration(10.0);
  edge_marker.frame_locked = false;

  std::unordered_map<int, std::unordered_map<int, double>> MapCopyTem_;
  GraphManager->getUndirectedMap(MapCopyTem_);
  std::set<int> IdVisited_;
  IdVisited_.clear();
  for (const auto &vertex1_ : MapCopyTem_)
  {
    int id1_ = vertex1_.first;
    int id2_;
    IdVisited_.insert(id1_);
    for (const auto &vertex2_ : vertex1_.second)
    {
      id2_ = vertex2_.first;
      if (IdVisited_.find(id2_) != IdVisited_.end())
      {
        continue;
      }
      geometry_msgs::Point p1;
      p1.x = v_map_visualize[id1_]->state[0];
      p1.y = v_map_visualize[id1_]->state[1];
      p1.z = v_map_visualize[id1_]->state[2];
      geometry_msgs::Point p2;
      p2.x = v_map_visualize[id2_]->state[0];
      p2.y = v_map_visualize[id2_]->state[1];
      p2.z = v_map_visualize[id2_]->state[2];
      edge_marker.points.push_back(p1);
      edge_marker.points.push_back(p2);
    }
  }

  if (edge_marker.points.size() < 1)
  {
  }
  else
  {
    MarkerArrayPublished_.markers.push_back(edge_marker);
  }
  TopicPublisher->publish(MarkerArrayPublished_);
}

void VRMap::VisualizeVRGraphManager(std::shared_ptr<VRGraphManager> GraphManager,
                                    std::shared_ptr<ros::Publisher> TopicPublisher, string ns_, bool IfPubEdge,
                                    std::shared_ptr<ros::Publisher> EdgeTopicPublisher)
{
  // std::unordered_map<int, Vertex *> &v_map_visualize = GraphManager->vertices_map_;

  if (GraphManager->getNumVertices() == 0)
  {
    return;
  }
  if ((TopicPublisher->getNumSubscribers() < 1) && (EdgeTopicPublisher->getNumSubscribers() < 1))
  {
    return;
  }

  visualization_msgs::MarkerArray MarkerArrayPublished_;
  MarkerArrayPublished_.markers.clear();

  // int normal_seq_middle_ = 0;
  // int obstacle_seq_middle_ = 0;

  int TheMaxZoomFactorTem_ = log(VRMapResolutionMax_ / VRMapResolutionMin_) / log(2);
  std::unordered_map<int, visualization_msgs::Marker> MakersMapTem_;
  MakersMapTem_.clear();
  for (int i = 0; i <= TheMaxZoomFactorTem_; i++)
  {
    std::string vertex_str_ = "vertex";
    visualization_msgs::Marker MarkerTem_;
    MarkerTem_.header.stamp = ros::Time::now();
    MarkerTem_.header.seq = 0;
    MarkerTem_.header.frame_id = WorldFrame_.c_str();
    MarkerTem_.id = i;
    MarkerTem_.ns = (ns_ + vertex_str_).c_str();
    MarkerTem_.action = visualization_msgs::Marker::ADD;
    // MarkerTem_.type = visualization_msgs::Marker::SPHERE_LIST;
    MarkerTem_.type = visualization_msgs::Marker::CUBE_LIST;
    MarkerTem_.scale.x = VRMapResolutionMax_ / (double)(pow(2.0, i));
    MarkerTem_.scale.y = VRMapResolutionMax_ / (double)(pow(2.0, i));
    MarkerTem_.scale.z = 0.1;
    MarkerTem_.color.r = 0.0 / 255.0;
    MarkerTem_.color.g = 255.0 / 255.0;
    MarkerTem_.color.b = 0.0;
    MarkerTem_.color.a = 1.0;
    MarkerTem_.lifetime = ros::Duration(0.0);
    MarkerTem_.frame_locked = false;
    MakersMapTem_[i] = MarkerTem_;
  }

  // std::unordered_map<int, visualization_msgs::Marker> ObstaclesMakersMapTem_;
  // ObstaclesMakersMapTem_.clear();
  // for (int i = 0; i <= TheMaxZoomFactorTem_; i++)
  // {
  //     std::string vertex_obstacle_str_ = "vertex_obstacle";
  //     visualization_msgs::Marker MarkerTem_;
  //     MarkerTem_.header.stamp = ros::Time::now();
  //     MarkerTem_.header.seq = 0;
  //     MarkerTem_.header.frame_id = WorldFrame_.c_str();
  //     MarkerTem_.id = i;
  //     MarkerTem_.ns = (ns_ + vertex_obstacle_str_).c_str();
  //     MarkerTem_.action = visualization_msgs::Marker::ADD;
  //     MarkerTem_.type = visualization_msgs::Marker::SPHERE_LIST;
  //     MarkerTem_.scale.x = VRMapResolutionMax_ / (double)(pow(2.0, i));
  //     MarkerTem_.scale.y = VRMapResolutionMax_ / (double)(pow(2.0, i));
  //     MarkerTem_.scale.z = 0.01;
  //     MarkerTem_.color.r = 255.0 / 255.0;
  //     MarkerTem_.color.g = 0.0 / 255.0;
  //     MarkerTem_.color.b = 255.0 / 255.0;
  //     MarkerTem_.color.a = 1.0;
  //     MarkerTem_.lifetime = ros::Duration(0.0);
  //     MarkerTem_.frame_locked = false;
  //     ObstaclesMakersMapTem_[i] = MarkerTem_;
  // }

  std::string vertex_obstacle_str_ = "vertex_obstacle";
  visualization_msgs::Marker ObstaclesMarkerTem_;
  ObstaclesMarkerTem_.header.stamp = ros::Time::now();
  ObstaclesMarkerTem_.header.seq = 0;
  ObstaclesMarkerTem_.header.frame_id = WorldFrame_.c_str();
  ObstaclesMarkerTem_.id = 3;
  ObstaclesMarkerTem_.ns = (ns_ + vertex_obstacle_str_).c_str();
  ObstaclesMarkerTem_.action = visualization_msgs::Marker::ADD;
  ObstaclesMarkerTem_.type = visualization_msgs::Marker::CUBE_LIST;
  ObstaclesMarkerTem_.scale.x = VRMapResolutionMin_;
  ObstaclesMarkerTem_.scale.y = VRMapResolutionMin_;
  ObstaclesMarkerTem_.scale.z = 0.01;
  ObstaclesMarkerTem_.color.r = 255.0 / 255.0;
  ObstaclesMarkerTem_.color.g = 0.0 / 255.0;
  ObstaclesMarkerTem_.color.b = 255.0 / 255.0;
  ObstaclesMarkerTem_.color.a = 1.0;
  ObstaclesMarkerTem_.lifetime = ros::Duration(0.0);
  ObstaclesMarkerTem_.frame_locked = false;

  for (auto &IterateMap_ : GraphManager->vertices_map_)
  {
    // int id = IterateMap_.first;
    geometry_msgs::Point p1;
    p1.x = IterateMap_.second->state[0];
    p1.y = IterateMap_.second->state[1];
    p1.z = IterateMap_.second->state[2];
    int ZoomFactorTem_ = log(VRMapResolutionMax_ / IterateMap_.second->resolution) / log(2);
    if (ZoomFactorTem_ > TheMaxZoomFactorTem_)
    {
      ROS_ERROR(
          "[VRMapping_Info]: There is something wrong as ZoomFactorTem_ is %d maxer than TheMaxZoomFactorTem_ is %d.",
          ZoomFactorTem_, TheMaxZoomFactorTem_);
      ROS_INFO("[VRMapping_Info]: IterateMap_.second->resolution is %f.", IterateMap_.second->resolution);
      continue;
    }
    if (IterateMap_.second->is_obstacle)
    {
      ObstaclesMarkerTem_.points.push_back(p1);
      // ObstaclesMakersMapTem_[ZoomFactorTem_].points.push_back(p1);
      //  std::string vertex_obstacle_str_ = "vertex_obstacle";
      //  visualization_msgs::Marker vertex_obstacles_marker;
      //  vertex_obstacles_marker.header.stamp = ros::Time::now();
      //  vertex_obstacles_marker.header.seq = obstacle_seq_middle_;
      //  vertex_obstacles_marker.header.frame_id = WorldFrame_.c_str();
      //  vertex_obstacles_marker.id = obstacle_seq_middle_;
      //  vertex_obstacles_marker.ns = (ns_ + vertex_obstacle_str_).c_str();
      //  vertex_obstacles_marker.action = visualization_msgs::Marker::ADD;
      //  vertex_obstacles_marker.type = visualization_msgs::Marker::SPHERE; // CUBE
      //  vertex_obstacles_marker.scale.x = IterateMap_.second->resolution;
      //  vertex_obstacles_marker.scale.y = IterateMap_.second->resolution;
      //  vertex_obstacles_marker.scale.z = 0.01;
      //  vertex_obstacles_marker.color.r = 255.0 / 255.0;
      //  vertex_obstacles_marker.color.g = 0.0 / 255.0;
      //  vertex_obstacles_marker.color.b = 255.0 / 255.0;
      //  vertex_obstacles_marker.color.a = 1.0;
      //  vertex_obstacles_marker.lifetime = ros::Duration(5.0);
      //  vertex_obstacles_marker.frame_locked = false;
      //  // vertex_obstacles_marker.points.push_back(p1);
      //  vertex_obstacles_marker.pose.position = p1;
      //  obstacle_seq_middle_++;
      //  MarkerArrayPublished_.markers.push_back(vertex_obstacles_marker);
    }
    else
    {
      MakersMapTem_[ZoomFactorTem_].points.push_back(p1);
      // std::string vertex_str_ = "vertex";
      // visualization_msgs::Marker vertex_marker;
      // // visualization_msgs::Marker vertex_marker;
      // vertex_marker.header.stamp = ros::Time::now();
      // vertex_marker.header.seq = normal_seq_middle_;
      // vertex_marker.header.frame_id = WorldFrame_.c_str();
      // vertex_marker.id = normal_seq_middle_;
      // vertex_marker.ns = (ns_ + vertex_str_).c_str();
      // vertex_marker.action = visualization_msgs::Marker::ADD;
      // vertex_marker.type = visualization_msgs::Marker::SPHERE; // CUBE
      // vertex_marker.scale.x = IterateMap_.second->resolution;
      // vertex_marker.scale.y = IterateMap_.second->resolution;
      // vertex_marker.scale.z = 0.01;
      // vertex_marker.color.r = 0.0 / 255.0;
      // vertex_marker.color.g = 255.0 / 255.0;
      // vertex_marker.color.b = 0.0 / 255.0;
      // vertex_marker.color.a = 1.0;
      // vertex_marker.lifetime = ros::Duration(5.0);
      // vertex_marker.frame_locked = false;
      // // vertex_marker.points.push_back(p1);
      // vertex_marker.pose.position = p1;
      // normal_seq_middle_++;
      // MarkerArrayPublished_.markers.push_back(vertex_marker);
    }
  }
  for (auto MarkerIteratorTem_ : MakersMapTem_)
  {
    // std::cout << "first is " << MarkerIteratorTem_.first << " and the second is " << MarkerIteratorTem_.second.id <<
    // " and " << MarkerIteratorTem_.second.points.size() << std::endl;
    MarkerArrayPublished_.markers.push_back(MarkerIteratorTem_.second);
  }
  // for (auto ObstacleMarkerIteratorTem_ : ObstaclesMakersMapTem_)
  // {
  //     MarkerArrayPublished_.markers.push_back(ObstacleMarkerIteratorTem_.second);
  // }
  MarkerArrayPublished_.markers.push_back(ObstaclesMarkerTem_);

  TopicPublisher->publish(MarkerArrayPublished_);
  if (!IfPubEdge)
  {
    return;
  }

  // Plot all edges
  std::string edge_str_ = "edge";
  visualization_msgs::Marker edge_marker;
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.header.seq = 1;
  edge_marker.header.frame_id = WorldFrame_.c_str();
  edge_marker.id = 0;
  edge_marker.ns = (ns_ + edge_str_).c_str();
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.scale.x = 0.02;
  edge_marker.color.r = 200.0 / 255.0;
  edge_marker.color.g = 100.0 / 255.0;
  edge_marker.color.b = 0.0;
  edge_marker.color.a = 1.0;
  edge_marker.lifetime = ros::Duration(10.0);
  edge_marker.frame_locked = false;

  std::unordered_map<int, std::unordered_map<int, double>> MapCopyTem_;
  GraphManager->getUndirectedMap(MapCopyTem_);
  std::set<int> IdVisited_;
  IdVisited_.clear();
  for (const auto &vertex1_ : MapCopyTem_)
  {
    int id1_ = vertex1_.first;
    int id2_;
    IdVisited_.insert(id1_);
    for (const auto &vertex2_ : vertex1_.second)
    {
      id2_ = vertex2_.first;
      if (IdVisited_.find(id2_) != IdVisited_.end())
      {
        continue;
      }
      geometry_msgs::Point p1;
      p1.x = GraphManager->vertices_map_[id1_]->state[0];
      p1.y = GraphManager->vertices_map_[id1_]->state[1];
      p1.z = GraphManager->vertices_map_[id1_]->state[2];
      geometry_msgs::Point p2;
      p2.x = GraphManager->vertices_map_[id2_]->state[0];
      p2.y = GraphManager->vertices_map_[id2_]->state[1];
      p2.z = GraphManager->vertices_map_[id2_]->state[2];
      edge_marker.points.push_back(p1);
      edge_marker.points.push_back(p2);
    }
  }

  if (edge_marker.points.size() < 1)
  {
  }
  else
  {
    EdgeTopicPublisher->publish(edge_marker);
  }
}

void VRMap::PlanToTarget3DPosition(Eigen::Vector3d TargetPosition3D,
                                   std::shared_ptr<VRGraphManager> SourceGlobalVariableResolutionGraphManager,
                                   std::shared_ptr<VRGraphManager> SourceLocalVariableResolutionGraphManager,
                                   std::shared_ptr<PathPlanner> SourcePathPlanner, int MaxTimeOfExpandGraph,
                                   std::shared_ptr<Ppath_Tracker> SourcePathTracker)
{
  StateVec TargetStateTem_(TargetPosition3D[0], TargetPosition3D[1], TargetPosition3D[2], 0);

  Vertex *TargetVertexTem_;
  int TargetIdTem2_;
  if (SourceGlobalVariableResolutionGraphManager->getNearestVertex(&TargetStateTem_, &TargetVertexTem_))
  {
    TargetIdTem2_ = TargetVertexTem_->id;
  }
  else
  {
    ROS_ERROR("[VRMapping_Info]: Could not find the global id of target when plan to target position %f, %f, %f.",
              TargetPosition3D[0], TargetPosition3D[1], TargetPosition3D[2]);
    return;
  }
  VRGraphManager::GraphType UndirectedMapTem_;
  UndirectedMapTem_.reset();
  Eigen::Vector3d GraphExpandVector3DTem_(0, 0, 0);
  SourcePathPlanner->PathRBVPosition_.clear();
  for (int IForExpand = 0; IForExpand < MaxTimeOfExpandGraph; IForExpand++)
  {
    ROS_INFO("[VRMapping_Info]: The %d th expand Local Plan Graph.", IForExpand);
    GraphExpandVector3DTem_[0] = GraphExpandVector3DTem_[0] + ExpandGraphX_;
    GraphExpandVector3DTem_[1] = GraphExpandVector3DTem_[1] + ExpandGraphX_;
    GraphExpandVector3DTem_[2] = GraphExpandVector3DTem_[2] + ExpandGraphX_;
    Eigen::Vector3d TargetVertexPosition3DTem_(TargetVertexTem_->state[0], TargetVertexTem_->state[1],
                                               TargetVertexTem_->state[2]);
    if (!SourcePathPlanner->GenerateLocalGraphManagerForPathPlanner(
            SourceGlobalVariableResolutionGraphManager, TargetVertexPosition3DTem_, GraphExpandVector3DTem_,
            TargetIdTem2_, AttractiveGain_, RepulsiveGain_, BoundRadiu_, WorldFrame_, RobotFrame_,
            SourceLocalVariableResolutionGraphManager, UndirectedMapTem_))
    {
      ROS_ERROR("[VRMapping_Info]: Can not Generate Local Plan Graph.");
      continue;
    }

    if (SourcePathPlanner->IfPlanPathByAStar_)
    {
      if (!SourcePathPlanner->PlanPathByAStarMethod(SourceLocalVariableResolutionGraphManager, UndirectedMapTem_,
                                                    TargetIdTem2_, SourcePathPlanner->PathRBVPosition_, WorldFrame_,
                                                    RobotFrame_))
      {
        ROS_ERROR("[VRMapping_Info]: Can not Generate Plan Path by A-Star Method.");
        continue;
      }
      if (SourcePathPlanner->PathRBVPosition_.size() < 1)
      {
        continue;
      }
      else
      {
        break;
      }
    }
    else
    {
      if (!SourcePathPlanner->PlanPathByDijkstraMethod(SourceLocalVariableResolutionGraphManager, TargetIdTem2_,
                                                       SourcePathPlanner->PathRBVPosition_, WorldFrame_, RobotFrame_,
                                                       UndirectedMapTem_))
      {
        ROS_ERROR("[VRMapping_Info]: Can not Generate Plan Path by Dijkstra Method.");
        continue;
      }
      if (SourcePathPlanner->PathRBVPosition_.size() < 1)
      {
        continue;
      }
      else
      {
        break;
      }
    }
  }

  if (SourcePathPlanner->PathRBVPosition_.size() < 1)
  {
    ROS_ERROR(
        "[VRMapping_Info]: Can not Generate Plan Path by after %d time expand and try to plan path by entire map.",
        MaxTimeOfExpandGraph);
    SourcePathPlanner->GenerateUndirectedMapByPotentialField(
        SourceGlobalVariableResolutionGraphManager, UndirectedMapTem_, TargetIdTem2_, AttractiveGain_, RepulsiveGain_,
        BoundRadiu_, SourceGlobalVariableResolutionGraphManager->ObstaclesSet_);
    if (SourcePathPlanner->IfPlanPathByAStar_)
    {
      if (!SourcePathPlanner->PlanPathByAStarMethod(SourceGlobalVariableResolutionGraphManager, UndirectedMapTem_,
                                                    TargetIdTem2_, SourcePathPlanner->PathRBVPosition_, WorldFrame_,
                                                    RobotFrame_))
      {
        ROS_ERROR("[VRMapping_Info]: Can not Generate Plan Path by A-Star Method.");
      }
    }
    else
    {
      if (!SourcePathPlanner->PlanPathByDijkstraMethod(SourceLocalVariableResolutionGraphManager, TargetIdTem2_,
                                                       SourcePathPlanner->PathRBVPosition_, WorldFrame_, RobotFrame_,
                                                       UndirectedMapTem_))
      {
        ROS_ERROR("[VRMapping_Info]: Can not Generate Plan Path by Dijkstra Method.");
      }
    }
  }

  if (SourcePathPlanner->PathRBVPosition_.size() < 1)
  {
    ROS_ERROR("[VRMapping_Info]: Can not Generate Plan Path by entire map.");
    VOXEL_LOC TargetPositionTem_(TargetPosition3D[0], TargetPosition3D[1],
                                 (double)((int)(TargetPosition3D[2] / 0.5)) * 0.5);
    SourcePathPlanner->NotReachableTargetPositions3D_[TargetPositionTem_] = 1;
    return;
  }

  string PNS_ = "Global"; // Planner Name Space.
  SourcePathPlanner->VisualizePath(SourcePathPlanner->PathRBVPosition_, PathPublisher_, PNS_, WorldFrame_);
  SourcePathTracker->clearPathTracked();
  SourcePathTracker->setPathTracked(SourcePathPlanner->PathRBVPosition_);
}

void VRMap::PlanToTarget3DPosition(Eigen::Vector3d TargetPosition3D,
                                   std::shared_ptr<VRGraphManager> SourceGlobalVariableResolutionGraphManager,
                                   std::shared_ptr<VRGraphManager> SourceLocalVariableResolutionGraphManager,
                                   std::shared_ptr<PathPlanner> SourcePathPlanner, int MaxTimeOfExpandGraph,
                                   std::shared_ptr<Ppath_Tracker> SourcePathTracker,
                                   bool IfReplacePotentialFiledByResolution, double MinResolutionValue,
                                   bool IfExplorationMode, double MinDistance)
{
  if (IfExplorationMode)
  {
    tf::TransformListener world_base_listener;
    tf::StampedTransform world_base_transform;
    try
    {
      world_base_listener.waitForTransform(WorldFrame_.c_str(), RobotFrame_.c_str(), ros::Time(0), ros::Duration(1.0));
      world_base_listener.lookupTransform(WorldFrame_.c_str(), RobotFrame_.c_str(), ros::Time(0), world_base_transform);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
      // ros::Duration(1.0).sleep();
    }
    Eigen::Vector3d RobotPosition3DTem_(world_base_transform.getOrigin().x(), world_base_transform.getOrigin().y(),
                                        world_base_transform.getOrigin().z() - SourcePathPlanner->RobotHeight_);
    Eigen::Vector3d Distance3DTem_ = TargetPosition3D - RobotPosition3DTem_;
    // TODO The 0.5 can be modified by different robots.
    if (Distance3DTem_.norm() < MinDistance)
    {
      ROS_ERROR(
          "[VRMapping_Info]: The target, whose position is %f, %f, %f, may be visited and should not be the target "
          "anymore.",
          TargetPosition3D[0], TargetPosition3D[1], TargetPosition3D[2]);
      VOXEL_LOC TargetPositionTem_(TargetPosition3D[0], TargetPosition3D[1],
                                   (double)((int)(TargetPosition3D[2] / 0.5)) * 0.5);
      SourcePathPlanner->NotReachableTargetPositions3D_[TargetPositionTem_] = 1;
      return;
    }
  }

  StateVec TargetStateTem_(TargetPosition3D[0], TargetPosition3D[1], TargetPosition3D[2], 0);

  Vertex *TargetVertexTem_;
  int TargetIdTem2_;
  if (SourceGlobalVariableResolutionGraphManager->getNearestVertex(&TargetStateTem_, &TargetVertexTem_))
  {
    TargetIdTem2_ = TargetVertexTem_->id;
    if (TargetVertexTem_->is_obstacle)
    {
      ROS_ERROR("[VRMapping_Info]:Target vertex is obstacle and the id is %d.", TargetIdTem2_);
      return;
    }
  }
  else
  {
    ROS_ERROR("[VRMapping_Info]: Could not find the global id of target when plan to target position %f, %f, %f.",
              TargetPosition3D[0], TargetPosition3D[1], TargetPosition3D[2]);
    return;
  }
  VRGraphManager::GraphType UndirectedMapTem_;
  UndirectedMapTem_.reset();
  Eigen::Vector3d GraphExpandVector3DTem_(0, 0, 0);
  SourcePathPlanner->PathRBVPosition_.clear();
  Eigen::Vector3d TargetVertexPosition3DTem_(TargetVertexTem_->state[0], TargetVertexTem_->state[1],
                                             TargetVertexTem_->state[2]);
  for (int IForExpand = 0; IForExpand < MaxTimeOfExpandGraph; IForExpand++)
  {
    ROS_INFO("[VRMapping_Info]: The %d th expand Local Plan Graph.", IForExpand);
    GraphExpandVector3DTem_[0] = GraphExpandVector3DTem_[0] + ExpandGraphX_;
    GraphExpandVector3DTem_[1] = GraphExpandVector3DTem_[1] + ExpandGraphX_;
    GraphExpandVector3DTem_[2] = GraphExpandVector3DTem_[2] + ExpandGraphX_;

    if (IfReplacePotentialFiledByResolution)
    {
      if (!SourcePathPlanner->GenerateLocalGraphManagerForPathPlanner(
              SourceGlobalVariableResolutionGraphManager, WorldFrame_, RobotFrame_, TargetVertexPosition3DTem_,
              GraphExpandVector3DTem_, MinResolutionValue, VRMapResolutionMax_, LimitBoxZ_,
              SourceLocalVariableResolutionGraphManager))
      {
        ROS_ERROR("[VRMapping_Info]: Can not Generate Local Plan Graph By Constrain Min Resolution.");
        continue;
      }
      if (SourceLocalVariableResolutionGraphManager->vertices_map_.find(TargetIdTem2_) ==
          SourceLocalVariableResolutionGraphManager->vertices_map_.end())
      {
        ROS_ERROR("[VRMapping_Info]:What the hail, the generated plan graph manager does not contain target vertex.");
        ros::shutdown();
        return;
      }
      UndirectedMapTem_.adjacencyList_ = SourceLocalVariableResolutionGraphManager->Graph_->adjacencyList_;
    }
    else
    {
      if (!SourcePathPlanner->GenerateLocalGraphManagerForPathPlanner(
              SourceGlobalVariableResolutionGraphManager, TargetVertexPosition3DTem_, GraphExpandVector3DTem_,
              TargetIdTem2_, AttractiveGain_, RepulsiveGain_, BoundRadiu_, WorldFrame_, RobotFrame_,
              SourceLocalVariableResolutionGraphManager, UndirectedMapTem_))
      {
        ROS_ERROR("[VRMapping_Info]: Can not Generate Local Plan Graph.");
        continue;
      }
    }

    if (SourcePathPlanner->IfPlanPathByAStar_)
    {
      if (!SourcePathPlanner->PlanPathByAStarMethod(SourceLocalVariableResolutionGraphManager, UndirectedMapTem_,
                                                    TargetIdTem2_, SourcePathPlanner->PathRBVPosition_, WorldFrame_,
                                                    RobotFrame_))
      {
        ROS_ERROR("[VRMapping_Info]: Can not Generate Plan Path by A-Star Method.");
        continue;
      }
      if (SourcePathPlanner->PathRBVPosition_.size() < 1)
      {
        continue;
      }
      else
      {
        break;
      }
    }
    else
    {
      if (!SourcePathPlanner->PlanPathByDijkstraMethod(SourceLocalVariableResolutionGraphManager, TargetIdTem2_,
                                                       SourcePathPlanner->PathRBVPosition_, WorldFrame_, RobotFrame_,
                                                       UndirectedMapTem_))
      {
        ROS_ERROR("[VRMapping_Info]: Can not Generate Plan Path by Dijkstra Method.");
        continue;
      }
      if (SourcePathPlanner->PathRBVPosition_.size() < 1)
      {
        continue;
      }
      else
      {
        break;
      }
    }
  }

  if ((SourcePathPlanner->PathRBVPosition_.size() < 1) || false)
  {
    ROS_ERROR(
        "[VRMapping_Info]: Can not Generate Plan Path by after %d time expand and try to plan path by entire map.",
        MaxTimeOfExpandGraph);
    if (IfReplacePotentialFiledByResolution)
    {
      if (SourcePathPlanner->GenerateLocalGraphManagerForPathPlanner(
              SourceGlobalVariableResolutionGraphManager, TargetVertexPosition3DTem_, MinTraResolution_,
              VRMapResolutionMax_, LimitBoxZ_, SourceLocalVariableResolutionGraphManager))
      {
        UndirectedMapTem_.adjacencyList_ = SourceLocalVariableResolutionGraphManager->Graph_->adjacencyList_;
      }
      else
      {
        ROS_ERROR("[VRMapping_Info]: Can not Generate Local Entire Plan Graph.");
        return;
      }
    }
    else
    {
      SourcePathPlanner->GenerateUndirectedMapByPotentialField(
          SourceGlobalVariableResolutionGraphManager, UndirectedMapTem_, TargetIdTem2_, AttractiveGain_, RepulsiveGain_,
          BoundRadiu_, SourceGlobalVariableResolutionGraphManager->ObstaclesSet_);
    }

    if (SourcePathPlanner->IfPlanPathByAStar_)
    {
      if (!SourcePathPlanner->PlanPathByAStarMethod(SourceGlobalVariableResolutionGraphManager, UndirectedMapTem_,
                                                    TargetIdTem2_, SourcePathPlanner->PathRBVPosition_, WorldFrame_,
                                                    RobotFrame_))
      {
        ROS_ERROR("[VRMapping_Info]: Can not Generate Plan Path by A-Star Method.");
      }
    }
    else
    {
      if (!SourcePathPlanner->PlanPathByDijkstraMethod(SourceLocalVariableResolutionGraphManager, TargetIdTem2_,
                                                       SourcePathPlanner->PathRBVPosition_, WorldFrame_, RobotFrame_,
                                                       UndirectedMapTem_))
      {
        ROS_ERROR("[VRMapping_Info]: Can not Generate Plan Path by Dijkstra Method.");
      }
    }
  }

  if (SourcePathPlanner->PathRBVPosition_.size() < 1)
  {
    ROS_ERROR("[VRMapping_Info]: Can not Generate Plan Path by entire map.");
    VOXEL_LOC TargetPositionTem_(TargetPosition3D[0], TargetPosition3D[1],
                                 (double)((int)(TargetPosition3D[2] / 0.5)) * 0.5);
    SourcePathPlanner->NotReachableTargetPositions3D_[TargetPositionTem_] = 1;
    return;
  }

  string PNS_ = "Global"; // Planner Name Space.
  SourcePathPlanner->VisualizePath(SourcePathPlanner->PathRBVPosition_, PathPublisher_, PNS_, WorldFrame_);
  SourcePathTracker->clearPathTracked();
  SourcePathTracker->setPathTracked(SourcePathPlanner->PathRBVPosition_);
}

void VRMap::UpdateTargetWithoutSampleGraph()
{
  // TODO TARGET SELECT AND PATH PLAN IS PROCESSING IN ONE SAME GRAPH.
  ROSTIME TStart_;
  START_TIME(TStart_);
  std::unique_lock<std::mutex> lock2(GVRGMMutex_);
  ROSTIME TStartCopy_;
  START_TIME(TStartCopy_);

  if (!PathPlanner_->DeepCopyVRGrahManager(GVRGM_, GFPPVRGM_))
  {
    ROS_ERROR("[VRMapping_Info]: Deep Copy GVRGM_ To GFPPVRGM_ Faild.");
    return;
  }
  std::string GFPPVRGMNS_ = "GlobalPlanner";
  VisualizeVRGraphManager(GFPPVRGM_, GFPPVRGMPublisher_, GFPPVRGMNS_, true, GFPPVRGMEPublisher_);

  lock2.unlock();
  std::unordered_map<int, int> IdAndPreviousTem_;
  IdAndPreviousTem_.clear();
  std::unordered_map<int, double> IdAndDistanceTem_;
  IdAndDistanceTem_.clear();
  std::unordered_map<int, double> IdAndAttributeTem_;
  IdAndAttributeTem_.clear();
  std::unordered_map<int, Eigen::Vector4d> IdCenterTem_;
  IdCenterTem_.clear();
  bool IfProcessedTem_ = false;

  Eigen::Vector3d SampledGraphExpandVector3DTem_(0, 0, 0);

  bool IfUseGlobalSampledGraphTem_ = false;
  float TimeUsedTem_ = GET_ELAPSED_TIME(TStart_);
  ROS_INFO(" Time used for prepare select targetis %f", TimeUsedTem_);

  for (int ITem_ = 0; ITem_ < MaxExpandSampledGraphTime_; ITem_++)
  {
    // ROS_INFO("[VRMapping_Info]: The %d th expand Local Sampled Target Selection Graph.", ITem_);
    SampledGraphExpandVector3DTem_[0] = SampledGraphExpandVector3DTem_[0] + ExpandSampledGraphForTargetSelectionX_;
    SampledGraphExpandVector3DTem_[1] = SampledGraphExpandVector3DTem_[1] + ExpandSampledGraphForTargetSelectionY_;
    SampledGraphExpandVector3DTem_[2] = SampledGraphExpandVector3DTem_[2] + ExpandSampledGraphForTargetSelectionZ_;
    if (PathPlanner_->GenerateLocalGraphManagerForTargetSelect(GlobalSampler_->GSOGM_, WorldFrame_, RobotFrame_,
                                                               SampledGraphExpandVector3DTem_, GlobalSampler_->LSOGM_))
    {
      GlobalSampler_->ConvertPositionToIdAttribute(
          GlobalSampler_->LSOGM_, FrontierExtractor_->Centers_, GlobalSampler_->EdgeLengthMin_ + 0.05, ICTEA_, COTEA_,
          PathPlanner_->NotReachableTargetPositions3D_, IdAndAttributeTem_, IdCenterTem_);
      if (IdAndAttributeTem_.size() > 0)
      {
        break;
      }
    }
    else
    {
      continue;
    }
  }

  if (IdAndAttributeTem_.size() < 1)
  {
    ROS_ERROR(
        "[VRMapping_Info]: Can not find frontiers in local sampled graphs corresponding to frontiers in global graph "
        "by %d times expand.  And try to convert by global sampled graph.",
        MaxExpandSampledGraphTime_);
    GlobalSampler_->ConvertPositionToIdAttribute(
        GlobalSampler_->GSOGM_, FrontierExtractor_->Centers_, GlobalSampler_->EdgeLengthMin_ + 0.05, ICTEA_, COTEA_,
        PathPlanner_->NotReachableTargetPositions3D_, IdAndAttributeTem_, IdCenterTem_);
    IfUseGlobalSampledGraphTem_ = true;
  }

  TimeUsedTem_ = GET_ELAPSED_TIME(TStart_);
  ROS_INFO(" Time used for convert target into sample graph is %f", TimeUsedTem_);

  if ((IdAndAttributeTem_.size() < 1) || false)
  {
    ROS_ERROR(
        "[VRMapping_Info]: The Id Vector to be selected is empty and we choose to select the target nearest in "
        "Euclidean distance.");
    Eigen::Vector3d TargetPosition3DTem_;
    if (PathPlanner_->TargetPointSelectByEuclideanDistance(FrontierExtractor_->Centers_, WorldFrame_, RobotFrame_,
                                                           ICTEA_, COTEA_, PathPlanner_->NotReachableTargetPositions3D_,
                                                           TargetPosition3DTem_))
    {
      PlanToTarget3DPosition(TargetPosition3DTem_, GFPPVRGM_, LFPPVRGM_, PathPlanner_, ExpandGraphTime_,
                             GlobalPathTracker_, IfReplacePotentialFiledByResolution_, MinTraResolution_, true,
                             GlobalPathTracker_->GetGoalReachingThresholdValue());
    }
    else
    {
      ROS_ERROR("[VRMapping_Info]: Can not select target by Euclidean distance.");
    }
    // ROS_INFO("[VRMapping_Info]: Target 3D Position Selected is %f, %f, %f.", TargetPosition3DTem_[0],
    // TargetPosition3DTem_[1], TargetPosition3DTem_[2]);
  }
  else
  {
    if (IfUseGlobalSampledGraphTem_)
    {
      IfProcessedTem_ = PathPlanner_->DijstraProcess(GlobalSampler_->GSOGM_, WorldFrame_, RobotFrame_,
                                                     IdAndPreviousTem_, IdAndDistanceTem_);
    }
    else
    {
      IfProcessedTem_ = PathPlanner_->DijstraProcess(GlobalSampler_->LSOGM_, WorldFrame_, RobotFrame_,
                                                     IdAndPreviousTem_, IdAndDistanceTem_);
    }

    if (IfProcessedTem_)
    {
      if (true)
      {
        TimeUsedTem_ = GET_ELAPSED_TIME(TStart_);
        ROS_INFO(" Time used for dijkstra process sample graph is %f", TimeUsedTem_);
        // std::cout << "Size of IdCenterTem_ is " << IdCenterTem_.size() << std::endl;
        // for (auto IteratorCentersTem_ : IdCenterTem_)
        // {
        //     std::cout << "Id is " << IteratorCentersTem_.first << " and center is " << std::endl
        //               << IteratorCentersTem_.second << std::endl;
        // }
        Eigen::Vector3d TargetPosition3DTem_;
        if (PathPlanner_->TargetPointSelectByExplorationGain(
                GFPPVRGM_, IdCenterTem_, IdAndDistanceTem_, PathPlanner_->NotReachableFrontierIdsInSamplerGraph_,
                WorldFrame_, RobotFrame_, PathPlanner_->LambdaSize_, PathPlanner_->LambdaDistance_,
                PathPlanner_->LambdaObstacle_, PathPlanner_->LambdaZDistance_, TargetPosition3DTem_))
        {
          TimeUsedTem_ = GET_ELAPSED_TIME(TStart_);
          ROS_INFO(" Time used for select by exploration gain is %f", TimeUsedTem_);
          PlanToTarget3DPosition(TargetPosition3DTem_, GFPPVRGM_, LFPPVRGM_, PathPlanner_, ExpandGraphTime_,
                                 GlobalPathTracker_, IfReplacePotentialFiledByResolution_, MinTraResolution_, true,
                                 GlobalPathTracker_->GetGoalReachingThresholdValue());
          TimeUsedTem_ = GET_ELAPSED_TIME(TStart_);
          ROS_INFO(" Time used for plan path is %f", TimeUsedTem_);
        }
        else
        {
          ROS_ERROR(
              "[VRMapping_Info]: Can not get the target Id by select max exploration gain and we choose to select the "
              "target nearest in Euclidean distance.");
          if (PathPlanner_->TargetPointSelectByEuclideanDistance(
                  FrontierExtractor_->Centers_, WorldFrame_, RobotFrame_, ICTEA_, COTEA_,
                  PathPlanner_->NotReachableTargetPositions3D_, TargetPosition3DTem_))
          {
            PlanToTarget3DPosition(TargetPosition3DTem_, GFPPVRGM_, LFPPVRGM_, PathPlanner_, ExpandGraphTime_,
                                   GlobalPathTracker_, IfReplacePotentialFiledByResolution_, MinTraResolution_, true,
                                   GlobalPathTracker_->GetGoalReachingThresholdValue());
          }
          else
          {
            ROS_ERROR("[VRMapping_Info]: Can not select target by Euclidean distance.");
          }
        }
      }
      else
      {
        int TargetIdInSamplerGraphTem_ = 0;
        if (PathPlanner_->TargetPointSelectByDistance(IdAndAttributeTem_, IdAndDistanceTem_, TargetIdInSamplerGraphTem_,
                                                      PathPlanner_->NotReachableFrontierIdsInSamplerGraph_))
        {
          Eigen::Vector4d TargetCenterTem_ = IdCenterTem_[TargetIdInSamplerGraphTem_];
          Eigen::Vector3d TargetPosition3DTem_(TargetCenterTem_[0], TargetCenterTem_[1], TargetCenterTem_[2]);
          // ROS_INFO("[VRMapping_Info]: Target 3D Position Selected is %f, %f, %f.", TargetPosition3DTem_[0],
          // TargetPosition3DTem_[1], TargetPosition3DTem_[2]);
          PlanToTarget3DPosition(TargetPosition3DTem_, GFPPVRGM_, LFPPVRGM_, PathPlanner_, ExpandGraphTime_,
                                 GlobalPathTracker_, IfReplacePotentialFiledByResolution_, MinTraResolution_, true,
                                 GlobalPathTracker_->GetGoalReachingThresholdValue());
          if (PathPlanner_->PathRBVPosition_.size() < 1)
          {
            ROS_ERROR("[VRMapping_Info]: Can not generate path to target position %f, %f, %f in sample graph.",
                      TargetPosition3DTem_[0], TargetPosition3DTem_[1], TargetPosition3DTem_[2]);

            // PathPlanner_->NotReachableFrontierIdsInSamplerGraph_.insert(TargetIdInSamplerGraphTem_);
            return;
          }
        }
        else
        {
          ROS_ERROR(
              "[VRMapping_Info]: Can not get the target Id by select min distance and we choose to select the target "
              "nearest in Euclidean distance.");
          Eigen::Vector3d TargetPosition3DTem_;
          if (PathPlanner_->TargetPointSelectByEuclideanDistance(
                  FrontierExtractor_->Centers_, WorldFrame_, RobotFrame_, ICTEA_, COTEA_,
                  PathPlanner_->NotReachableTargetPositions3D_, TargetPosition3DTem_))
          {
            PlanToTarget3DPosition(TargetPosition3DTem_, GFPPVRGM_, LFPPVRGM_, PathPlanner_, ExpandGraphTime_,
                                   GlobalPathTracker_, IfReplacePotentialFiledByResolution_, MinTraResolution_, true,
                                   GlobalPathTracker_->GetGoalReachingThresholdValue());
          }
          else
          {
            ROS_ERROR("[VRMapping_Info]: Can not select target by Euclidean distance.");
          }
        }
      }
    }
    else
    {
      ROS_ERROR("[VRMapping_Info]: Use Dijstra method to process sampled graph failed.");
    }
  }
  float TimeUsed_ = GET_ELAPSED_TIME(TStart_);
  ROS_INFO(" Time used for select target and plan path is %f", TimeUsed_);
}

void VRMap::SetupIntegrator()
{
  CheckCudaDevice();
  IntegratorNS::IntegratorBase::Config IntegratorConfig_;
  IntegratorConfig_.max_integration_time_s = 1.0;
  IntegratorConfig_.integration_order_mode = "FastSimple";
  IntegratorConfig_.ElevationMapResolution_ = ElevationMapResolution_;
  IntegratorConfig_.LimitBoxZ_ = LimitBoxZ_;
  IntegratorConfig_.LimitBoxZForConnectEdge_ = LimitBoxZForConnectEdge_;
  IntegratorConfig_.ValidThresholdForMultiMapResolution_ = ValidThresholdForMultiMapResolution_;
  IntegratorConfig_.VRMapResolutionMax_ = VRMapResolutionMax_;
  IntegratorConfig_.VRMapResolutionMin_ = VRMapResolutionMin_;

  int IntegratorThreadsTem_ = 0;
  nh_.param<int>("/vrmapping/vrmapping_settings/IntegratorThreads",
                 IntegratorThreadsTem_, true);
  if (IntegratorThreadsTem_ > 0)
  {
    IntegratorConfig_.integrator_threads = IntegratorThreadsTem_;
  }
  IntegratorPtr_.reset(new IntegratorNS::FastSimpleIntegrator(IntegratorConfig_, GVRGM_));
}

bool VRMap::CheckCudaDevice()
{
  int deviceCount = 0; // gpu count.
  cudaError_t error_id = cudaGetDeviceCount(&deviceCount);

  if (error_id != cudaSuccess)
  {
    ROS_ERROR("CUDA error: %s", cudaGetErrorString(error_id));
    return false;
  }

  if (deviceCount == 0)
  {
    ROS_ERROR("No CUDA devices found.");
    return false;
  }

  ROS_INFO("Found %d CUDA devices.", deviceCount);
  cudaDeviceProp devProp;
  cudaError_t error2_id = cudaGetDeviceProperties(&devProp, deviceCount);
  std::cout << "GPU Device" << deviceCount << ": " << devProp.name << std::endl;
  std::cout << "SM: " << devProp.multiProcessorCount << std::endl;
  std::cout << "devProp.sharedMemPerBlock:" << devProp.sharedMemPerBlock / 1024.0 << " KB" << std::endl;
  std::cout << "devProp.maxThreadsPerBlock:" << devProp.maxThreadsPerBlock << std::endl;
  std::cout << "devProp.maxThreadsPerMultiProcessor:" << devProp.maxThreadsPerMultiProcessor << std::endl;
  std::cout << "devProp.maxThreadsPerMultiProcessor:" << devProp.maxThreadsPerMultiProcessor / 32 << std::endl;
  return true;
}

bool VRMap::PlanningByTargetPositonCallback(vrmapping_msgs::VarmappingPubTarget::Request &req,
                                            vrmapping_msgs::VarmappingPubTarget::Response &res)
{
  std::unique_lock<std::mutex> lock2(GVRGMMutex_);
  if (!PathPlanner_->DeepCopyVRGrahManager(GVRGM_, GFPPVRGM_))
  {
    ROS_ERROR("[VRMapping_Info]: Deep Copy GVRGM_ To GFPPVRGM_ Faild.");
    res.result = false;
    return false;
  }
  lock2.unlock();

  Eigen::Vector3d TargetPosition_(req.TargetPosition.x, req.TargetPosition.y, req.TargetPosition.z);
  PlanToTarget3DPosition(TargetPosition_, GFPPVRGM_, LFPPVRGM_, PathPlanner_, ExpandGraphTime_, GlobalPathTracker_,
                         IfReplacePotentialFiledByResolution_, MinTraResolution_, false, 0.0);

  res.result = true;
  return true;
}

bool VRMap::InitializationCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  std::vector<Eigen::Vector3d> path_foward_;
  path_foward_.clear();
  tf::TransformListener World_Robot_listener;
  tf::StampedTransform World_Robot_transform;
  try
  {
    World_Robot_listener.waitForTransform(WorldFrame_.c_str(), RobotFrame_.c_str(), ros::Time(0), ros::Duration(1.0));
    World_Robot_listener.lookupTransform(WorldFrame_.c_str(), RobotFrame_.c_str(), ros::Time(0), World_Robot_transform);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
    // ros::Duration(1.0).sleep();
  }
  Eigen::Vector3d robot_current_state(World_Robot_transform.getOrigin().x(), World_Robot_transform.getOrigin().y(),
                                      World_Robot_transform.getOrigin().z());
  tf::Quaternion q_robot_current = World_Robot_transform.getRotation();
  double robot_yaw_current = tf::getYaw(q_robot_current);
  double x_forward = 2.0;
  double x_add_in_world_frame = x_forward * cos(robot_yaw_current);
  double y_add_in_world_frame = x_forward * sin(robot_yaw_current);
  Eigen::Vector3d target_state(robot_current_state[0] + x_add_in_world_frame,
                               robot_current_state[1] + y_add_in_world_frame, robot_current_state[2]);
  path_foward_.push_back(robot_current_state);
  path_foward_.push_back(target_state);

  GlobalPathTracker_->clearPathTracked();
  GlobalPathTracker_->setPathTracked(path_foward_);

  res.success = true;
  return true;
}

bool VRMap::StartExplorationCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  IfExplore_ = true;
  res.success = true;
  return true;
}

bool VRMap::StopExplorationCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  IfExplore_ = false;
  GlobalPathTracker_->stopTracker();
  res.success = true;
  return true;
}
