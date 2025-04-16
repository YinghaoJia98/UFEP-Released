#include <vrmapping/PathTracker.h>

Ppath_Tracker::Ppath_Tracker(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private)
{
  start_execution_ = false;
  if_need_stop = true;
  if_track_succeed = false;
  if_pause = false;
  status = false;
  CarrotPointIdToOutput = 0;
  CurrentCarrotPose << 0, 0, 0;
  if_need_turn = false;
  Target_attitude << 0, 0;
  IfNeedUpdateNextTarget_ = false;
  listener_ = std::make_shared<tf::TransformListener>();
  LoadParam();
  Initialize();
}

Ppath_Tracker::~Ppath_Tracker()
{
}

void Ppath_Tracker::LoadParam()
{
  nh_.param<bool>("/vrmapping/path_tracker_settings/if_tracker_enable", if_tracker_enable, false);
  std::string cmd_topic_, carrot_point_topic;
  nh_.param<std::string>("/vrmapping/path_tracker_settings/cmd_topic_", cmd_topic_, "/cmd_vel");
  nh_.param<double>("/vrmapping/path_tracker_settings/goal_reaching_threshold", goal_reaching_threshold_, 0.1);
  nh_.param<std::string>("/vrmapping/path_tracker_settings/World_Frame", World_Frame_, std::string("world"));
  nh_.param<std::string>("/vrmapping/path_tracker_settings/Robot_Frame", Robot_Frame_, std::string("base_link"));
  nh_.param<double>("/vrmapping/path_tracker_settings/look_ahead_distance", look_ahead_distance_, 0.5);
  nh_.param<bool>("/vrmapping/path_tracker_settings/IfTryFindInABigDistance", IfTryFindInABigDistance_, false);
  nh_.param<double>("/vrmapping/path_tracker_settings/look_ahead_distanceInMax", look_ahead_distanceInMax_, 1.0);
  nh_.param<int>("/vrmapping/path_tracker_settings/look_ahead_verticesInMax", look_ahead_verticesInMax_, 100);
  nh_.param<double>("/vrmapping/path_tracker_settings/look_ahead_error_margin", look_ahead_error_margin_, 0.2);
  nh_.param<std::string>("/vrmapping/path_tracker_settings/carrot_point_topic", carrot_point_topic, "/carrot_point");
  nh_.param<double>("/vrmapping/path_tracker_settings/curvature_scaling_angle", curvature_scaling_angle_, 0.52);
  nh_.param<double>("/vrmapping/path_tracker_settings/critical_angle", critical_angle_, 0.52);
  nh_.param<double>("/vrmapping/path_tracker_settings/max_ang_vel", max_ang_vel_, 0.5);
  nh_.param<double>("/vrmapping/path_tracker_settings/desired_vel", desired_vel_, 0.5);
  nh_.param<int>("/vrmapping/path_tracker_settings/num_point_in_path_look_ahead_for_turn_",
                 num_point_in_path_look_ahead_for_turn_, 10);
  nh_.param<int>("/vrmapping/path_tracker_settings/min_inver_point", min_inver_point, 10);

  nh_.param<bool>("/vrmapping/path_tracker_settings/IfAheadThePathAllTheTime", IfAheadThePathAllTheTime_, false);

  nh_.param<double>("/vrmapping/vrmapping_settings/RobotHeight", RobotHeight_, 0.235);
  nh_.param<int>("/vrmapping/path_tracker_settings/RemainVerticesNumMinValue", RemainVerticesNumMinValue_, 3);
  nh_.param<int>("/vrmapping/path_tracker_settings/RemainVerticesNumMaxValue", RemainVerticesNumMaxValue_, 6);
  nh_.param<double>("/vrmapping/path_tracker_settings/DistanceOfTargetMinValue", DistanceOfTargetMinValue_, 0.5);

  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_.c_str(), 10);
  carrot_point_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(carrot_point_topic.c_str(), 10);
}

void Ppath_Tracker::Initialize()
{
  if (if_tracker_enable)
  {
    double path_track_fps;
    nh_.param<double>("/vrmapping/path_tracker_settings/path_track_fps", path_track_fps, 1.0);
    double duration = 1.0 / (path_track_fps + 0.00001);
    Path_Tracker_Timer = nh_.createTimer(ros::Duration(duration),
                                         &Ppath_Tracker::PathTrackTimerCallBack, this);
  }

  if (1)
  {
    double pose_update_fps;
    nh_.param<double>("/vrmapping/path_tracker_settings/pose_update_fps", pose_update_fps, 10.0);
    double duration_update_pose = 1.0 / (pose_update_fps + 0.00001);
    Update_Current_Robot_Pose_Timer = nh_.createTimer(ros::Duration(duration_update_pose),
                                                      &Ppath_Tracker::UpdateRobotPoseCallBack, this);
  }
}

void Ppath_Tracker::setPathTracked(std::vector<Eigen::Vector3d> path_tracked_input)
{
  std::lock_guard<std::mutex> lock1(PathMutex_);
  std::lock_guard<std::mutex> lock2(PauseMutex_);
  std::lock_guard<std::mutex> lock3(StopMutex_);
  path_tracked_.clear();
  path_tracked_ = path_tracked_input;
  if_need_stop = false;
  if_pause = false;
  status = true;
  // TODO pause logic could be change if needed;
}

void Ppath_Tracker::Track_path(std::vector<Eigen::Vector3d> &path_tracked)
{
  start_execution_ = true;
  if (if_need_stop)
  {
    ROS_INFO("Tracker is stopping.");
    // Publish 0 velocity
    geometry_msgs::Twist vel;
    vel.linear.x = 0.0;
    vel.angular.z = 0.0;
    cmd_vel_ = vel;
    cmd_vel_pub_.publish(cmd_vel_);
    path_tracked.clear();
    status = false;
    return;
  }
  else
  {
  }

  if (calculateDistance(path_tracked.back(), current_position_) < goal_reaching_threshold_)
  {
    ROS_INFO("Ppath Tracker: Reached end of path");
    if_track_succeed = true;
    if_need_stop = true;
    path_tracked.clear();
    geometry_msgs::Twist vel;
    vel.linear.x = 0.0;
    vel.angular.z = 0.0;
    cmd_vel_ = vel;
    cmd_vel_pub_.publish(cmd_vel_);
    path_tracked.clear();
    status = false;
    // result_.success = true;
    // as_.setSucceeded(result_);
    // start_execution_ = false;

    return;
  }
  if (computeCommandVelocity(path_tracked))
  {
    if_track_succeed = false;
    // std::cout << "cmd vel: " << cmd_vel_.linear.x << ", " << cmd_vel_.angular.z << std::endl;
    cmd_vel_pub_.publish(cmd_vel_);
  }
  else
  {
    ROS_WARN("Ppath Tracker: Can't track given path");
  }
}

void Ppath_Tracker::PathTrackTimerCallBack(const ros::TimerEvent &event)
{
  std::lock_guard<std::mutex> lock1(PathMutex_);
  // std::lock_guard<std::mutex> lock2(PauseMutex_);
  // std::lock_guard<std::mutex> lock3(StopMutex_);
  std::lock_guard<std::mutex> lock4(StatueMutex_);
  std::lock_guard<std::mutex> lock5(IfSucceedMutex_);
  std::lock_guard<std::mutex> lock6(CmdVelMutex_);
  std::lock_guard<std::mutex> lock7(CarrotPointMutex_);
  std::lock_guard<std::mutex> lock8(IfNeedToTurnMutex_);
  if (!if_pause)
  {
    if (path_tracked_.size() > 0)
    {
      // if_need_stop = false;
      // if_track_succeed = false;
      Track_path(path_tracked_);
      status = true;
    }
    else
    {
      status = false;
    }
  }
  else
  {
    status = false;
  }
}

void Ppath_Tracker::UpdateRobotPoseCallBack(const ros::TimerEvent &event)
{
  try
  {
    World_Robot_listener.waitForTransform(World_Frame_.c_str(), Robot_Frame_.c_str(), ros::Time(0), ros::Duration(1.0));
    World_Robot_listener.lookupTransform(World_Frame_.c_str(), Robot_Frame_.c_str(),
                                         ros::Time(0), World_Robot_transform);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("There might be something wrong when tryying update robot pose for path tracker");
    ROS_ERROR("%s", ex.what());
    // ros::Duration(1.0).sleep();
  }

  Eigen::Vector3d pos_middle(World_Robot_transform.getOrigin().x(), World_Robot_transform.getOrigin().y(), World_Robot_transform.getOrigin().z() - RobotHeight_);
  current_position_ = pos_middle;
}

double Ppath_Tracker::calculateDistance(const Eigen::Vector3d p1, const Eigen::Vector3d p2)
{
  return (p1 - p2).head(3).norm();
}

bool Ppath_Tracker::computeCommandVelocity(std::vector<Eigen::Vector3d> &PathTracked)
{
  // Find closest point on the path
  std::vector<double> dists;
  for (size_t i = 0; i < PathTracked.size(); ++i)
  {
    dists.push_back(calculateDistance(current_position_, PathTracked[i]));
  }
  int closest_waypoint_ind = std::min_element(dists.begin(), dists.end()) - dists.begin();
  int ForwardVerticesTrackedNumTem_ = PathTracked.size() - closest_waypoint_ind;
  double DistanceToTrackedTargetTem_ = calculateDistance(current_position_, PathTracked.back());
  if (((ForwardVerticesTrackedNumTem_ < RemainVerticesNumMinValue_) &&
       (DistanceToTrackedTargetTem_ < DistanceOfTargetMinValue_)) ||
      (ForwardVerticesTrackedNumTem_ > RemainVerticesNumMaxValue_))
  {
    IfNeedUpdateNextTarget_ = true;
  }
  else
  {
    IfNeedUpdateNextTarget_ = false;
  }
  // Prune path to the closest waypoint
  std::vector<Eigen::Vector3d> tmp_path = PathTracked;
  PathTracked.clear();
  for (size_t i = closest_waypoint_ind; i < tmp_path.size(); ++i)
  {
    PathTracked.push_back(tmp_path[i]);
  }
  // Now the path is from the nearest point to target.

  // Find carrot waypoint
  size_t carrot_point_ind = PathTracked.size() - 1;
  // size_t carrot_point_ind = 0;
  bool IfFindBaseCarrotTem_ = false;
  for (size_t i = 0; i < PathTracked.size(); ++i)
  {
    if (IfTryFindInABigDistance_)
    {
      if (!IfFindBaseCarrotTem_)
      {
        if (calculateDistance(current_position_, PathTracked[i]) >= look_ahead_distance_)
        {
          carrot_point_ind = i;
          IfFindBaseCarrotTem_ = true;
        }
      }
      if ((calculateDistance(current_position_, PathTracked[i]) >= look_ahead_distanceInMax_) && (i < look_ahead_verticesInMax_))
      {
        carrot_point_ind = i;
        break;
      }
    }
    else
    {
      if (calculateDistance(current_position_, PathTracked[i]) >= look_ahead_distance_)
      {
        carrot_point_ind = i;
        break;
      }
    }
  }
  // Now find the next point need to go.
  CarrotPointIdToOutput = carrot_point_ind + closest_waypoint_ind;
  // std::cout << "CarrotPointIdToOutput is " << CarrotPointIdToOutput << std::endl;
  // std::cout << "tmp_path CarrotPointIdToOutput is " << tmp_path[CarrotPointIdToOutput] << std::endl;
  // std::cout << "tmp_path.size is " << tmp_path.size() << std::endl;

  // Carrot waypoint to follow
  Eigen::Vector3d carrot_waypoint = PathTracked[carrot_point_ind];
  CurrentCarrotPose = carrot_waypoint;

  std::vector<Eigen::Vector3d> path_used_for_turn_judgement_;
  path_used_for_turn_judgement_.clear();
  path_used_for_turn_judgement_ = PathTracked;
  // std::cout << "CurrentCarrotPose is " << CurrentCarrotPose << std::endl;

  if (calculateDistance(current_position_, PathTracked[carrot_point_ind]) >= (look_ahead_distance_ + look_ahead_error_margin_) && carrot_point_ind != 0)
  {
    // Carrot point is further away than the look ahead distance
    // and it is not the beginning of the path
    Eigen::Vector3d v2 = PathTracked[carrot_point_ind];

    Eigen::Vector3d v1 = PathTracked[carrot_point_ind - 1];

    Eigen::Vector3d carrot_point_vec;
    Eigen::Vector3d dir_vec = v2 - v1;
    for (double d = 0.0; d < dir_vec.norm(); d += look_ahead_error_margin_)
    {
      Eigen::Vector3d v = v1 + (v2 - v1).normalized() * d;
      Eigen::Vector3d cv = current_position_;
      if ((v - cv).norm() >= look_ahead_distance_)
      {
        carrot_point_vec = v;
        break;
      }
    }
    // carrot_point_vec = look_ahead_distance_ * (v2 - v1).normalized() + v1;
    tf::Quaternion quat;
    quat.setEuler(0.0, 0.0, std::atan2(dir_vec(1), dir_vec(0)));
    tf::Vector3 origin(carrot_point_vec(0), carrot_point_vec(1), carrot_point_vec(2));
    tf::Pose poseTF(quat, origin);
    geometry_msgs::Pose carrot_waypoint_pos_geo_msg;
    carrot_waypoint_pos_geo_msg.position.x = carrot_waypoint[0];
    carrot_waypoint_pos_geo_msg.position.y = carrot_waypoint[1];
    carrot_waypoint_pos_geo_msg.position.z = carrot_waypoint[2];
    tf::poseTFToMsg(poseTF, carrot_waypoint_pos_geo_msg);
  }

  tmp_path = PathTracked;
  PathTracked.clear();
  Eigen::Vector3d carrot_waypoint_stamped;
  carrot_waypoint_stamped = carrot_waypoint;
  PathTracked.push_back(carrot_waypoint_stamped);
  for (size_t ind = carrot_point_ind + 1; ind < tmp_path.size(); ++ind)
  {
    PathTracked.push_back(tmp_path[ind]);
  }

  // Transform carrot point into robot frame for easier calculations:
  tf::StampedTransform fixed_to_robot_transform;
  try
  {
    listener_->lookupTransform(Robot_Frame_, World_Frame_, ros::Time(0), fixed_to_robot_transform);
    // TODO Is it wrong? Robot to World?
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("The tf get wrong when compute CommandVelocity in Ppath tracker.");
    ROS_ERROR("%s", ex.what());
    return false;
  }
  geometry_msgs::Pose carrot_waypoint_robot;
  tf::Vector3 carrot_point_tf_fixed;

  geometry_msgs::Pose carrot_waypoint_pos_geo_msg2;
  carrot_waypoint_pos_geo_msg2.position.x = carrot_waypoint[0];
  carrot_waypoint_pos_geo_msg2.position.y = carrot_waypoint[1];
  carrot_waypoint_pos_geo_msg2.position.z = carrot_waypoint[2];

  tf::pointMsgToTF(carrot_waypoint_pos_geo_msg2.position, carrot_point_tf_fixed);
  tf::Vector3 carrot_point_tf_robot = fixed_to_robot_transform * carrot_point_tf_fixed;
  tf::pointTFToMsg(carrot_point_tf_robot, carrot_waypoint_robot.position);

  geometry_msgs::PoseStamped ps;
  ps.header.frame_id = Robot_Frame_;
  ps.pose = carrot_waypoint_robot;

  carrot_point_pub_.publish(ps);

  Eigen::Vector3d carrot_vec_robot;
  carrot_vec_robot << carrot_waypoint_robot.position.x, carrot_waypoint_robot.position.y, carrot_waypoint_robot.position.z;

  int lin_vel_sign = (carrot_waypoint_robot.position.x >= 0) ? 1 : -1;
  double delta_theta = std::atan2(carrot_waypoint_robot.position.y, carrot_waypoint_robot.position.x);
  if (lin_vel_sign < 0)
  {
    if (delta_theta > 0)
    {
      // delta_theta -= M_PI;
      if (!IfAheadThePathAllTheTime_)
      {
        delta_theta -= M_PI;
      }
    }
    else
    {
      // delta_theta += M_PI;
      if (!IfAheadThePathAllTheTime_)
      {
        delta_theta += M_PI;
      }
    }
  }
  int ang_vel_sign = (delta_theta >= 0) ? 1 : -1;

  // To judge if need turn around
  if (lin_vel_sign < 0)
  {
    size_t inverse_way_point_count = 0;
    for (size_t ini = carrot_point_ind + 1;
         ((ini < carrot_point_ind + 1 + num_point_in_path_look_ahead_for_turn_) &&
          (ini < path_used_for_turn_judgement_.size()));
         ini++)
    {
      Eigen::Vector3d next_target_middle = path_used_for_turn_judgement_[ini];

      geometry_msgs::Pose next_target_robot;
      tf::Vector3 next_target_tf_fixed;

      geometry_msgs::Pose next_target_fixed_pos_geo_msg2;
      next_target_fixed_pos_geo_msg2.position.x = next_target_middle[0];
      next_target_fixed_pos_geo_msg2.position.y = next_target_middle[1];
      next_target_fixed_pos_geo_msg2.position.z = next_target_middle[2];

      tf::pointMsgToTF(next_target_fixed_pos_geo_msg2.position, next_target_tf_fixed);
      tf::Vector3 next_target_robot_in_tf_ = fixed_to_robot_transform * next_target_tf_fixed;
      tf::pointTFToMsg(next_target_robot_in_tf_, next_target_robot.position);

      if (next_target_robot.position.x < 0)
      {
        inverse_way_point_count++;
      }
      else
      {
      }
    }
    if (inverse_way_point_count > min_inver_point - 1)
    {
      if_need_turn = true;
    }
    else if (inverse_way_point_count == path_used_for_turn_judgement_.size() - carrot_point_ind - 1)
    {
      if_need_turn = true;
    }
    else
    {
      if_need_turn = false;
    }
  }
  else
  {
    if_need_turn = false;
  }
  // get the target
  if (if_need_turn)
  {
    if (carrot_waypoint_robot.position.y > 0)
    {
      // left
      Target_attitude[0] = 1;
      double target_angle = std::atan(carrot_waypoint_robot.position.y / carrot_waypoint_robot.position.x);
      if (target_angle > 0)
      {
        target_angle = target_angle;
      }
      else
      {
        target_angle = target_angle + M_PI;
      }
      Target_attitude[1] = target_angle;
    }
    else if (carrot_waypoint_robot.position.y < 0)
    {
      // right
      Target_attitude[0] = -1;

      double target_angle = std::atan(carrot_waypoint_robot.position.y / carrot_waypoint_robot.position.x);
      if (target_angle > 0)
      {
        target_angle = M_PI - target_angle;
      }
      else
      {
        target_angle = -target_angle;
      }
      Target_attitude[1] = target_angle;
    }
    else
    {
      // background
      Target_attitude[0] = 1;
      Target_attitude[1] = M_PI;
    }
  }
  else
  {
  }

  double radius = std::pow(carrot_vec_robot.head(2).norm(), 2) / (2.0 * (std::abs(carrot_waypoint_robot.position.y) + 0.01));
  double radius_min = carrot_vec_robot.head(2).norm() / (2.0 * std::sin(curvature_scaling_angle_));
  double radius_max = carrot_vec_robot.head(2).norm() / (2.0 * std::sin(10.0 * M_PI / 180.0));

  if (delta_theta >= critical_angle_)
  {
    // std::cout << "Case 1: 0 lin vel" << std::endl;
    cmd_vel_.linear.x = 0.0;
    cmd_vel_.angular.z = max_ang_vel_ * ang_vel_sign;
    return true;
  }
  else if (delta_theta >= curvature_scaling_angle_)
  {
    cmd_vel_.linear.x = lin_vel_sign * desired_vel_ * (radius / radius_min);
    cmd_vel_.angular.z = ang_vel_sign * std::abs(cmd_vel_.linear.x) / radius;
    if (std::abs(cmd_vel_.angular.z) > max_ang_vel_)
    {
      cmd_vel_.angular.z = ang_vel_sign * max_ang_vel_;
    }
    return true;
  }
  else
  {
    double r = std::min(radius_max, radius);
    cmd_vel_.linear.x = lin_vel_sign * desired_vel_;
    cmd_vel_.angular.z = ang_vel_sign * std::abs(cmd_vel_.linear.x) / radius;
    if (std::abs(cmd_vel_.angular.z) > max_ang_vel_)
    {
      cmd_vel_.angular.z = ang_vel_sign * max_ang_vel_;
    }
    cmd_vel_.linear.x *= ((r - radius_min) / (radius_max - radius_min));
    return true;
  }
}

void Ppath_Tracker::clearPathTracked()
{
  std::lock_guard<std::mutex> lock1(PathMutex_);
  path_tracked_.clear();
}

void Ppath_Tracker::pauseTracker()
{
  std::lock_guard<std::mutex> lock2(PauseMutex_);
  if_pause = true;
}

void Ppath_Tracker::continueTracker()
{
  std::lock_guard<std::mutex> lock2(PauseMutex_);
  if_pause = false;
}
bool Ppath_Tracker::getStatus()
{
  // std::lock_guard<std::mutex> lock4(StatueMutex_);
  return status;
}
double Ppath_Tracker::get_x_of_cmd_vel()
{
  // std::lock_guard<std::mutex> lock6(CmdVelMutex_);
  return cmd_vel_.linear.x;
}
bool Ppath_Tracker::getIfPause()
{
  // std::lock_guard<std::mutex> lock2(PauseMutex_);
  return if_pause;
}
void Ppath_Tracker::stopTracker()
{
  std::lock_guard<std::mutex> lock3(StopMutex_);
  if_need_stop = true;
}
int Ppath_Tracker::getCarrotPointIdToOutput()
{
  // std::lock_guard<std::mutex> lock7(CarrotPointMutex_);
  return CarrotPointIdToOutput;
}

Eigen::Vector3d Ppath_Tracker::getCurrentCarrot()
{
  // std::lock_guard<std::mutex> lock7(CarrotPointMutex_);
  return CurrentCarrotPose;
}
bool Ppath_Tracker::getIfTrackSucceed()
{
  // std::lock_guard<std::mutex> lock5(IfSucceedMutex_);
  return if_track_succeed;
}
bool Ppath_Tracker::getIfNeedTurn()
{
  // std::lock_guard<std::mutex> lock8(IfNeedToTurnMutex_);
  return if_need_turn;
}

Eigen::Vector2d Ppath_Tracker::getTargetAttitude()
{
  // std::lock_guard<std::mutex> lock8(IfNeedToTurnMutex_);
  return Target_attitude;
}

void Ppath_Tracker::clearPathAndSetIfSucceedTrue()
{
  std::lock_guard<std::mutex> lock1(PathMutex_);
  std::lock_guard<std::mutex> lock5(IfSucceedMutex_);
  path_tracked_.clear();
  if_track_succeed = true;
}
bool Ppath_Tracker::IfNeedToUpdateTarget()
{
  return IfNeedUpdateNextTarget_;
}
double Ppath_Tracker::GetGoalReachingThresholdValue()
{
  return goal_reaching_threshold_;
}