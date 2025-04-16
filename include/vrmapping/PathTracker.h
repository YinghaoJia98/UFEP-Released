#ifndef PATHTRACKER_H_
#define PATHTRACKER_H_

#include <unordered_map>
#include <eigen3/Eigen/Dense>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class Ppath_Tracker
{
public:
    Ppath_Tracker(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    ~Ppath_Tracker();
    void LoadParam();
    void Initialize();
    void setPathTracked(std::vector<Eigen::Vector3d> path_tracked_input);
    void Track_path(std::vector<Eigen::Vector3d> &path_tracked);
    double calculateDistance(const Eigen::Vector3d p1, const Eigen::Vector3d p2);
    bool computeCommandVelocity(std::vector<Eigen::Vector3d> &PathTracked);
    void clearPathTracked();
    void pauseTracker();
    void continueTracker();
    bool getStatus();
    double get_x_of_cmd_vel();
    bool getIfPause();
    void stopTracker();
    int getCarrotPointIdToOutput();
    Eigen::Vector3d getCurrentCarrot();
    bool getIfTrackSucceed();
    bool getIfNeedTurn();
    Eigen::Vector2d getTargetAttitude();
    void clearPathAndSetIfSucceedTrue();
    double RobotHeight_;
    bool IfNeedToUpdateTarget();
    double GetGoalReachingThresholdValue();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher carrot_point_pub_;
    geometry_msgs::Twist cmd_vel_;
    bool if_tracker_enable;
    bool if_need_stop;
    bool start_execution_;
    std::vector<Eigen::Vector3d> path_tracked_;
    Eigen::Vector3d current_position_;
    double goal_reaching_threshold_;
    bool if_track_succeed;
    double look_ahead_distance_;
    bool IfTryFindInABigDistance_;
    double look_ahead_distanceInMax_;
    int look_ahead_verticesInMax_;
    double look_ahead_error_margin_;
    std::shared_ptr<tf::TransformListener> listener_;
    double curvature_scaling_angle_;
    double critical_angle_;
    double max_ang_vel_;
    double desired_vel_;
    bool if_pause;
    bool status;
    int CarrotPointIdToOutput;
    Eigen::Vector3d CurrentCarrotPose;
    bool if_need_turn;
    int num_point_in_path_look_ahead_for_turn_;
    int min_inver_point;
    Eigen::Vector2d Target_attitude;
    bool IfAheadThePathAllTheTime_;

    bool IfNeedUpdateNextTarget_;
    int RemainVerticesNumMinValue_;
    int RemainVerticesNumMaxValue_;
    double DistanceOfTargetMinValue_;

    std::string World_Frame_;
    std::string Robot_Frame_;
    tf::TransformListener World_Robot_listener;
    tf::StampedTransform World_Robot_transform;

    ros::Timer Path_Tracker_Timer;
    void PathTrackTimerCallBack(const ros::TimerEvent &event);

    ros::Timer Update_Current_Robot_Pose_Timer;
    void UpdateRobotPoseCallBack(const ros::TimerEvent &event);

    mutable std::mutex PathMutex_;
    mutable std::mutex PauseMutex_;
    mutable std::mutex StopMutex_;
    mutable std::mutex StatueMutex_;
    mutable std::mutex IfSucceedMutex_;
    mutable std::mutex CmdVelMutex_;
    mutable std::mutex CarrotPointMutex_;
    mutable std::mutex IfNeedToTurnMutex_;
};

#endif
