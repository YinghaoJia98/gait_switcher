#ifndef GAIT_SWITCHER_H_
#define GAIT_SWITCHER_H_
#include <ros/ros.h>
#include <string>
#include <mutex>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <eigen3/Eigen/Dense>
#include <gait_switcher/deep_srv.h>

class GaitSwitcher
{
public:
    GaitSwitcher(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    void initialization();
    void setTimer();
    void reset();
    void SetStairTowardFrontMode();
    void SetStairTowardBackMode();
    void SetNormalMode();
    bool IfNeedSetVisionMotionMode(grid_map::GridMap ElevationMap,
                                   Eigen::Vector3d RobotPos,
                                   double LookAheadDistance,
                                   double RobotWidth,
                                   double angle,
                                   double ElevationMapResolution,
                                   std::string ElevationLayer,
                                   std::string TraversabilityLayer,
                                   std::string SurfaceNormalLayer,
                                   double SlopeCriticalValue,
                                   double TraversabilityThreshold,
                                   double SlopeThreshold,
                                   int BadPointThreshold,
                                   double TraversabilityCompareThreshold,
                                   double TraversabilityCompareMinThreshold);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber ElevationMapSubscriber_;
    ros::Subscriber CmdVelSubscriber_;
    void ElevationMapCallback(const grid_map_msgs::GridMap &msg);
    void CmdVelCallback(const geometry_msgs::Twist &msg);
    grid_map_msgs::GridMap ElevationMapMsg_;
    grid_map::GridMap ElevationMap_;
    geometry_msgs::Twist CmdVel_;

    bool IfMapInitiallized_;
    bool IfVelInitiallized_;

    int RobotMotionMode_; // 0 for normal, 1 for visual front, 2 for visual back.
    double LookAheadTime_;
    double RobotWidth_;

    std::string ElevationLayer_;
    std::string TraversabilityLayer_;
    std::string SurfaceNormalLayer_;
    std::string TraversabilityCompareLayer_;

    double SlopeCriticalValue_;
    double TraversabilityThreshold_;
    double SlopeThreshold_;
    int BadPointThreshold_;
    bool IfTestMode_;
    double TraversabilityCompareThreshold_;
    double TraversabilityCompareMinThreshold_;

    std::string WorldFrame_;
    std::string RobotFrame_;
    tf::TransformListener World_Robot_listener;
    tf::StampedTransform World_Robot_transform;

    ros::ServiceClient GaitSwitchClient_;

    ros::Timer GaitSwitcherTimer_;
    void GaitSwitcherTimerCallback(const ros::TimerEvent &event);

    mutable std::mutex ElevationMapMutex_;
    mutable std::mutex CmdVelMutex_;
};

#endif
