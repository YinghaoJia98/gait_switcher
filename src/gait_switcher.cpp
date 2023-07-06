#include "gait_switcher/gait_switcher.h"
GaitSwitcher::GaitSwitcher(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private)
{
  ROS_INFO("Hello, GaitSwitcher.");
  reset();
  initialization();
  setTimer();
}

void GaitSwitcher::initialization()
{
  std::string ElevationMapTopic_, CmdVelTopic_, ServiceName_;
  nh_.param<std::string>("/gait_switcher/gait_switcher_settings/ElevationMapTopic", ElevationMapTopic_,
                         std::string("/elevation_mapping/elevation_map_raw"));
  nh_.param<std::string>("/gait_switcher/gait_switcher_settings/CmdVelTopic", CmdVelTopic_,
                         std::string("/cmd_vel"));
  ElevationMapSubscriber_ = nh_.subscribe(ElevationMapTopic_.c_str(), 1,
                                          &GaitSwitcher::ElevationMapCallback, this);
  CmdVelSubscriber_ = nh_.subscribe(CmdVelTopic_.c_str(), 1,
                                    &GaitSwitcher::CmdVelCallback, this);

  nh_.param<std::string>("/gait_switcher/gait_switcher_settings/WorldFrame", WorldFrame_, std::string("world"));
  nh_.param<std::string>("/gait_switcher/gait_switcher_settings/RobotFrame", RobotFrame_, std::string("base_link"));

  nh_.param<std::string>("/gait_switcher/gait_switcher_settings/ServiceName", ServiceName_, std::string("basic_command"));
  GaitSwitchClient_ =
      nh_.serviceClient<gait_switcher::deep_srv>(ServiceName_.c_str());
  nh_.param<double>("/gait_switcher/gait_switcher_settings/LookAheadTime", LookAheadTime_, 1.0);
  nh_.param<double>("/gait_switcher/gait_switcher_settings/RobotWidth", RobotWidth_, 0.6);

  nh_.param<std::string>("/gait_switcher/gait_switcher_settings/ElevationLayer", ElevationLayer_, std::string("elevation"));
  nh_.param<std::string>("/gait_switcher/gait_switcher_settings/TraversabilityLayer", TraversabilityLayer_, std::string("step"));
  nh_.param<std::string>("/gait_switcher/gait_switcher_settings/SurfaceNormalLayer", SurfaceNormalLayer_, std::string("normal_z"));
  nh_.param<std::string>("/gait_switcher/gait_switcher_settings/TraversabilityCompareLayer", TraversabilityCompareLayer_, std::string("traversability"));

  nh_.param<double>("/gait_switcher/gait_switcher_settings/SlopeCriticalValue", SlopeCriticalValue_, 1.0);
  nh_.param<double>("/gait_switcher/gait_switcher_settings/TraversabilityThreshold", TraversabilityThreshold_, 0.8);
  nh_.param<double>("/gait_switcher/gait_switcher_settings/SlopeThreshold", SlopeThreshold_, 0.5);
  nh_.param<int>("/gait_switcher/gait_switcher_settings/BadPointThreshold", BadPointThreshold_, 10);
  nh_.param<bool>("/gait_switcher/gait_switcher_settings/IfTestMode", IfTestMode_, true);
  nh_.param<double>("/gait_switcher/gait_switcher_settings/TraversabilityCompareThreshold", TraversabilityCompareThreshold_, 0.8);
  nh_.param<double>("/gait_switcher/gait_switcher_settings/TraversabilityCompareMinThreshold", TraversabilityCompareMinThreshold_, 0.3);
}

void GaitSwitcher::setTimer()
{
  if (1)
  {
    double SwitcherCheckFps_;
    nh_.param<double>("/gait_switcher/gait_switcher_settings/SwitcherCheckFps", SwitcherCheckFps_, 1.0);
    double SwitcherCheckDuration_ = 1.0 / (SwitcherCheckFps_ + 0.00001);
    GaitSwitcherTimer_ = nh_.createTimer(ros::Duration(SwitcherCheckDuration_),
                                         &GaitSwitcher::GaitSwitcherTimerCallback, this);
  }
}

void GaitSwitcher::reset()
{
  IfMapInitiallized_ = false;
  IfVelInitiallized_ = false;
  CmdVel_.linear.x = 0;
  CmdVel_.linear.y = 0;
  CmdVel_.linear.z = 0;
  RobotMotionMode_ = 0;
}

void GaitSwitcher::ElevationMapCallback(const grid_map_msgs::GridMap &msg)
{
  std::lock_guard<std::mutex> lock1(ElevationMapMutex_);
  ElevationMapMsg_ = msg;
  grid_map::GridMapRosConverter::fromMessage(ElevationMapMsg_, ElevationMap_);
  if (!IfMapInitiallized_)
  {
    IfMapInitiallized_ = true;
  }
  else
  {
  }
  // ROS_INFO("Received elevation map.");
}
void GaitSwitcher::CmdVelCallback(const geometry_msgs::Twist &msg)
{
  std::lock_guard<std::mutex> lock2(CmdVelMutex_);
  CmdVel_ = msg;
  if (!IfVelInitiallized_)
  {
    IfVelInitiallized_ = true;
  }
  else
  {
  }
  // ROS_INFO("Received velocity data.");
}
void GaitSwitcher::GaitSwitcherTimerCallback(const ros::TimerEvent &event)
{
  std::lock_guard<std::mutex> lock1(ElevationMapMutex_);
  std::lock_guard<std::mutex> lock2(CmdVelMutex_);
  if ((!IfMapInitiallized_)) // || (!IfVelInitiallized_))
  {
    return;
  }
  Eigen::Vector3d CmdVelMiddle_(0, 0, 0);
  if (!IfTestMode_)
  {
    if (!IfVelInitiallized_)
    {
      return;
    }

    if ((CmdVel_.linear.x == 0) && (CmdVel_.linear.y == 0))
    {
      return;
    }
    // Eigen::Vector3d CmdVelMiddle_(CmdVel_.linear.x,
    //                               CmdVel_.linear.y,
    //                               CmdVel_.linear.z);
    CmdVelMiddle_ << CmdVel_.linear.x,
        CmdVel_.linear.y,
        CmdVel_.linear.z;
    CmdVel_.linear.x = 0;
    CmdVel_.linear.y = 0;
    CmdVel_.linear.z = 0;
  }
  else
  {
  }

  try
  {
    World_Robot_listener.waitForTransform(WorldFrame_.c_str(), RobotFrame_.c_str(), ros::Time(0), ros::Duration(1.0));
    World_Robot_listener.lookupTransform(WorldFrame_.c_str(), RobotFrame_.c_str(),
                                         ros::Time(0), World_Robot_transform);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("There might be something wrong when tryying update robot pose for gait switcher.");
    ROS_ERROR("%s", ex.what());
    // ros::Duration(1.0).sleep();
  }
  Eigen::Vector3d pos_middle(World_Robot_transform.getOrigin().x(), World_Robot_transform.getOrigin().y(), World_Robot_transform.getOrigin().z());
  tf::Quaternion q_robot_middle = World_Robot_transform.getRotation();
  double yaw_middle_ = tf::getYaw(q_robot_middle);
  // Eigen::Vector3d RobotPos_(pos_middle[0], pos_middle[1], yaw_middle_);
  Eigen::Vector3d RobotPos_(ElevationMapMsg_.info.pose.position.x,
                            ElevationMapMsg_.info.pose.position.y,
                            yaw_middle_);
  // current_position_ = pos_middle;
  double LookAheadDistance_;
  if (!IfTestMode_)
  {
    LookAheadDistance_ = CmdVelMiddle_[0] * LookAheadTime_;
  }
  else
  {
    LookAheadDistance_ = 0.5 * 2.0;
  }

  double ElevationMapResolution_ = ElevationMapMsg_.info.resolution;
  bool IfNeedSetVisionMode_ = IfNeedSetVisionMotionMode(ElevationMap_,
                                                        RobotPos_,
                                                        LookAheadDistance_,
                                                        RobotWidth_,
                                                        0,
                                                        ElevationMapResolution_,
                                                        ElevationLayer_,
                                                        TraversabilityLayer_,
                                                        SurfaceNormalLayer_,
                                                        SlopeCriticalValue_,
                                                        TraversabilityThreshold_,
                                                        SlopeThreshold_,
                                                        BadPointThreshold_,
                                                        TraversabilityCompareThreshold_,
                                                        TraversabilityCompareMinThreshold_);
  if (IfNeedSetVisionMode_)
  {
    // ROS_INFO("Vision Mode.");
    if (RobotMotionMode_ == 0)
    {
      if (CmdVelMiddle_[0] >= 0)
      {
        ROS_INFO("Switch to vision front mode.");
        SetStairTowardFrontMode();
        RobotMotionMode_ = 1;
      }
      else
      {
        ROS_INFO("Switch to vision back mode.");
        SetStairTowardBackMode();
        RobotMotionMode_ = 2;
      }
      // ROS_INFO("Switch To Vision Mode.");
      // setStairMode();
      // RobotMotionMode_ = 1;
    }
    if (RobotMotionMode_ == 1)
    {
      if (CmdVelMiddle_[0] < 0)
      {
        ROS_INFO("Switch to vision back mode.");
        SetStairTowardBackMode();
        RobotMotionMode_ = 2;
      }
    }
    if (RobotMotionMode_ == 2)
    {
      if (CmdVelMiddle_[0] >= 0)
      {
        ROS_INFO("Switch to vision front mode.");
        SetStairTowardBackMode();
        RobotMotionMode_ = 1;
      }
    }
  }
  else
  {
    // ROS_INFO("Normal Mode.");
    if ((RobotMotionMode_ == 1) || (RobotMotionMode_ == 2))
    {
      ROS_INFO("Switch To Normal Mode.");
      // Set Normal Mode
      SetNormalMode();
      RobotMotionMode_ = 0;
    }
  }
}
void GaitSwitcher::SetStairTowardFrontMode()
{
  try
  {
    gait_switcher::deep_srv deep_srv_;
    deep_srv_.request.a = 8;
    deep_srv_.request.b = 1;
    deep_srv_.request.x = 0;
    deep_srv_.request.y = 0;
    deep_srv_.request.yaw = 0;
    if (!GaitSwitchClient_.call(deep_srv_))
    {
      ROS_ERROR("[GaitSwitcher]: Service call failed: %s",
                GaitSwitchClient_.getService().c_str());
    }
    else
    {
      ROS_INFO("[GaitSwitcher]: Service call succeed: %s",
               GaitSwitchClient_.getService().c_str());
    }
    if (deep_srv_.response.c == -1)
    {
      deep_srv_.request.a = 6;
      deep_srv_.request.b = 0;
      deep_srv_.request.x = 0;
      deep_srv_.request.y = 0;
      deep_srv_.request.yaw = 0;
      GaitSwitchClient_.call(deep_srv_);
      ROS_INFO("[GaitSwitcher]: Service return -1.");
    }

    deep_srv_.request.a = 5;
    deep_srv_.request.b = 3;
    deep_srv_.request.x = 0;
    deep_srv_.request.y = 0;
    deep_srv_.request.yaw = 0;
    if (!GaitSwitchClient_.call(deep_srv_))
    {
      ROS_ERROR("[GaitSwitcher]: Second time. Service call failed: %s",
                GaitSwitchClient_.getService().c_str());
    }
    else
    {
      ROS_INFO("[GaitSwitcher]: Second time Service call succeed: %s",
               GaitSwitchClient_.getService().c_str());
    }
  }
  catch (const std::exception &e)
  {
    ROS_ERROR("[GaitSwitcher]: Service error: %s",
              GaitSwitchClient_.getService().c_str());
    std::cerr << e.what() << '\n';
  }
}

void GaitSwitcher::SetStairTowardBackMode()
{
  try
  {
    gait_switcher::deep_srv deep_srv_;
    deep_srv_.request.a = 8;
    deep_srv_.request.b = 2;
    deep_srv_.request.x = 0;
    deep_srv_.request.y = 0;
    deep_srv_.request.yaw = 0;
    if (!GaitSwitchClient_.call(deep_srv_))
    {
      ROS_ERROR("[GaitSwitcher]: Service call failed: %s",
                GaitSwitchClient_.getService().c_str());
    }
    else
    {
      ROS_INFO("[GaitSwitcher]: Service call succeed: %s",
               GaitSwitchClient_.getService().c_str());
    }
    if (deep_srv_.response.c == -1)
    {
      deep_srv_.request.a = 6;
      deep_srv_.request.b = 0;
      deep_srv_.request.x = 0;
      deep_srv_.request.y = 0;
      deep_srv_.request.yaw = 0;
      GaitSwitchClient_.call(deep_srv_);
      ROS_INFO("[GaitSwitcher]: Service return -1.");
    }

    deep_srv_.request.a = 5;
    deep_srv_.request.b = 3;
    deep_srv_.request.x = 0;
    deep_srv_.request.y = 0;
    deep_srv_.request.yaw = 0;
    if (!GaitSwitchClient_.call(deep_srv_))
    {
      ROS_ERROR("[GaitSwitcher]: Second time. Service call failed: %s",
                GaitSwitchClient_.getService().c_str());
    }
    else
    {
      ROS_INFO("[GaitSwitcher]: Second time Service call succeed: %s",
               GaitSwitchClient_.getService().c_str());
    }
  }
  catch (const std::exception &e)
  {
    ROS_ERROR("[GaitSwitcher]: Service error: %s",
              GaitSwitchClient_.getService().c_str());
    std::cerr << e.what() << '\n';
  }
}

void GaitSwitcher::SetNormalMode()
{
  try
  {
    gait_switcher::deep_srv deep_srv_;
    deep_srv_.request.a = 8;
    deep_srv_.request.b = 0;
    deep_srv_.request.x = 0;
    deep_srv_.request.y = 0;
    deep_srv_.request.yaw = 0;
    if (!GaitSwitchClient_.call(deep_srv_))
    {
      ROS_ERROR("[GaitSwitcher]: Service call failed: %s",
                GaitSwitchClient_.getService().c_str());
    }
    else
    {
      ROS_INFO("[GaitSwitcher]: Service call succeed: %s",
               GaitSwitchClient_.getService().c_str());
    }
    if (deep_srv_.response.c == -1)
    {
      deep_srv_.request.a = 6;
      deep_srv_.request.b = 0;
      deep_srv_.request.x = 0;
      deep_srv_.request.y = 0;
      deep_srv_.request.yaw = 0;
      GaitSwitchClient_.call(deep_srv_);
      ROS_INFO("[GaitSwitcher]: Service return -1.");
    }

    deep_srv_.request.a = 5;
    deep_srv_.request.b = 0;
    deep_srv_.request.x = 0;
    deep_srv_.request.y = 0;
    deep_srv_.request.yaw = 0;
    if (!GaitSwitchClient_.call(deep_srv_))
    {
      ROS_ERROR("[GaitSwitcher]: Second time. Service call failed: %s",
                GaitSwitchClient_.getService().c_str());
    }
    else
    {
      ROS_INFO("[GaitSwitcher]: Second time Service call succeed: %s",
               GaitSwitchClient_.getService().c_str());
    }
  }
  catch (const std::exception &e)
  {
    ROS_ERROR("[GaitSwitcher]: Service error: %s",
              GaitSwitchClient_.getService().c_str());
    std::cerr << e.what() << '\n';
  }
}

bool GaitSwitcher::IfNeedSetVisionMotionMode(grid_map::GridMap ElevationMap,
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
                                             double TraversabilityCompareMinThreshold)
{
  // double angle = RobotPos[2];
  double target_x_in_robot_frame_ = LookAheadDistance * cos(angle);
  double target_y_in_robot_frame_ = LookAheadDistance * sin(angle);
  double yaw_middle_ = RobotPos[2];
  double target_x_in_world_frame_ = RobotPos[0] +
                                    target_x_in_robot_frame_ * cos(yaw_middle_) -
                                    target_y_in_robot_frame_ * sin(yaw_middle_);
  double target_y_in_world_frame_ = RobotPos[1] +
                                    target_x_in_robot_frame_ * sin(yaw_middle_) +
                                    target_y_in_robot_frame_ * cos(yaw_middle_);
  Eigen::Vector2d start_pos_0(RobotPos[0], RobotPos[1]);
  Eigen::Vector2d end_pos_0(target_x_in_world_frame_, target_y_in_world_frame_);
  int size_in_y = RobotWidth / ElevationMapResolution;
  int size_y_used = size_in_y / 2;
  int bad_point_count_ = 0;

  for (int i = -size_y_used; i < size_y_used + 1; i++)
  {
    double y_change_in_robot_frame_ = i * ElevationMapResolution;
    double x_start_in_body_frame_ = y_change_in_robot_frame_ * (-sin(angle));
    double y_start_in_body_frame = y_change_in_robot_frame_ * (cos(angle));
    double x_start_in_world_frame = RobotPos[0] +
                                    x_start_in_body_frame_ * cos(yaw_middle_) -
                                    y_start_in_body_frame * sin(yaw_middle_);
    double y_start_in_world_frame = RobotPos[1] +
                                    x_start_in_body_frame_ * sin(yaw_middle_) +
                                    y_start_in_body_frame * cos(yaw_middle_);

    Eigen::Vector2d start_middle(x_start_in_world_frame, y_start_in_world_frame);
    Eigen::Vector2d end_middle = end_pos_0 - start_pos_0 + start_middle;

    for (grid_map::LineIterator iterator(ElevationMap, start_middle, end_middle);
         !iterator.isPastEnd(); ++iterator)
    {
      double elevation = ElevationMap.at(ElevationLayer.c_str(), *iterator);
      double traversability_score = ElevationMap.at(TraversabilityLayer.c_str(), *iterator);
      // double traversability_supplementary_score =
      //     traversability_map_fun.at(Traversability_supplementary_layer_.c_str(), *iterator);
      // traversability_score = std::max(traversability_score, traversability_supplementary_score);
      double traversability_compare_score =
          ElevationMap.at(TraversabilityCompareLayer_.c_str(), *iterator);
      // double SurfaceNormalZ = ElevationMap.at(SurfaceNormalLayer.c_str(), *iterator);
      if (elevation != elevation)
      {
        // bad_point_count_++;
        continue;
      }
      else
      {
      }

      if (traversability_score != traversability_score)
      {
        // bad_point_count_++;
        continue;
      }
      else
      {
      }

      if (traversability_compare_score != traversability_compare_score)
      {
        continue;
      }
      else
      {
      }

      // if (SurfaceNormalZ != SurfaceNormalZ)
      // {
      //   continue;
      // }
      // else
      // {
      // }

      // double SlopeAngle = acos(SurfaceNormalZ);
      // double SlopeScore = 0.0;

      // if (SlopeAngle < SlopeCriticalValue)
      // {
      //   SlopeScore = 1.0 - SlopeAngle / SlopeCriticalValue;
      // }
      // else
      // {
      //   SlopeScore = 0.0;
      // }
      if (traversability_score < TraversabilityThreshold)
      {
        continue;
      }
      else
      {
      }
      // if (SlopeScore < SlopeThreshold)
      // {
      //   bad_point_count_++;
      // }

      if ((traversability_compare_score < TraversabilityCompareThreshold) &&
          (traversability_compare_score > TraversabilityCompareMinThreshold))
      {
        bad_point_count_++;
      }
    }
  }

  //
  // ROS_INFO("Number of bad point is %d.", bad_point_count_);

  if (bad_point_count_ > BadPointThreshold)
  {
    // Need set vision mode
    return true;
  }
  else
  {
    // Need set normal mode
    return false;
  }

  return false;
}