/* This file is part of LRT-MPICPF - (https://github.com/gollertim/lrt_mpicpf)
 *
 * LRT-MPICPF -- A software framework for model predictive interaction control based on a path-following formulation (MPIC-PF) for robotic manipulation tasks
 *
 * Copyright 2025 Tim Goller, Tobias Gold, Andreas Voelz, Knut Graichen.
 * All rights reserved.
 *
 * LRT-MPICPF is distributed under the BSD-3-Clause license, see LICENSE.txt
 *
 */

#include "gripper.h"

namespace lrt_mpic
{

  GripperController::GripperController()
      : move_client_("/franka_gripper/move", true),
        stop_client_("/franka_gripper/stop", true),
        grasp_client_("/franka_gripper/grasp", true),
        homing_client_("/franka_gripper/homing", true)
  {
    gripper_position_ = Eigen::VectorXd::Zero(2);
    gripper_velocity_ = Eigen::VectorXd::Zero(2);
    gripper_effort_ = Eigen::VectorXd::Zero(2);
    connection_timeout_ = ros::Duration(2.0);
    inner_grasp_epsilon_ = 0.005;
    outer_grasp_epsilon_ = 0.005;
  }

  GripperController::~GripperController() {}

  bool GripperController::initGripper(ros::NodeHandle &node_handle)
  {
    gripper_sub_ = node_handle.subscribe("/franka_gripper/joint_states", 1, &GripperController::gripperJointStateCallback, this);

    if (move_client_.waitForServer(connection_timeout_))
    {
      ROS_INFO("[Gripper::initGripper]: Gripper actions connected");
    }
    else
    {
      ROS_WARN("[Gripper::initGripper]: Failed to connect to action clients. Programm will succeed without moving gripper");
    }
    return true;
  }

  GripperAnswer GripperController::updateGripper(bool stop, double width, double speed, double force)
  {
    if (curgoal_stop_ == stop &&
        curgoal_width_ == width &&
        curgoal_speed_ == speed &&
        curgoal_force_ == force)
    { // check success
      switch (curgoal_action_)
      {
      case GripperAction::noaction:
        return GripperAnswer::stopped;
        break;

      case GripperAction::stop:
        if (stop_client_.getState() == actionlib::SimpleClientGoalState::ACTIVE)
        {
          return GripperAnswer::running;
        }
        if (stop_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          curgoal_action_ = GripperAction::noaction;
          return GripperAnswer::success;
        };
        break;

      case GripperAction::move:
        if (move_client_.getState() == actionlib::SimpleClientGoalState::ACTIVE)
        {
          return GripperAnswer::running;
        }
        if (move_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          curgoal_action_ = GripperAction::noaction;
          return GripperAnswer::success;
        };
        break;

      case GripperAction::grasp:
        if (grasp_client_.getState() == actionlib::SimpleClientGoalState::ACTIVE)
        {
          return GripperAnswer::running;
        }
        if (grasp_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          curgoal_action_ = GripperAction::noaction;
          return GripperAnswer::success;
        };
        break;

      case GripperAction::homing:
        if (homing_client_.getState() == actionlib::SimpleClientGoalState::ACTIVE)
        {
          return GripperAnswer::running;
        }
        if (homing_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          curgoal_action_ = GripperAction::noaction;
          return GripperAnswer::success;
        };
        break;

      default:
        return GripperAnswer::failed;
        break;
      }
      return GripperAnswer::failed;
    }
    else
    { // new command

      if (speed <= 0)
      { // probaly no gripper action specified in current MP
        if (curgoal_action_ == GripperAction::noaction)
        {
          curgoal_width_ = width;
          curgoal_speed_ = speed;
          curgoal_force_ = force;
          curgoal_action_ = GripperAction::noaction;
          return GripperAnswer::stopped;
        }
        else
        {
          ROS_WARN("[Gripper::updateGripper]: Cannot do nothing without terminating previous action");
        }
      }

      if (stop)
      { // force stop all running actions
        franka_gripper::StopGoal stop_goal;
        stop_client_.sendGoal(stop_goal);
        curgoal_stop_ = stop;
        curgoal_action_ = GripperAction::stop;
        return GripperAnswer::running;
      }

      if ((force > 0) && (width < curgoal_width_))
      { // grasp
        franka_gripper::GraspGoal grasp_goal;
        grasp_goal.width = width;
        grasp_goal.speed = speed;
        grasp_goal.force = force;
        grasp_goal.epsilon.inner = inner_grasp_epsilon_;
        grasp_goal.epsilon.outer = outer_grasp_epsilon_;
        grasp_client_.sendGoal(grasp_goal);
        curgoal_width_ = width;
        curgoal_speed_ = speed;
        curgoal_force_ = force;
        curgoal_action_ = GripperAction::grasp;
        return GripperAnswer::running;
      }

      if (curgoal_width_ != width)
      { // move
        franka_gripper::MoveGoal move_goal;
        move_goal.width = width;
        move_goal.speed = speed;
        move_client_.sendGoal(move_goal);
        curgoal_width_ = width;
        curgoal_speed_ = speed;
        curgoal_force_ = force;
        curgoal_action_ = GripperAction::move;
        return GripperAnswer::running;
      }

      curgoal_stop_ = stop;
      curgoal_width_ = width;
      curgoal_speed_ = speed;
      curgoal_force_ = force;
      ROS_WARN("[Gripper::updateGripper]: Unspecified argument combination");
      return GripperAnswer::failed;
    }
  }

  void GripperController::gripperJointStateCallback(const sensor_msgs::JointState &msg)
  {
    gripper_position_[0] = msg.position[0];
    gripper_position_[1] = msg.position[1];

    gripper_velocity_[0] = msg.velocity[0];
    gripper_velocity_[1] = msg.velocity[1];

    gripper_effort_[0] = msg.effort[0];
    gripper_effort_[1] = msg.effort[1];
  }

} // namespace lrt_mpic