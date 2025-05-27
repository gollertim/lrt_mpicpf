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

#ifndef GRIPPER_H
#define GRIPPER_H

// --- c++ ---
#include <Eigen/Dense>

// --- ROS ---
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <sensor_msgs/JointState.h>

// --- Gripper ---
#include <franka/gripper.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/StopAction.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>

namespace lrt_mpic
{
  /*
    Current state of the gripper
  */
  enum class GripperAnswer
  {
    stopped,
    running,
    success,
    failed
  };

  /*
    Gripper commands
  */
  enum class GripperAction
  {
    noaction,
    stop,
    move,
    grasp,
    homing
  };

  /**
     * This class is responsible for controlling the gripper
  */
  class GripperController
  {
  public:
    /* Constructor */
    GripperController();
    /* Destructor */
    ~GripperController();

    /* Init gripper node and connect to ROS action service */
    bool initGripper(ros::NodeHandle &node_handle);

    /* State machine to control the gripper commands */
    GripperAnswer updateGripper(bool stop, double width, double speed, double effort);

    /* Dynamic gripper states */
    Eigen::VectorXd gripper_position_;
    Eigen::VectorXd gripper_velocity_;
    Eigen::VectorXd gripper_effort_;

  private:
    /* ROS action clients */
    actionlib::SimpleActionClient<franka_gripper::MoveAction> move_client_;
    actionlib::SimpleActionClient<franka_gripper::StopAction> stop_client_;
    actionlib::SimpleActionClient<franka_gripper::GraspAction> grasp_client_;
    actionlib::SimpleActionClient<franka_gripper::HomingAction> homing_client_;

    /* Callback for gripper state */
    void gripperJointStateCallback(const sensor_msgs::JointState &msg);

    /* Subscriber for gripper states */
    ros::Subscriber gripper_sub_;

    /* Timeout for connection to ROS action server*/
    ros::Duration connection_timeout_;

    /* Current gripper command */
    GripperAction curgoal_action_;
    
    /* Desired gripper states */
    double inner_grasp_epsilon_;
    double outer_grasp_epsilon_;
    bool curgoal_stop_;
    double curgoal_width_;
    double curgoal_speed_;
    double curgoal_force_;
  };

} // namespace lrt_mpic
#endif // GRIPPER_H