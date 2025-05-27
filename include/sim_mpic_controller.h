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

#ifndef SIM_MPIC_CONTROLLER_H
#define SIM_MPIC_CONTROLLER_H

#define PINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
#define PINOCCHIO_URDFDOM_USE_STD_SHARED_PTR

#pragma once

#include "mpic_controller.h"
#include "mpic_utils.h"

#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/frames-derivatives.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <vector>
#include <Eigen/Dense>

#include <controller_interface/controller_base.h>
#include <controller_interface/controller.h>
#include <pluginlib/class_list_macros.h>
#include <hardware_interface/joint_command_interface.h>

#include <ros/ros.h>

namespace lrt_mpic {
/**
   * This class implements the controller for the gazebo simulation
   */
class SimMpicController : public  controller_interface::Controller<hardware_interface::EffortJointInterface>
 {
  public:
    SimMpicController();
    ~SimMpicController();
    /* Init controller */
    bool init(hardware_interface::EffortJointInterface *robot_hw, ros::NodeHandle& node_handle) override;
    /* Start controller */
    void starting(const ros::Time&) override;
    /* Real time control loop */
    void update(const ros::Time&, const ros::Duration& period) override;
    /* Shut down controller */
    void stopping(const ros::Time &time) override;

  private:
    /* Get the current positions of the group of joints */
    void updateStates();

    /* Controller instance */
    boost::scoped_ptr<lrt_mpic::MpicController> mpicController_;

    /* robot joint handles */
    std::vector<hardware_interface::JointHandle> joint_handles_;
    struct Commands{double effort;}; // Last commanded position
    std::vector<Commands> commands_; // pre-allocated memory that is re-used to set the realtime buffer

    int Ndof_;      // number of robot joints
    int CartDof_;   // number of cartesian degrees of freedom (DOF)
    int Nstates_;   // number of MPC states
    int Nctr_;      // number of MPC inputs
    int Neqc_;      // number of MPC equality constraints
    int Nieqc_;     // number of MPC inequality constraints
    int Nhor_;      // number of MPC discretization steps
    double Thor_;   // length of MPC prediction horizon in seconds

    // robot states
    Eigen::VectorXd q_;
    Eigen::VectorXd dq_;
    Eigen::VectorXd qd_;
    Eigen::VectorXd tau_J_;
    Eigen::VectorXd tau_ext_;
    Eigen::VectorXd tau_nl_;
    Eigen::VectorXd pose_;
    Eigen::VectorXd cart_vel_;
    Eigen::VectorXd f_ext_;
    Eigen::VectorXd f_ext_death_;
    Eigen::MatrixXd jac_;
    Eigen::MatrixXd jac_transpose_pinv_;
    Eigen::MatrixXd jac_pinv_;
    Eigen::Vector3d position_;
    Eigen::Quaterniond orientation_;

    // kinematic model
    pinocchio::Data *data_;
    pinocchio::Model model_;
};

}  // namespace lrt_controller
#endif // SIM_MPIC_CONTROLLER_H
