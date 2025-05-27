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

#ifndef REAL_MPIC_CONTROLLER_H
#define REAL_MPIC_CONTROLLER_H

#pragma once

#include "mpic_controller.h"
#include "mpic_utils.h"

#include <memory>
#include <vector>
#include <Eigen/Dense>

#include <controller_interface/controller_base.h>
#include <controller_interface/controller.h>
#include <pluginlib/class_list_macros.h>
#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include <franka/robot_state.h>

#include <ros/ros.h>

namespace lrt_mpic
{
  /**
   * This class implements the controller for real robot
   */
  class RealMpicController : public controller_interface::MultiInterfaceController<
                                 franka_hw::FrankaModelInterface,
                                 hardware_interface::EffortJointInterface,
                                 franka_hw::FrankaStateInterface>
  {
  public:
    RealMpicController();
    ~RealMpicController();
    /* Init controller */
    bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;
    /* Start controller */
    void starting(const ros::Time &) override;
    /* Real time control loop */
    void update(const ros::Time &, const ros::Duration &period) override;
    /* Shut down controller */
    void stopping(const ros::Time &time) override;

  private:
    /* Get the currentstates of the real robot */
    void updateStates();

    /* Saturate torque rate to avoid discontinuities */
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(
        const Eigen::Matrix<double, 7, 1> &tau_d_calculated,
        const Eigen::Matrix<double, 7, 1> &tau_J_d); // NOLINT (readability-identifier-naming)

    /* Callback for external force/torque sensor */
    void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg);

    /* Controller instance */
    boost::scoped_ptr<lrt_mpic::MpicController> mpicController_;

    /* Interface handles */
    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
    std::vector<hardware_interface::JointHandle> joint_handles_;
    struct Commands
    {
      double effort;
    };                               // Last commanded position
    std::vector<Commands> commands_; // pre-allocated memory that is re-used to set the realtime buffer

    int Ndof_;    // number of robot joints
    int CartDof_; // number of cartesian degrees of freedom (DOF)
    int Nstates_; // number of MPC states
    int Nctr_;    // number of MPC inputs
    int Neqc_;    // number of MPC equality constraints
    int Nieqc_;   // number of MPC inequality constraints
    int Nhor_;    // number of MPC discretization steps
    double Thor_; // length of MPC prediction horizon in seconds

    // robot states
    Eigen::VectorXd q_;
    Eigen::VectorXd dq_;
    Eigen::VectorXd qd_;
    Eigen::VectorXd tau_J_;
    Eigen::VectorXd tau_ext_;
    Eigen::VectorXd pose_;
    Eigen::VectorXd cart_vel_;
    Eigen::VectorXd f_ext_;
    Eigen::VectorXd f_ext_offset_;
    Eigen::VectorXd f_ext_death_;
    Eigen::MatrixXd jac_;
    Eigen::MatrixXd jac_transpose_pinv_;
    Eigen::MatrixXd jac_pinv_;
    Eigen::Vector3d position_;
    Eigen::Quaterniond orientation_;

    const double filter_gain_{0.001};
    const double delta_tau_max_{1.0};

    // kinematic model
    pinocchio::Data *data_;
    pinocchio::Model model_;
    pinocchio::FrameIndex flange_id_;

    // subscriber
    ros::Subscriber wrench_sub_;
    boost::shared_ptr<geometry_msgs::WrenchStamped> msg;
  };

} // namespace lrt_controller
#endif // REAL_MPIC_CONTROLLER_H
