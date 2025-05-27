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

#ifndef MPIC_CONTROLLER_H
#define MPIC_CONTROLLER_H

#pragma once
#include "mpic_problem_description.hpp"

#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>

#include "grampc.hpp"
#include "grampc_util_cpp.h"

#include "mpic_utils.h"
#include "mp_framework.h"
#include "mp_library/generic_mpic.hpp"
#include "skill_framework.h"
#include "task_framework.h"
#include "gripper.h"
#include "mpic_statemachine.h"
#include "mpic_exception.h"

#include <chrono>

#include "lock_free_queue.hpp"

namespace lrt_mpic
{
  /**
     * This class is the basic controller for the Franka-Emika robot
  */
  class MpicController
  {
  public:
    MpicController(int Ndof, int CartDof_, int Nstates, int Nctr, int Neqc, int Nieqc, int Nhor, double Thor);
    ~MpicController();
    bool initMPIC(ros::NodeHandle &node_handle);
    void startingMPIC(const ros::Time &);
    Eigen::VectorXd updateMPIC(const ros::Time &time, const ros::Duration &period);
    void stoppingMPIC(const ros::Time &time);
    void setStates(const Eigen::VectorXd &q, const Eigen::VectorXd &dq, const Eigen::VectorXd &tau_J,
                   const Eigen::VectorXd &tau_ext, const Eigen::VectorXd &f_ext, const Eigen::VectorXd &f_ext_death,
                   const Eigen::VectorXd &pose, const Eigen::VectorXd &cart_vel, const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation,
                   const Eigen::MatrixXd &jac, const Eigen::MatrixXd &jac_pinv, const Eigen::MatrixXd &jac_transp_pinv);
    controlOptions *getCtrOpt();

  private:
    /**
     * \brief Configure grampc problem
     */
    lrt::MPICProblemDescriptionPtr configureProblem();
    /**
     * \brief Configure grampc solver
     */
    grampc::GrampcPtr configureSolver(const lrt::MPICProblemDescriptionPtr &problem);
    /**
     * \brief Get Parameter from the parameter server
     */
    bool readParameters();
    /**
     * \brief Get environment from the parameter server
     */
    bool readEnvir();
    /**
     * \brief Update the current Stiffness
     */
    void updateStiffness();
    /**
     * \brief Publish important states as topics
     */
    void publishStates();
    /**
     * \brief Publish important states as topics
     */
    void writeStates();
    /**
     * \brief Initialize MPC-Thread
     */
    void mpcThreadInit();
    /**
     * \brief Execute MPC-Thread oncee
     */
    void mpcThreadExecute();
    /**
     * \brief Update vector of current and next MP
     */
    void updateMPvec(std::vector<std::shared_ptr<ManipulationPrimitive>> &MPvec, const std::shared_ptr<Skill> &skill);
    /**
     * \brief Configures the path for the move to MP
     */
    void configureMoveTo(ManipulationPrimitive *MP);
    /**
     * \brief Data logging function
     */
    void loggingThreadFunction();

    //--- Grampc ---
    lrt::MPICProblemDescriptionPtr problem_description_;
    grampc::GrampcPtr solver_;
    std::thread mpc_thread_; // thread for solving the optimization problem
    typeRNum *xnow_;         // vector of actual MPC states

    int Ndof_;    // number of robot joints
    int CartDof_; // number of cartesian degrees of freedom (DOF)
    int Nstates_; // number of MPC states
    int Nctr_;    // number of MPC inputs
    int Neqc_;    // number of MPC equality constraints
    int Nieqc_;   // number of MPC inequality constraints
    int Nhor_;    // number of MPC discretization steps
    double Thor_; // length of MPC prediction horizon in seconds

    // general parameters
    controlOptions ctrOpt_;           // optional settings for MPC and task control
    controlGain ctrGains_;            // gains of underlying PD controller
    Eigen::MatrixXd Ke_;              // environment stiffness
    Eigen::MatrixXd Kges_;            // overall stiffness
    bool use_pbd_controller_ = false; // flag for controller choice: true = PBD-Controller, false = MPIC-PF

    // measured data e.g. from robot or sensors
    struct
    {
      Eigen::VectorXd q_;
      Eigen::VectorXd dq_;
      Eigen::VectorXd tau_J_;
      Eigen::VectorXd tau_ext_;
      Eigen::VectorXd pose_;
      Eigen::Vector3d position_;
      Eigen::Vector3d position_elbow_;
      Eigen::Quaterniond orientation_;
      Eigen::Matrix3d rot_mat_;
      Eigen::VectorXd cart_vel_;
      Eigen::VectorXd f_ext_;
      Eigen::VectorXd f_ext_death_;
      Eigen::MatrixXd jac_;
      Eigen::MatrixXd jac_transpose_pinv_;
      Eigen::MatrixXd jac_pinv_;
    } meas_mpc_data_{};

    // predicted data of the MPC
    struct
    {
      Eigen::VectorXd q_mpc_;
      Eigen::VectorXd dq_mpc_;
      Eigen::VectorXd q_ref_mpc_;
      Eigen::VectorXd dq_ref_mpc_;
      Eigen::VectorXd u_mpc_;
      Eigen::VectorXd F_mpc_;
      Eigen::VectorXd s_mpc_;
      Eigen::VectorXd ds_mpc_;
      Eigen::VectorXd J_mpc_;
      int dt_mpc_; // computation time
    } pred_mpc_data_{};

    // shared data between ROS-controller and MPC-thread
    struct
    {
      std::mutex mutex_;
      unsigned int cnt_mpc_;
      double t0_;
      Eigen::VectorXd x0_;
      Eigen::VectorXd xnext_;
      Eigen::VectorXd unext_;
      Eigen::VectorXd J_;
      setPoint setPnt_;
      weights wghts_;
      controlOptions ctrOpt_;
      controlGain ctrGains_;
      constraints constr_;
      Eigen::MatrixXd Kges_;
      double dt_;
      double elapsed_time_;
      std::vector<std::shared_ptr<ManipulationPrimitive>> actMPvec_;
      bool task_updated_;
      TaskMeasurement task_meas_;
      double s_f_;
    } shared_mpc_data_{};

    // kinematic model
    pinocchio::Data *data_;
    pinocchio::Model model_;
    pinocchio::FrameIndex flange_id_;

    // --- Gripper ---
    std::shared_ptr<GripperController> gripper_;

    // ROS publisher and subscribers
    std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>> wrench_pub_;
    std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>> pose_pub_;
    std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped>> cart_vel_pub_;

    // Task
    std::string task_folder_path_;
    std::string task_file_name_;
    std::string exception_folder_path_;
    std::shared_ptr<Task> my_task_;
    std::shared_ptr<Skill> my_skill_;
    setPoint setPnt_; // control objective as set point

    // Excption sequences
    struct
    {
      std::shared_ptr<ManipulationPrimitive> stopMp_;
      std::shared_ptr<ManipulationPrimitive> moveToMp_;
    } exception_portfolio_{};
    TaskStatemachine smach_;

    // times and counters
    ros::Duration elapsed_time_;
    unsigned int loop_count_;
    int time_update_;

    // --- Data Logging ---
    std::string log_path_;
    LockFreeQueue<std::string> log_queue_;
    std::thread logging_thread_;
    std::atomic<bool> stop_logging_;
    std::ofstream data_meas_;
  };

} // namespace lrt_controller
#endif // MPIC_CONTROLLER_H
