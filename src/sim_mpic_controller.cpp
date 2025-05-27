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

#include <sim_mpic_controller.h>

namespace lrt_mpic
{

  SimMpicController::SimMpicController()
  {
    ROS_INFO("[SimMpicController:SimMpicController]: Constructor called!");
  }
  SimMpicController::~SimMpicController()
  {
    ros::shutdown();
    stopping(ros::Time::now());
    ROS_INFO("[SimMpicController~:SimMpicController]: Destructor called!");
  }

  bool SimMpicController::init(hardware_interface::EffortJointInterface *robot_hw,
                               ros::NodeHandle &node_handle)
  {
    if (!node_handle.getParam("/mpic/Ndof", Ndof_) ||
        !node_handle.getParam("/mpic/CartDof", CartDof_) ||
        !node_handle.getParam("/mpic/Nstates", Nstates_) ||
        !node_handle.getParam("/mpic/Nctr", Nctr_) ||
        !node_handle.getParam("/mpic/Neqc", Neqc_) ||
        !node_handle.getParam("/mpic/Nieqc", Nieqc_) ||
        !node_handle.getParam("/grampc/opt/Nhor", Nhor_) ||
        !node_handle.getParam("/grampc/param/Thor", Thor_))
    {
      ROS_ERROR(
          "[SimMpicController:init]: Invalid or no mpic fundamental parameter provided, aborting controller init!");
      return false;
    }

    // Init MPIC Controller
    ROS_INFO("[SimMpicController:init]: Start init MPIC Controller");
    mpicController_.reset(new MpicController(Ndof_, CartDof_, Nstates_, Nctr_, Neqc_, Nieqc_, Nhor_, Thor_));
    mpicController_->initMPIC(node_handle);
    ROS_INFO("[SimMpicController:init]: Init MPIC Controller done");

    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != Ndof_)
    {
      ROS_ERROR(
          "[SimMpicController:init]: Invalid or no joint_names parameters provided, aborting "
          "controller init!");
      return false;
    }

    // init hardware interface
    for (unsigned int i = 0; i < Ndof_; i++)
    {
      try
      {
        // get the joint object to use in the realtime loop
        joint_handles_.push_back(robot_hw->getHandle(joint_names[i]));
      }
      catch (const hardware_interface::HardwareInterfaceException &e)
      {
        ROS_ERROR_STREAM("[SimMpicController:init]: Exception thrown: " << e.what());
        return false;
      }
    }

    // Initialization of the KDL Library
    urdf::Model urdf;
    std::vector<urdf::JointConstSharedPtr> joints_urdf;
    if (!urdf.initParamWithNodeHandle("/robot_description", node_handle))
    {
      ROS_ERROR("[SimMpicController:init]: Failed to parse urdf file");
      return false;
    }
    for (unsigned int i = 0; i < Ndof_; i++)
    {
      joints_urdf.push_back(urdf.getJoint(joint_names[i])); // get URDF Object
      if (!joints_urdf[i])
      {
        ROS_ERROR("[SimMpicController:init]: Could not find joint '%s' in urdf", joint_names[i].c_str());
        return false;
      }
    }
    std::string urdf_stream;
    if (!node_handle.getParam("/robot_description", urdf_stream))
    {
      ROS_ERROR("[SimMpicController:init]: Failed to load urdf stream");
    }
    pinocchio::urdf::buildModelFromXML(urdf_stream, model_);
    data_ = new pinocchio::Data(model_);

    // robot states
    q_ = Eigen::VectorXd::Zero(Ndof_);
    dq_ = Eigen::VectorXd::Zero(Ndof_);
    qd_ = Eigen::VectorXd::Zero(Ndof_);
    tau_J_ = Eigen::VectorXd::Zero(Ndof_);
    tau_ext_ = Eigen::VectorXd::Zero(Ndof_);
    tau_nl_ = Eigen::VectorXd::Zero(Ndof_);
    // s_ = Eigen::VectorXd::Zero(1);

    pose_ = Eigen::VectorXd::Zero(7);
    cart_vel_ = Eigen::VectorXd::Zero(6);
    f_ext_ = Eigen::VectorXd::Zero(CartDof_);
    f_ext_death_ = Eigen::VectorXd::Zero(CartDof_);

    jac_ = Eigen::MatrixXd::Zero(CartDof_, Ndof_);
    jac_pinv_ = Eigen::MatrixXd::Zero(CartDof_, Ndof_);
    jac_transpose_pinv_ = Eigen::MatrixXd::Zero(Ndof_, CartDof_);

    ROS_INFO("[SimMpicController:init]: Inititialisation done");
    return true;
  }

  void SimMpicController::starting(const ros::Time &time)
  {

    // Update robot and grampc states
    ROS_INFO("[SimMpicController:starting]: Initial state update started");
    updateStates();
    ROS_INFO("[SimMpicController:starting]: Initial state update done");

    mpicController_->startingMPIC(time);

    ROS_INFO("[SimMpicController:starting]:  SimMpicController started");
  }
  void SimMpicController::update(const ros::Time &time, const ros::Duration &period)
  {

    // Update robot and grampc states
    updateStates();

    // compute control
    // allocate variables
    static Eigen::VectorXd tau_mpic = Eigen::VectorXd::Zero(Ndof_);
    static Eigen::VectorXd tau_d = Eigen::VectorXd::Zero(Ndof_);

    tau_mpic = mpicController_->updateMPIC(time, period);
    tau_d = tau_mpic + tau_nl_;

    for (size_t i = 0; i < Ndof_; ++i)
    {
      joint_handles_[i].setCommand(tau_d[i]);
    }
  }
  void SimMpicController::stopping(const ros::Time &time)
  {
    mpicController_->stoppingMPIC(time);
    ROS_WARN("[SimMpicController:starting]:  SimMpicController stopped");
  }
  void SimMpicController::updateStates()
  {
    for (unsigned int i = 0; i < Ndof_; i++)
    {
      q_[i] = joint_handles_[i].getPosition();
      dq_[i] = joint_handles_[i].getVelocity();
      tau_J_[i] = joint_handles_[i].getEffort();
    }
    // forward kinematic
    pinocchio::forwardKinematics(model_, *data_, q_, dq_);
    pinocchio::FrameIndex frame_id = model_.getFrameId("panda_link8");
    Eigen::Matrix3d rot_mat = data_->oMf[frame_id].rotation();
    orientation_ = rot_mat;
    position_ = data_->oMf[frame_id].translation();
    pose_.head(3) = position_;
    pose_[3] = orientation_.w();
    pose_.tail(3) = orientation_.vec();
    // jacobian
    pinocchio::computeFrameJacobian(model_, *data_, q_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, jac_);
    pseudoInverse(jac_.transpose(), jac_transpose_pinv_);
    pseudoInverse(jac_, jac_pinv_);
    // nonlinear effects
    tau_nl_ = pinocchio::rnea(model_, *data_, q_, dq_, Eigen::VectorXd::Zero(Ndof_));

    mpicController_->setStates(q_, dq_, tau_J_,
                               tau_ext_, f_ext_, f_ext_death_,
                               pose_, cart_vel_, position_, orientation_,
                               jac_, jac_pinv_, jac_transpose_pinv_);
  }

} // namespace lrt_mpic

PLUGINLIB_EXPORT_CLASS(lrt_mpic::SimMpicController, controller_interface::ControllerBase)
