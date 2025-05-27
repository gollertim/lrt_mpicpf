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

#include <real_mpic_controller.h>

namespace lrt_mpic
{

  RealMpicController::RealMpicController() {}
  RealMpicController::~RealMpicController()
  {
    stopping(ros::Time::now());
  }

  bool RealMpicController::init(hardware_interface::RobotHW *robot_hw,
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
          "RealMpicController: Invalid or no mpic fundamental parameter provided, aborting controller init!");
      return false;
    }

    // Init MPIC Controller
    mpicController_.reset(new MpicController(Ndof_, CartDof_, Nstates_, Nctr_, Neqc_, Nieqc_, Nhor_, Thor_));
    mpicController_->initMPIC(node_handle);

    std::string arm_id;
    if (!node_handle.getParam("/franka_control/arm_id", arm_id))
    {
      ROS_ERROR_STREAM("CartesianImpedanceExampleController: Could not read parameter arm_id");
      return false;
    }
    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != Ndof_)
    {
      ROS_ERROR(
          "RealMpicController: Invalid or no joint_names parameters provided, aborting "
          "controller init!");
      return false;
    }

    auto *model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr)
    {
      ROS_ERROR_STREAM(
          "CartesianImpedanceExampleController: Error getting model interface from hardware");
      return false;
    }
    try
    {
      model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
          model_interface->getHandle(arm_id + "_model"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM(
          "CartesianImpedanceExampleController: Exception getting model handle from interface: "
          << ex.what());
      return false;
    }

    auto *state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr)
    {
      ROS_ERROR_STREAM(
          "CartesianImpedanceExampleController: Error getting state interface from hardware");
      return false;
    }
    try
    {
      state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
          state_interface->getHandle(arm_id + "_robot"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM(
          "CartesianImpedanceExampleController: Exception getting state handle from interface: "
          << ex.what());
      return false;
    }

    auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr)
    {
      ROS_ERROR_STREAM(
          "CartesianImpedanceExampleController: Error getting effort joint interface from hardware");
      return false;
    }
    for (size_t i = 0; i < Ndof_; ++i)
    {
      try
      {
        joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
      }
      catch (const hardware_interface::HardwareInterfaceException &ex)
      {
        ROS_ERROR_STREAM(
            "CartesianImpedanceExampleController: Exception getting joint handles: " << ex.what());
        return false;
      }
    }

    // pinocchio
    std::string urdf_stream;
    if (!node_handle.getParam("/robot_description", urdf_stream))
    {
      ROS_ERROR("Failed to load urdf stream");
    }
    pinocchio::urdf::buildModelFromXML(urdf_stream, model_);
    data_ = new pinocchio::Data(model_);
    flange_id_ = model_.getFrameId("panda_link8");

    // Robot states
    q_ = Eigen::VectorXd::Zero(Ndof_);
    dq_ = Eigen::VectorXd::Zero(Ndof_);
    qd_ = Eigen::VectorXd::Zero(Ndof_);
    tau_J_ = Eigen::VectorXd::Zero(Ndof_);
    tau_ext_ = Eigen::VectorXd::Zero(Ndof_);

    pose_ = Eigen::VectorXd::Zero(7);
    cart_vel_ = Eigen::VectorXd::Zero(6);
    position_ = Eigen::VectorXd::Zero(3);
    f_ext_ = Eigen::VectorXd::Zero(CartDof_);
    f_ext_offset_ = Eigen::VectorXd::Zero(CartDof_);
    f_ext_death_ = Eigen::VectorXd::Zero(CartDof_);

    jac_ = Eigen::MatrixXd::Zero(CartDof_, Ndof_);
    jac_pinv_ = Eigen::MatrixXd::Zero(CartDof_, Ndof_);
    jac_transpose_pinv_ = Eigen::MatrixXd::Zero(Ndof_, CartDof_);

    // ft-sensor
    if (mpicController_->getCtrOpt()->fts)
    {
      boost::function<void(const geometry_msgs::WrenchStamped::ConstPtr &)> callback =
          boost::bind(&RealMpicController::wrenchCallback, this, _1);

      ros::SubscribeOptions subscribe_options;
      subscribe_options.init("/robotiq_ft_wrench", 1, callback);
      subscribe_options.transport_hints = ros::TransportHints().reliable().tcpNoDelay();
      wrench_sub_ = node_handle.subscribe(subscribe_options);

      auto msg = ros::topic::waitForMessage<geometry_msgs::WrenchStamped>("/robotiq_ft_wrench");

      ROS_INFO("[RealMpicController::init]: Initialisation of RealMpicController done");
    }

    return true;
  }

  void RealMpicController::starting(const ros::Time &time)
  {
    updateStates();

    mpicController_->startingMPIC(time);

    ROS_INFO("RealMpicController started");
  }
  void RealMpicController::update(const ros::Time &time, const ros::Duration &period)
  {
    // Update robot and grampc states
    updateStates();

    // get state variables
    franka::RobotState robot_state = state_handle_->getRobotState();
    // mass matrix
    std::array<double, 49> mass_array = model_handle_->getMass();
    // convert to Eigen
    Eigen::Map<Eigen::Matrix<double, 7, 7>> mass(mass_array.data());

    // coriolis vector
    std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
    // convert to Eigen
    Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data(), Ndof_);

    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(
        robot_state.tau_J_d.data(), Ndof_);

    // compute control
    // allocate variables
    static Eigen::VectorXd tau_mpic = Eigen::VectorXd::Zero(Ndof_);
    static Eigen::VectorXd tau_d = Eigen::VectorXd::Zero(Ndof_);

    tau_mpic = mpicController_->updateMPIC(time, period);
    // Desired torque
    tau_d << tau_mpic + coriolis;

    //   Saturate torque rate to avoid discontinuities
    tau_d << saturateTorqueRate(tau_d, tau_J_d);

    for (size_t i = 0; i < Ndof_; ++i)
    {
      joint_handles_[i].setCommand(tau_d[i]);
    }
  }
  void RealMpicController::stopping(const ros::Time &time)
  {
    mpicController_->stoppingMPIC(time);
    ROS_WARN("RealMpicController stopped");
  }
  void RealMpicController::updateStates()
  {
    franka::RobotState robot_state = state_handle_->getRobotState();

    Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J(robot_state.tau_J.data());

    q_ = q;
    dq_ = dq;
    tau_J_ = tau_J;

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

    if (!mpicController_->getCtrOpt()->fts)
    {
      // use intrinsic joint torque sensors otherwise f_ext_ is set by wrenchCallback()
      Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_ext_hat_filtered(robot_state.tau_ext_hat_filtered.data());
      tau_ext_ = tau_ext_hat_filtered;
      Eigen::Map<Eigen::Matrix<double, 6, 1>> f_ext_hat_filtered(robot_state.O_F_ext_hat_K.data());
      f_ext_ = Eigen::VectorXd::Zero(CartDof_);
      f_ext_ = f_ext_hat_filtered;
    }
    for (int i = 0; i < CartDof_; ++i)
    {
      f_ext_death_[i] = deathFilter(f_ext_[i], 1);
    }

    // Cartesian velocity
    cart_vel_ = jac_ * dq_;

    mpicController_->setStates(q_, dq_, tau_J_,
                               tau_ext_, f_ext_, f_ext_death_,
                               pose_, cart_vel_, position_, orientation_,
                               jac_, jac_pinv_, jac_transpose_pinv_);
  }
  Eigen::Matrix<double, 7, 1> RealMpicController::saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1> &tau_d_calculated,
      const Eigen::Matrix<double, 7, 1> &tau_J_d)
  {
    Eigen::Matrix<double, 7, 1> tau_d_saturated{};
    for (size_t i = 0; i < Ndof_; ++i)
    {
      double difference = tau_d_calculated[i] - tau_J_d[i];
      tau_d_saturated[i] =
          tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
    }
    return tau_d_saturated;
  }

  void RealMpicController::wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
  {
    f_ext_[0] = msg->wrench.force.x;
    f_ext_[1] = msg->wrench.force.y;
    f_ext_[2] = msg->wrench.force.z;
    f_ext_[3] = msg->wrench.torque.x;
    f_ext_[4] = msg->wrench.torque.y;
    f_ext_[5] = msg->wrench.torque.z;
  }

}
PLUGINLIB_EXPORT_CLASS(lrt_mpic::RealMpicController, controller_interface::ControllerBase)
