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

#include <mpic_controller.h>
#include <math.h>
namespace lrt_mpic
{
  MpicController::MpicController(int Ndof, int CartDof, int Nstates, int Nctr, int Neqc, int Nieqc, int Nhor, double Thor)
      : Ndof_(Ndof), CartDof_(CartDof), Nstates_(Nstates), Nctr_(Nctr), Neqc_(Neqc), Nieqc_(Nieqc), Nhor_(Nhor), Thor_(Thor), stop_logging_(false)
  {
    // Initialization
    // PD Controller gains
    ctrGains_.Kpd = Eigen::MatrixXd::Zero(Ndof_, Ndof_);
    ctrGains_.Dpd = Eigen::MatrixXd::Zero(Ndof_, Ndof_);
    ctrGains_.Dctr_inv = Eigen::MatrixXd::Zero(Ndof_, Ndof_);
    ctrGains_.K_D = Eigen::MatrixXd::Zero(Ndof_, Ndof_);

    // Environment
    Ke_ = Eigen::MatrixXd::Zero(CartDof_, CartDof_);

    // Overal Stiffness
    Kges_ = Eigen::MatrixXd::Zero(CartDof_, CartDof_);

    // initialize task
    task_folder_path_ = "";
    task_file_name_ = "";
    exception_folder_path_ = "";
    my_task_ = std::make_shared<Task>(Ndof_, CartDof_);
    my_skill_ = std::make_shared<Skill>(Ndof_, CartDof_);

    // measured data e.g. robot state, jacobians, ...
    meas_mpc_data_.q_ = Eigen::VectorXd::Zero(Ndof_);
    meas_mpc_data_.dq_ = Eigen::VectorXd::Zero(Ndof_);
    meas_mpc_data_.tau_J_ = Eigen::VectorXd::Zero(Ndof_);
    meas_mpc_data_.tau_ext_ = Eigen::VectorXd::Zero(Ndof_);
    meas_mpc_data_.pose_ = Eigen::VectorXd::Zero(7);
    meas_mpc_data_.position_ = Eigen::VectorXd::Zero(7);
    meas_mpc_data_.position_elbow_ = Eigen::VectorXd::Zero(7);
    meas_mpc_data_.orientation_.setIdentity();
    meas_mpc_data_.rot_mat_ = Eigen::MatrixXd::Zero(3, 3);
    meas_mpc_data_.cart_vel_ = Eigen::VectorXd::Zero(6);
    meas_mpc_data_.f_ext_ = Eigen::VectorXd::Zero(CartDof_);
    meas_mpc_data_.f_ext_death_ = Eigen::VectorXd::Zero(CartDof_);
    meas_mpc_data_.jac_ = Eigen::MatrixXd::Zero(CartDof_, Ndof_);
    meas_mpc_data_.jac_pinv_ = Eigen::MatrixXd::Zero(Ndof_, CartDof_);
    meas_mpc_data_.jac_transpose_pinv_ = Eigen::MatrixXd::Zero(Ndof_, CartDof_);

    // predicted MPC data
    pred_mpc_data_.q_mpc_ = Eigen::VectorXd::Zero(Ndof_);
    pred_mpc_data_.dq_mpc_ = Eigen::VectorXd::Zero(Ndof_);
    pred_mpc_data_.q_ref_mpc_ = Eigen::VectorXd::Zero(Ndof_);
    pred_mpc_data_.dq_ref_mpc_ = Eigen::VectorXd::Zero(Ndof_);
    pred_mpc_data_.u_mpc_ = Eigen::VectorXd::Zero(Ndof_ + 1); // dq_ref + ds
    pred_mpc_data_.F_mpc_ = Eigen::VectorXd::Zero(CartDof_);
    pred_mpc_data_.s_mpc_ = Eigen::VectorXd::Zero(1);
    pred_mpc_data_.ds_mpc_ = Eigen::VectorXd::Zero(1);
    pred_mpc_data_.J_mpc_ = Eigen::VectorXd::Zero(2);
    pred_mpc_data_.dt_mpc_ = 0; // computation time

    // Shared Data between control and MPC thread
    shared_mpc_data_.x0_ = Eigen::VectorXd::Zero(Nstates_);
    shared_mpc_data_.xnext_ = Eigen::VectorXd::Zero(Nstates_);
    shared_mpc_data_.unext_ = Eigen::VectorXd::Zero(Nctr_);
    shared_mpc_data_.J_ = Eigen::VectorXd::Zero(2);
    shared_mpc_data_.ctrGains_ = ctrGains_;
    shared_mpc_data_.Kges_ = Kges_;
    shared_mpc_data_.elapsed_time_ = 0.0;
    shared_mpc_data_.actMPvec_.push_back(std::make_shared<GenericMPIC>()); // first element
    shared_mpc_data_.actMPvec_.push_back(std::make_shared<GenericMPIC>()); // second element
    shared_mpc_data_.task_updated_ = false;
    shared_mpc_data_.task_meas_;
    shared_mpc_data_.s_f_ = 0.0;

    // control objective
    initSetPoint(shared_mpc_data_.setPnt_, Ndof_, CartDof_);

    initSetPoint(setPnt_, Ndof_, CartDof_);

    // gripper initialisation
    gripper_ = std::make_shared<lrt_mpic::GripperController>();

    // logging thread
    logging_thread_ = std::thread(&MpicController::loggingThreadFunction, this);
  }

  MpicController::~MpicController()
  {
    data_meas_.close();
    stoppingMPIC(ros::Time::now());

    // stop logging thread
    stop_logging_ = true;
    logging_thread_.join();
  }

  bool MpicController::initMPIC(ros::NodeHandle &node_handle)
  {
    bool init_successful = false;
    std::vector<std::string> joint_names;
    init_successful = readParameters();
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != Ndof_)
    {
      ROS_ERROR(
          "[MpicController::initMPIC]: Invalid or no joint_names parameters provided, aborting "
          "controller init!");
      return false;
    }
    // pinocchio
    ros::NodeHandle nh;
    std::string urdf_stream;
    if (!nh.getParam("/robot_description", urdf_stream))
    {
      ROS_ERROR("[MpicController::initMPIC]: Failed to load urdf stream");
    }
    pinocchio::urdf::buildModelFromXML(urdf_stream, model_);
    data_ = new pinocchio::Data(model_);
    flange_id_ = model_.getFrameId("panda_link8");

    gripper_->initGripper(node_handle);

    wrench_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>(node_handle, "/wrench", 1));
    pose_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>(node_handle, "/pose", 1));
    cart_vel_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped>(node_handle, "/cart_vel", 1));

    time_update_ = 0.0;
    loop_count_ = 0;

    if (!nh.getParam("/use_pbd_controller", use_pbd_controller_))
    {
      ROS_ERROR("[MpicController::initMPIC]: Reading configuration parameter use_pbd_controller failed!");
    }
    else
    {
      if (use_pbd_controller_)
      {
        ROS_INFO("[MpicController::initMPIC]: PBD-Controller is used!");
      }
      else
      {
        ROS_INFO("[MpicController::initMPIC]: MPIC-PF is used as controller!");
      }
    }

    // read task parameter from task.yaml
    if (ctrOpt_.skill)
    {
      if (!nh.getParam("/task_folder_path", task_folder_path_) || !nh.getParam("/task_file_name", task_file_name_))
      {
        ROS_ERROR("[MpicController::initMPIC]: Task yaml file not found!");
      }
      else
      {
        my_task_->readTaskParam(task_folder_path_, task_file_name_);
      }
      if (!nh.getParam("/exception_folder_path", exception_folder_path_))
      {
        ROS_ERROR("[MpicController::initMPIC]: Path to Exception Portfolio not found;");
      }
      else
      {
        ROS_INFO("[MpicController::initMPIC]: Loading exception portfolio");
        exception_portfolio_.stopMp_ = std::make_shared<GenericMPIC>(exception_folder_path_, "mp_stop.yaml");
        exception_portfolio_.moveToMp_ = std::make_shared<GenericMPIC>(exception_folder_path_, "mp_move_to.yaml");
      }
    }
    else
    {
      ROS_ERROR("[MpicController::initMPIC]: Reading task parameters failed!");
    }
    smach_.opt_.restart_ = ctrOpt_.repeatTask;

    // data logging
    data_meas_.open(log_path_ + "data.csv", std::ios::trunc);
    data_meas_ << "time" << "; " << "time_solver" << "; " << "time_update" << "; " << "q_1" << "; " << "q_2" << "; " << "q_3" << "; " << "q_4" << "; " << "q_5" << "; " << "q_6" << "; " << "q_7" << "; " << "dq_1" << "; " << "dq_2" << "; " << "dq_3" << "; " << "dq_4" << "; " << "dq_5" << "; " << "dq_6" << "; " << "dq_7" << "; " << "q_d_1" << "; " << "q_d_2" << "; " << "q_d_3" << "; " << "q_d_4" << "; " << "q_d_5" << "; " << "q_d_6" << "; " << "q_d_7" << "; " << "q_setPnt_1" << "; " << "q_setPnt_2" << "; " << "q_setPnt_3" << "; " << "q_setPnt_4" << "; " << "q_setPnt_5" << "; " << "q_setPnt_6" << "; " << "q_setPnt_7" << "; " << "p_x" << "; " << "p_y" << "; " << "p_z" << "; " << "q_x" << "; " << "q_y" << "; " << "q_z" << "; " << "q_w" << "; " << "v_x" << "; " << "v_y" << "; " << "v_z" << "; " << "w_x" << "; " << "w_y" << "; " << "w_z" << "; " << "p_setPnt_x" << "; " << "p_setPnt_y" << "; " << "p_setPnt_z" << "; " << "q_setPnt_x" << "; " << "q_setPnt_y" << "; " << "q_setPnt_z" << "; " << "q_setPnt_w" << "; " << "F_x" << "; " << "F_y" << "; " << "F_z" << "; " << "M_x" << "; " << "M_y" << "; " << "M_z" << "; " << "tau_1" << "; " << "tau_2" << "; " << "tau_3" << "; " << "tau_4" << "; " << "tau_5" << "; " << "tau_6" << "; " << "tau_7" << "; " << "F_mpc_x" << "; " << "F_mpc_y" << "; " << "F_mpc_z" << "; " << "M_mpc_x" << "; " << "M_mpc_y" << "; " << "M_mpc_z" << "; " << "F_setPnt_x" << "; " << "F_setPnt_y" << "; " << "F_setPnt_z" << "; " << "M_setPnt_x" << "; " << "M_setPnt_y" << "; " << "M_setPnt_z" << ";" << "s" << "; " << "ds" << ";" << "u_1" << "; " << "u_2" << "; " << "u_3" << "; " << "u_4" << "; " << "u_5" << "; " << "u_6" << "; " << "u_7" << "; " << "dds" << ";" << "J" << "; " << "J_opt" << "; " << "iSkill" << "; " << "iMP" << "; " << "state" << "; " << "cont_class" << "\n";

    ROS_INFO("[MpicController::initMPIC]: Initialisation of MpicController done");

    return init_successful;
  }

  void MpicController::startingMPIC(const ros::Time &time)
  {
    elapsed_time_ = ros::Duration(0.0);
    // init mpc variables with actual measured datas
    pred_mpc_data_.q_mpc_ = meas_mpc_data_.q_;
    pred_mpc_data_.dq_mpc_ = meas_mpc_data_.dq_;
    pred_mpc_data_.q_ref_mpc_ = meas_mpc_data_.q_;
    pred_mpc_data_.dq_ref_mpc_ = Eigen::VectorXd::Zero(Ndof_);
    pred_mpc_data_.s_mpc_ = Eigen::VectorXd::Zero(1);
    pred_mpc_data_.ds_mpc_ = Eigen::VectorXd::Zero(1);

    {
      // Init shared data
      std::lock_guard<std::mutex> lock(shared_mpc_data_.mutex_);
      shared_mpc_data_.t0_ = time.now().toSec();
      shared_mpc_data_.ctrGains_ = ctrGains_;
      shared_mpc_data_.Kges_ = Kges_;
      shared_mpc_data_.ctrOpt_ = ctrOpt_;

      if (ctrOpt_.skill)
      { // load either trajectories or set points and weights
        my_task_->resetTask();
        my_skill_ = std::make_shared<Skill>(*my_task_->getActSkill());
        for (int i = 0; i < 2; i++)
        {
          shared_mpc_data_.actMPvec_.at(i).reset();
          shared_mpc_data_.actMPvec_.at(i) = exception_portfolio_.stopMp_->clone();
        }
        shared_mpc_data_.s_f_ = 0;
        smach_.startTask();
      }
      else
      {
        // shared_mpc_data_.setPnt_=setPnt_;
      }

      // init x0
      for (unsigned i = 0; i < Ndof_; ++i)
      {
        shared_mpc_data_.x0_[i] = pred_mpc_data_.q_mpc_[i];
        shared_mpc_data_.x0_[Ndof_ + i] = pred_mpc_data_.q_ref_mpc_[i];
      }
      for (unsigned i = 0; i < CartDof_; ++i)
      {
        shared_mpc_data_.x0_[2 * Ndof_ + i] = 0.0;
      }
      shared_mpc_data_.x0_[2 * Ndof_ + CartDof_] = 0.0;     // s
      shared_mpc_data_.x0_[2 * Ndof_ + CartDof_ + 1] = 0.0; // ds

      // init xnext
      for (unsigned i = 0; i < Ndof_; ++i)
      {
        shared_mpc_data_.xnext_[i] = pred_mpc_data_.q_mpc_[i];
        shared_mpc_data_.xnext_[i + Ndof_] = pred_mpc_data_.q_ref_mpc_[i];
      }
      for (unsigned i = 0; i < CartDof_; ++i)
      {
        shared_mpc_data_.xnext_[i + 2 * Ndof_] = 0.0;
      }
      shared_mpc_data_.xnext_[2 * Ndof_ + CartDof_] = 0.0;     // s
      shared_mpc_data_.xnext_[2 * Ndof_ + CartDof_ + 1] = 0.0; // ds
    }
    ROS_INFO("[MpicController::startingMPIC]: Initial grampc update done");

    // MPC thread
    if (!use_pbd_controller_)
    {
      mpc_thread_ = std::thread([this]
                                {
      ros::Rate r(ctrOpt_.updateFreqMPIC); // threading frequency
      mpcThreadInit(); // Start MPC thread
      while(ros::ok()){ // while loop for MPC thread
        mpcThreadExecute();
        r.sleep();
      } });
      mpc_thread_.detach();
    }
    ROS_INFO("[MpicController::startingMPIC]: Starting MPIC Controller done");
  }

  Eigen::VectorXd MpicController::updateMPIC(const ros::Time &time, const ros::Duration &period)
  {
    auto start_clock_update = std::chrono::high_resolution_clock::now(); // Start time measurement

    elapsed_time_ += period;
    updateStiffness(); // update stiffness for force prediction
    {
      std::lock_guard<std::mutex> lock(shared_mpc_data_.mutex_);
      // -------------- Data exchange robot - grampc ---------------------------------
      // --- roboter -> grampc ---

      // states
      shared_mpc_data_.t0_ = time.now().toSec();
      for (unsigned i = 0; i < Ndof_; ++i)
      {
        shared_mpc_data_.x0_[i] = meas_mpc_data_.q_[i];
        shared_mpc_data_.x0_[Ndof_ + i] = pred_mpc_data_.q_ref_mpc_[i]; // q_ref from last MPC step
      }
      for (unsigned i = 0; i < CartDof_; ++i)
      {
        shared_mpc_data_.x0_[2 * Ndof_ + i] = meas_mpc_data_.f_ext_death_[i];
      }

      pred_mpc_data_.s_mpc_[0] = shared_mpc_data_.xnext_[2 * Ndof_ + CartDof_];      // s
      pred_mpc_data_.ds_mpc_[0] = shared_mpc_data_.xnext_[2 * Ndof_ + CartDof_ + 1]; // ds
      pred_mpc_data_.u_mpc_[Ndof_] = shared_mpc_data_.unext_[Ndof_];                 // dds

      pred_mpc_data_.dt_mpc_ = shared_mpc_data_.dt_;
      pred_mpc_data_.J_mpc_ = shared_mpc_data_.J_;

      // Task Skill MP
      if (!shared_mpc_data_.task_updated_)
      {
        shared_mpc_data_.task_updated_ = true;
        // -------------- Evaluate Task: Sense -----------------------------------------
        // calculate distance from task start TODO only calculates joint distance
        Eigen::VectorXd delta_q;
        delta_q = meas_mpc_data_.q_ - my_task_->getActSkill()->getMP(0)->getSetPoint(0, Eigen::VectorXd::Zero(Ndof_), Eigen::VectorXd::Zero(Ndof_), Eigen::VectorXd::Zero(Ndof_))->q_des;
        double delta_q_norm = delta_q.transpose() * delta_q;
        shared_mpc_data_.task_meas_.tooFarFromNextMP_ = (delta_q_norm < 0.02) ? false : true;

        // observe if mp is finished
        shared_mpc_data_.task_meas_.finished_ = shared_mpc_data_.actMPvec_[0]->evaluateMP(shared_mpc_data_.xnext_[2 * Ndof_ + CartDof_], meas_mpc_data_.q_, meas_mpc_data_.pose_, meas_mpc_data_.f_ext_death_);
        if (smach_.getCurrentState() == TaskState::reaction && shared_mpc_data_.xnext_[2 * Ndof_ + CartDof_] < 0.01)
        {
          shared_mpc_data_.task_meas_.finished_ = FinishFlag::finishedMP;
        }

        // check if current task or exception is finished
        shared_mpc_data_.task_meas_.hasSuccessor_ = my_task_->hasNextMP();

        shared_mpc_data_.task_meas_.reaction_ = shared_mpc_data_.actMPvec_[0]->getReactionStrategy();
        shared_mpc_data_.task_meas_.exception_ = shared_mpc_data_.actMPvec_[0]->getExceptionStrategy();

        // // -------------- Evaluate Task: Act -------------------------------------------
        TaskCommand task_cmd = smach_.evaluateTask(shared_mpc_data_.task_meas_);
        TaskState task_state = smach_.getCurrentState();

        // -------------- Evaluate Task: Act -------------------------------------------
        shared_mpc_data_.x0_[2 * Ndof_ + CartDof_] = 0; // s, overwritten if neccessary
        shared_mpc_data_.x0_[2 * Ndof_ + CartDof_ + 1] = 0;

        switch (task_cmd)
        {
        case TaskCommand::proceed:
          shared_mpc_data_.x0_[2 * Ndof_ + CartDof_] = shared_mpc_data_.xnext_[2 * Ndof_ + CartDof_];         // s
          shared_mpc_data_.x0_[2 * Ndof_ + CartDof_ + 1] = shared_mpc_data_.xnext_[2 * Ndof_ + CartDof_ + 1]; // ds
          shared_mpc_data_.actMPvec_.at(1)->rePlanPath(0.0, meas_mpc_data_.q_, meas_mpc_data_.pose_, meas_mpc_data_.f_ext_);
          break;

        case TaskCommand::stopExecution:
        {
          for (int i = 0; i < 2; i++)
          {
            shared_mpc_data_.actMPvec_.at(i).reset();
            shared_mpc_data_.actMPvec_.at(i) = exception_portfolio_.stopMp_->clone();
            shared_mpc_data_.actMPvec_.at(i)->rePlanPath(0.0, meas_mpc_data_.q_, meas_mpc_data_.pose_, meas_mpc_data_.f_ext_);
          }
          shared_mpc_data_.s_f_ = 0;
          break;
        }

        case TaskCommand::moveToNextMP:
        {
          shared_mpc_data_.s_f_ = 1;
          configureMoveTo(my_task_->getSkill(0)->getMP(0).get());
          shared_mpc_data_.actMPvec_.at(0).reset();
          shared_mpc_data_.actMPvec_.at(0) = exception_portfolio_.moveToMp_->clone();
          shared_mpc_data_.actMPvec_.at(1).reset();
          shared_mpc_data_.actMPvec_.at(1) = exception_portfolio_.stopMp_->clone(); // TODO 2. MP could be first of task
          shared_mpc_data_.actMPvec_.at(1)->rePlanPath(0.0, meas_mpc_data_.q_, meas_mpc_data_.pose_, meas_mpc_data_.f_ext_);
          break;
        }

        case TaskCommand::advanceToNextMP:
        {
          my_task_->advanceToNextMP();
          my_skill_ = std::make_shared<Skill>(*my_task_->getActSkill());
          updateMPvec(shared_mpc_data_.actMPvec_, my_skill_);
          shared_mpc_data_.s_f_ = (my_skill_->hasNextMP()) ? 2 : 1;
          break;
        }

        case TaskCommand::prepareStartConditon:
        {
          for (int i = 0; i < 2; i++)
          {
            shared_mpc_data_.actMPvec_.at(i).reset();
            shared_mpc_data_.actMPvec_.at(i) = exception_portfolio_.stopMp_->clone();
            shared_mpc_data_.actMPvec_.at(i)->rePlanPath(0.0, meas_mpc_data_.q_, meas_mpc_data_.pose_, meas_mpc_data_.f_ext_);
          }
          my_task_->resetTask();
          shared_mpc_data_.s_f_ = 0;
          break;
        }

        case TaskCommand::beginReaction:
        {
          if (shared_mpc_data_.actMPvec_[0]->getReactionStrategy() == ReactionStrategy::backward)
          {
            shared_mpc_data_.x0_[2 * Ndof_ + CartDof_] = shared_mpc_data_.xnext_[2 * Ndof_ + CartDof_];         // s
            shared_mpc_data_.x0_[2 * Ndof_ + CartDof_ + 1] = shared_mpc_data_.xnext_[2 * Ndof_ + CartDof_ + 1]; // ds
            shared_mpc_data_.s_f_ = 0;
          }
          break;
        }

        case TaskCommand::restartSkill:
        {
          my_task_->getActSkill()->resetSkill();
          my_task_->advanceToNextMP();
          my_skill_ = std::make_shared<Skill>(*my_task_->getActSkill());
          updateMPvec(shared_mpc_data_.actMPvec_, my_skill_);
          shared_mpc_data_.s_f_ = (my_skill_->hasNextMP()) ? 2 : 1;
          break;
        }

        case TaskCommand::restartSkillMoveTo:
        {
          my_task_->getActSkill()->resetSkill();
          my_skill_ = std::make_shared<Skill>(*my_task_->getActSkill());
          configureMoveTo(my_task_->getActSkill()->getMP(0).get());
          shared_mpc_data_.actMPvec_.at(0).reset();
          shared_mpc_data_.actMPvec_.at(0) = exception_portfolio_.moveToMp_->clone();
          shared_mpc_data_.actMPvec_.at(1).reset();
          shared_mpc_data_.actMPvec_.at(1) = my_skill_->getMP(0)->clone();
          shared_mpc_data_.s_f_ = 2;
          break;
        }

        case TaskCommand::restartMP:
        {
          updateMPvec(shared_mpc_data_.actMPvec_, my_skill_);
          shared_mpc_data_.s_f_ = (my_skill_->hasNextMP()) ? 2 : 1;
          break;
        }

        case TaskCommand::beginException:
        {
          switch (shared_mpc_data_.task_meas_.exception_)
          {
          default:
            break;
          }
          // default behaviour
          break;
        }

        default:
          break;
        }
      }

      // stiffness matrix
      shared_mpc_data_.Kges_ = Kges_;

      // ----------------------------------------------
      // --- grampc -> roboter ---
      for (unsigned i = 0; i < Ndof_; ++i)
      {
        pred_mpc_data_.q_mpc_[i] = shared_mpc_data_.xnext_[i];
        pred_mpc_data_.q_ref_mpc_[i] = shared_mpc_data_.xnext_[i + Ndof_];
        pred_mpc_data_.u_mpc_[i] = shared_mpc_data_.unext_[i];
      }
      for (unsigned i = 0; i < CartDof_; ++i)
      {
        pred_mpc_data_.F_mpc_[i] = shared_mpc_data_.xnext_[i + 2 * Ndof_];
      }

      setPnt_ = shared_mpc_data_.setPnt_;
    } // unlock

    // compute control
    // allocate variables
    static Eigen::VectorXd tau_mpic(Ndof_);
    // PD control
    if (use_pbd_controller_)
    {
      tau_mpic << Eigen::VectorXd::Zero(Ndof_); // only gravity compensation active
    }
    else
    {
      if (loop_count_ % 10 == 0)
      {
        std::cout << "s = " << pred_mpc_data_.s_mpc_[0] << ", s_f = " << shared_mpc_data_.s_f_ << ", state = " << taskStateToString(smach_.getCurrentState()) << " skill/MP: " << my_task_->getActSkillNo() << "/" << my_task_->getActSkill()->getActMPNo() << std::endl;
      }

      tau_mpic << (ctrGains_.Kpd * (pred_mpc_data_.q_ref_mpc_ - meas_mpc_data_.q_) + ctrGains_.Dpd * (pred_mpc_data_.u_mpc_.head(Ndof_) - meas_mpc_data_.dq_));
      // Check for NaN values in tau_mpic
      for (int i = 0; i < tau_mpic.size(); ++i)
      {
        if (std::isnan(tau_mpic[i]))
        {
          tau_mpic.setZero();
          break;
        }
      }
      GripperAnswer gripper_ans;
      Gripper gripper_goal = my_skill_->getActPathParam()->gripper;
      gripper_ans = gripper_->updateGripper(false,
                                            gripper_goal.width,
                                            gripper_goal.speed,
                                            gripper_goal.force);
    }
    writeStates();
    publishStates();
    loop_count_++;

    auto stop_clock_update = std::chrono::high_resolution_clock::now(); // Stop time measurement
    time_update_ = static_cast<int>(std::chrono::duration_cast<std::chrono::microseconds>(stop_clock_update - start_clock_update).count());
    return tau_mpic;
  }

  void MpicController::writeStates()
  {
    if (ctrOpt_.writeStates && loop_count_ % ctrOpt_.writeFreq == 0)
    {
      std::ostringstream buffer;

      // time
      buffer << elapsed_time_.toSec() << "; " << pred_mpc_data_.dt_mpc_ << "; " << time_update_ << "; ";

      // joint positions
      for (int i = 0; i < Ndof_; ++i)
        buffer << pred_mpc_data_.q_mpc_[i] << "; ";
      // joint velocities
      for (int i = 0; i < Ndof_; ++i)
        buffer << pred_mpc_data_.dq_mpc_[i] << "; ";
      // desired joint position (MPC state)
      for (int i = 0; i < Ndof_; ++i)
        buffer << pred_mpc_data_.q_ref_mpc_[i] << "; ";
      // control objective for MPC (joint position)
      for (int i = 0; i < Ndof_; ++i)
        buffer << setPnt_.q_des[i] << "; ";

      // Cartesian pose
      for (int i = 0; i < Ndof_; ++i)
        buffer << meas_mpc_data_.pose_[i] << "; ";
      // Cartesian velocity
      for (int i = 0; i < CartDof_; ++i)
        buffer << meas_mpc_data_.cart_vel_[i] << "; ";
      // desired pose
      for (int i = 0; i < Ndof_; ++i)
        buffer << setPnt_.p_des[i] << "; ";

      // measured interaction wrench
      for (int i = 0; i < CartDof_; ++i)
        buffer << meas_mpc_data_.f_ext_death_[i] << "; ";
      // external joint torques
      for (int i = 0; i < Ndof_; ++i)
        buffer << meas_mpc_data_.tau_ext_[i] << "; ";
      // predicted interaction wrench
      for (int i = 0; i < CartDof_; ++i)
        buffer << pred_mpc_data_.F_mpc_[i] << "; ";
      // desired interaction wrench
      for (int i = 0; i < CartDof_; ++i)
        buffer << setPnt_.F_des[i] << "; ";

      // path progress and velocity
      buffer << pred_mpc_data_.s_mpc_[0] << "; " << pred_mpc_data_.ds_mpc_[0] << "; ";

      // control inputs
      for (int i = 0; i < Ndof_; ++i)
        buffer << pred_mpc_data_.u_mpc_[i] << "; ";
      buffer << pred_mpc_data_.u_mpc_[Ndof_] << "; ";

      // MPC cost
      buffer << pred_mpc_data_.J_mpc_[0] << "; " << pred_mpc_data_.J_mpc_[1] << "; ";

      // Push the buffer to the log queue
      buffer << std::endl;
      log_queue_.push(buffer.str());
    }
  }

  void MpicController::publishStates()
  {
    if (ctrOpt_.pubStates && loop_count_ % ctrOpt_.pubFreq == 0)
    {
      // publish actual pose
      pose_pub_->lock();
      pose_pub_->msg_.header.stamp = ros::Time::now();
      pose_pub_->msg_.header.frame_id = "panda_link0";
      pose_pub_->msg_.pose.position.x = meas_mpc_data_.position_.x();
      pose_pub_->msg_.pose.position.y = meas_mpc_data_.position_.y();
      pose_pub_->msg_.pose.position.z = meas_mpc_data_.position_.z();
      pose_pub_->msg_.pose.orientation.w = meas_mpc_data_.orientation_.w();
      pose_pub_->msg_.pose.orientation.x = meas_mpc_data_.orientation_.x();
      pose_pub_->msg_.pose.orientation.y = meas_mpc_data_.orientation_.y();
      pose_pub_->msg_.pose.orientation.z = meas_mpc_data_.orientation_.z();
      pose_pub_->unlockAndPublish();
      cart_vel_pub_->lock();
      cart_vel_pub_->msg_.header.stamp = ros::Time::now();
      cart_vel_pub_->msg_.header.frame_id = "panda_link0";
      cart_vel_pub_->msg_.twist.linear.x = meas_mpc_data_.cart_vel_[0];
      cart_vel_pub_->msg_.twist.linear.y = meas_mpc_data_.cart_vel_[1];
      cart_vel_pub_->msg_.twist.linear.z = meas_mpc_data_.cart_vel_[2];
      cart_vel_pub_->msg_.twist.angular.x = meas_mpc_data_.cart_vel_[3];
      cart_vel_pub_->msg_.twist.angular.y = meas_mpc_data_.cart_vel_[4];
      cart_vel_pub_->msg_.twist.angular.z = meas_mpc_data_.cart_vel_[5];
      cart_vel_pub_->unlockAndPublish();
      // publish wrench
      wrench_pub_->lock();
      wrench_pub_->msg_.header.stamp = ros::Time::now();
      wrench_pub_->msg_.header.frame_id = "panda_link8";
      wrench_pub_->msg_.wrench.force.x = meas_mpc_data_.f_ext_death_[0];
      wrench_pub_->msg_.wrench.force.y = meas_mpc_data_.f_ext_death_[1];
      wrench_pub_->msg_.wrench.force.z = meas_mpc_data_.f_ext_death_[2];

      wrench_pub_->msg_.wrench.torque.x = meas_mpc_data_.f_ext_death_[3];
      wrench_pub_->msg_.wrench.torque.y = meas_mpc_data_.f_ext_death_[4];
      wrench_pub_->msg_.wrench.torque.z = meas_mpc_data_.f_ext_death_[5];
      wrench_pub_->unlockAndPublish();
    }
  }
  void MpicController::stoppingMPIC(const ros::Time &time)
  {
    ROS_WARN("MpicController stopped");
  }

  void MpicController::setStates(const Eigen::VectorXd &q, const Eigen::VectorXd &dq, const Eigen::VectorXd &tau_J,
                                 const Eigen::VectorXd &tau_ext, const Eigen::VectorXd &f_ext, const Eigen::VectorXd &f_ext_death,
                                 const Eigen::VectorXd &pose, const Eigen::VectorXd &cart_vel, const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation,
                                 const Eigen::MatrixXd &jac, const Eigen::MatrixXd &jac_pinv, const Eigen::MatrixXd &jac_transp_pinv)
  {

    {
      std::lock_guard<std::mutex> lock(shared_mpc_data_.mutex_);
      meas_mpc_data_.q_ = q;
      meas_mpc_data_.dq_ = dq;
      meas_mpc_data_.tau_J_ = tau_J;
      meas_mpc_data_.tau_ext_ = tau_ext;
      meas_mpc_data_.f_ext_ = f_ext;
      meas_mpc_data_.f_ext_death_ = f_ext_death;
      meas_mpc_data_.pose_ = pose;
      meas_mpc_data_.cart_vel_ = cart_vel;
      meas_mpc_data_.position_ = position;
      meas_mpc_data_.orientation_ = orientation;
      meas_mpc_data_.jac_ = jac;
      meas_mpc_data_.jac_pinv_ = jac_pinv;
      meas_mpc_data_.jac_transpose_pinv_ = jac_transp_pinv;
    }
  }

  lrt_mpic::controlOptions *MpicController::getCtrOpt()
  {
    return &ctrOpt_;
  }

  void MpicController::updateStiffness()
  {
    Eigen::MatrixXd Ke_tmp = Ke_;
    Eigen::MatrixXd Kctr_cart = (meas_mpc_data_.jac_transpose_pinv_ * ctrGains_.Kpd * meas_mpc_data_.jac_pinv_);
    Eigen::MatrixXd K_sum_inv = Eigen::MatrixXd::Zero(CartDof_, CartDof_);
    pseudoInverse(Kctr_cart + Ke_tmp, K_sum_inv);
    Eigen::MatrixXd Kges_tmp = (K_sum_inv * Ke_tmp * Kctr_cart);
    Kges_ = Kges_tmp;
  }

  bool MpicController::readParameters()
  {
    bool paramReadSucc = true;
    std::vector<double> Ke, pref_min, pref_max;

    if (!ros::param::get("/environment/Ke", Ke) || Ke.size() != CartDof_)
    {
      ROS_ERROR(
          "MpicController:  Invalid or no Environment parameters provided, aborting "
          "controller init!");
      return false;
    }
    else
    {
      for (unsigned int i = 0; i < CartDof_; ++i)
      {
        Ke_(i, i) = Ke[i];
      }
    }

    // read PD controller params
    std::vector<double> Kctr, Dctr;
    if (!ros::param::get("/ctr_param/Kpd", Kctr) || Kctr.size() != Ndof_ ||
        !ros::param::get("/ctr_param/Dpd", Dctr) || Dctr.size() != Ndof_)
    {
      ROS_ERROR(
          "MpicController:  Invalid or no PD-controller param provided, aborting "
          "controller init!");
      paramReadSucc = false;
    }
    else
    {
      for (unsigned int i = 0; i < Ndof_; ++i)
      {
        ctrGains_.Kpd(i, i) = Kctr[i];
        ctrGains_.Dpd(i, i) = Dctr[i];
      }
    }

    int writeFreq = 1;
    int pubFreq = 1;
    if (!ros::param::get("/controllerOptions/updateWeights", ctrOpt_.updateWeights) ||
        !ros::param::get("/controllerOptions/updateSetPoint", ctrOpt_.updateSetPoint) ||
        !ros::param::get("/controllerOptions/updateFreqMPIC", ctrOpt_.updateFreqMPIC) ||
        !ros::param::get("/controllerOptions/writeStates", ctrOpt_.writeStates) ||
        !ros::param::get("/controllerOptions/writeHor", ctrOpt_.writeHor) ||
        !ros::param::get("/controllerOptions/writeFreq", writeFreq) ||
        !ros::param::get("/controllerOptions/pubStates", ctrOpt_.pubStates) ||
        !ros::param::get("/controllerOptions/pubFreq", pubFreq) ||
        !ros::param::get("/controllerOptions/log_path", log_path_) ||
        !ros::param::get("/controllerOptions/skill", ctrOpt_.skill) ||
        !ros::param::get("/controllerOptions/repeatTask", ctrOpt_.repeatTask) ||
        !ros::param::get("/controllerOptions/extFaultDetect", ctrOpt_.extFaultDetect) ||
        !ros::param::get("/controllerOptions/fts", ctrOpt_.fts))
    {
      ROS_ERROR(
          "MpicController:  Invalid or no MPIC parameters provided, aborting "
          "controller init!");
      paramReadSucc = false;
    }
    ctrOpt_.writeFreq = (int)floor(1000 / writeFreq);
    ctrOpt_.pubFreq = (int)floor(1000 / pubFreq);
    return paramReadSucc;
  }

  bool MpicController::readEnvir()
  {
    ros::NodeHandle nh("/environment");
    std::vector<double> Ke, pref_min, pref_max;
    if (!nh.getParam("/environment/Ke", Ke) || Ke.size() != CartDof_)
    {
      ROS_ERROR(
          "MpicController:  Invalid or no Environment parameters provided, aborting "
          "controller init!");
      return false;
    }
    else
    {
      for (unsigned int i = 0; i < CartDof_; ++i)
      {
        Ke_(i, i) = Ke[i];
      }
      return true;
    }
    return false;
  }
  lrt::MPICProblemDescriptionPtr MpicController::configureProblem()
  {
    lrt::MPICProblemDescriptionPtr problem(new lrt::MPICProblemDescription(Ndof_, CartDof_, Nstates_, Nctr_, Neqc_, Nieqc_, meas_mpc_data_.q_, meas_mpc_data_.pose_, shared_mpc_data_.actMPvec_, Nhor_, Thor_));
    return problem;
  }
  grampc::GrampcPtr MpicController::configureSolver(const lrt::MPICProblemDescriptionPtr &problem)
  {
    grampc::GrampcPtr solver_(new grampc::Grampc(problem.get()));

    // set optimization options
    ros::NodeHandle nh_opt("/grampc/opt");
    lrt::grampc_setopt(*solver_, nh_opt);

    // print options
    bool print_opt = true;
    nh_opt.getParamCached("print", print_opt);
    if (print_opt)
    {
      solver_->printopt();
    }
    // set optimization parameters
    ros::NodeHandle nh_param("/grampc/param");
    lrt::grampc_setparam(*solver_, nh_param);

    // print parameters
    bool print_param = true;
    nh_param.getParamCached("print", print_param);
    if (print_param)
    {
      solver_->printparam();
    }
    return solver_;
  }

  void MpicController::mpcThreadInit()
  {
    {
      // Init shared data
      std::lock_guard<std::mutex> lock(shared_mpc_data_.mutex_);
      // --- Initialization of Grampc ---
      // create GRAMPC problem
      problem_description_ = configureProblem();
      // create GRAMPC solver
      solver_ = configureSolver(problem_description_);
      // initial set of MPIC data
      problem_description_->setControllerParam(shared_mpc_data_.ctrGains_);
      problem_description_->setKges(shared_mpc_data_.Kges_, shared_mpc_data_.actMPvec_.at(0)->getWeights()->Q_F);
      problem_description_->setControllerOpt(shared_mpc_data_.ctrOpt_);
    }
    xnow_ = (typeRNum *)calloc(Nstates_, sizeof(*xnow_));
    ROS_INFO("[MpicController::startingMPIC]: Threading initialization done");
  }

  void MpicController::mpcThreadExecute()
  {
    auto start_high_res_clk = std::chrono::high_resolution_clock::now(); // Start measurement
    {
      std::lock_guard<std::mutex> lock(shared_mpc_data_.mutex_);

      for (unsigned i = 0; i < Ndof_; ++i)
      {
        xnow_[i] = shared_mpc_data_.x0_[i];
        xnow_[Ndof_ + i] = shared_mpc_data_.x0_[Ndof_ + i];
      }
      for (unsigned i = 0; i < CartDof_; ++i)
      {
        xnow_[2 * Ndof_ + i] = shared_mpc_data_.x0_[2 * Ndof_ + i];
      }

      xnow_[2 * Ndof_ + CartDof_] = shared_mpc_data_.x0_[2 * Ndof_ + CartDof_];         // s
      xnow_[2 * Ndof_ + CartDof_ + 1] = shared_mpc_data_.x0_[2 * Ndof_ + CartDof_ + 1]; // ds

      problem_description_->setKges(shared_mpc_data_.Kges_, shared_mpc_data_.actMPvec_.at(0)->getWeights()->Q_F); // update Kges for MPIC due to config dependency
      problem_description_->setMPs(shared_mpc_data_.actMPvec_);
      problem_description_->setDesiredProgressParam(shared_mpc_data_.s_f_);
    }
    // Optimization
    solver_->setparam_real_vector("x0", xnow_);
    solver_->run();
    {
      std::lock_guard<std::mutex> lock(shared_mpc_data_.mutex_);

      for (unsigned i = 0; i < Ndof_; ++i)
      {
        shared_mpc_data_.xnext_[i] = solver_->getSolution()->xnext[i];
        shared_mpc_data_.xnext_[i + Ndof_] = solver_->getSolution()->xnext[i + Ndof_];
        shared_mpc_data_.unext_[i] = solver_->getSolution()->unext[i];
      }
      for (unsigned i = 0; i < CartDof_; ++i)
      {
        shared_mpc_data_.xnext_[i + 2 * Ndof_] = solver_->getSolution()->xnext[i + 2 * Ndof_];
      }

      // path dynamic
      shared_mpc_data_.xnext_[2 * Ndof_ + CartDof_] = solver_->getSolution()->xnext[2 * Ndof_ + CartDof_];         // s
      shared_mpc_data_.xnext_[2 * Ndof_ + CartDof_ + 1] = solver_->getSolution()->xnext[2 * Ndof_ + CartDof_ + 1]; // ds
      shared_mpc_data_.unext_[Ndof_] = solver_->getSolution()->unext[Ndof_];                                       // dds
      shared_mpc_data_.J_[0] = solver_->getSolution()->J[0];
      shared_mpc_data_.J_[1] = solver_->getSolution()->J[1];
      shared_mpc_data_.cnt_mpc_ = solver_->getSolution()->iter[0];
      shared_mpc_data_.setPnt_ = problem_description_->getActSetPoint();
      shared_mpc_data_.task_updated_ = false; // reset flag

      auto stop_high_res_clk = std::chrono::high_resolution_clock::now(); // Stop measurement
      const int time_solver_micro_second_ = static_cast<int>(std::chrono::duration_cast<std::chrono::microseconds>(stop_high_res_clk - start_high_res_clk).count());
      shared_mpc_data_.dt_ = time_solver_micro_second_;
    }
  }

  void MpicController::updateMPvec(std::vector<std::shared_ptr<ManipulationPrimitive>> &MPvec, const std::shared_ptr<Skill> &skill)
  {
    unsigned idx_actMP = skill->getActMPNo();
    MPvec.at(0).reset();
    MPvec.at(0) = skill->getMP(idx_actMP)->clone();
    MPvec.at(0)->rePlanPath(0.0, meas_mpc_data_.q_, meas_mpc_data_.pose_, meas_mpc_data_.f_ext_);

    MPvec.at(1).reset();

    if (skill->hasNextMP())
    {
      MPvec.at(1) = skill->getMP(idx_actMP + 1)->clone();
      MPvec.at(1)->rePlanPath(0.0, meas_mpc_data_.q_, meas_mpc_data_.pose_, meas_mpc_data_.f_ext_);
    }
    else
    {
      MPvec.at(1) = exception_portfolio_.stopMp_->clone();
      MPvec.at(1)->rePlanPath(0.0, meas_mpc_data_.q_, meas_mpc_data_.pose_, meas_mpc_data_.f_ext_);
    }
  }

  void MpicController::configureMoveTo(ManipulationPrimitive *MP)
  {
    GenericPath *tmppath = exception_portfolio_.moveToMp_->getPathParam();
    if (MP->getWeights()->Q_q.norm() >= 0.01)
    {
      tmppath->q.q_1[0] = meas_mpc_data_.q_[0];
      tmppath->q.q_2[0] = meas_mpc_data_.q_[1];
      tmppath->q.q_3[0] = meas_mpc_data_.q_[2];
      tmppath->q.q_4[0] = meas_mpc_data_.q_[3];
      tmppath->q.q_5[0] = meas_mpc_data_.q_[4];
      tmppath->q.q_6[0] = meas_mpc_data_.q_[5];
      tmppath->q.q_7[0] = meas_mpc_data_.q_[6];

      Eigen::VectorXd des_q = MP->getSetPoint(0, Eigen::VectorXd::Zero(Ndof_), Eigen::VectorXd::Zero(Ndof_), Eigen::VectorXd::Zero(Ndof_))->q_des;
      tmppath->q.q_1[1] = des_q[0];
      tmppath->q.q_2[1] = des_q[1];
      tmppath->q.q_3[1] = des_q[2];
      tmppath->q.q_4[1] = des_q[3];
      tmppath->q.q_5[1] = des_q[4];
      tmppath->q.q_6[1] = des_q[5];
      tmppath->q.q_7[1] = des_q[6];

      weights *wghts = exception_portfolio_.moveToMp_->getWeights();
      wghts->Q_p = Eigen::MatrixXd::Zero(CartDof_, CartDof_);
      wghts->Q_q.diagonal() << 1000, 1000, 1000, 1000, 1000, 1000, 1000;
    }
    else
    {
      tmppath->pose.p_x[0] = meas_mpc_data_.pose_[0];
      tmppath->pose.p_y[0] = meas_mpc_data_.pose_[1];
      tmppath->pose.p_z[0] = meas_mpc_data_.pose_[2];
      tmppath->pose.q_0[0] = meas_mpc_data_.pose_[3];
      tmppath->pose.q_1[0] = meas_mpc_data_.pose_[4];
      tmppath->pose.q_2[0] = meas_mpc_data_.pose_[5];
      tmppath->pose.q_3[0] = meas_mpc_data_.pose_[6];

      Eigen::VectorXd des_p = MP->getSetPoint(0, Eigen::VectorXd::Zero(Ndof_), Eigen::VectorXd::Zero(Ndof_), Eigen::VectorXd::Zero(Ndof_))->p_des;
      tmppath->pose.p_x[1] = des_p[0];
      tmppath->pose.p_y[1] = des_p[1];
      tmppath->pose.p_z[1] = des_p[2];
      tmppath->pose.q_0[1] = des_p[3];
      tmppath->pose.q_1[1] = des_p[4];
      tmppath->pose.q_2[1] = des_p[5];
      tmppath->pose.q_3[1] = des_p[6];

      weights *wghts = exception_portfolio_.moveToMp_->getWeights();
      wghts->Q_p.diagonal() << 1000, 1000, 1000, 500, 500, 500;
      wghts->Q_q = Eigen::MatrixXd::Zero(Ndof_, Ndof_);
    }
  }

  void MpicController::loggingThreadFunction()
  {
    while (!stop_logging_)
    {
      std::string log_entry;
      log_queue_.wait_and_pop(log_entry);
      data_meas_ << log_entry;
    }
  }

} // namespace lrt_controller
