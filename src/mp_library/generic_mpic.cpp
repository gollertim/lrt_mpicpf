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

#include "mp_library/generic_mpic.hpp"

namespace lrt_mpic
{
  GenericMPIC::GenericMPIC() {}

  GenericMPIC::GenericMPIC(const std::string folder_path, const std::string mp_file_name)
  {
    Ndof_ = 7;
    CartDof_ = 6;
    type_ = MPtype::generic_mpic;
    name_ = "";
    index_ = 0;
    sloc_ = 0;
    delta_s_loc_ = 0.1;

    TerminationCondition transition_cond_(CartDof_ + 1, CartDof_);
    TerminationCondition error_cond_(CartDof_ + 1, CartDof_);
    reaction_ = ReactionStrategy::none;
    exception_ = ExceptionStrategy::none;

    pathSetPnt_.p_des = Eigen::VectorXd::Zero(CartDof_ + 1);

    // weights
    wghts_.R = Eigen::MatrixXd::Zero(Ndof_, Ndof_);
    wghts_.Q_q = Eigen::MatrixXd::Zero(Ndof_, Ndof_);
    wghts_.Q_dq = Eigen::MatrixXd::Zero(Ndof_, Ndof_);
    wghts_.Q_p = Eigen::MatrixXd::Zero(CartDof_, CartDof_);
    wghts_.Q_v = Eigen::MatrixXd::Zero(3, 3);
    wghts_.Q_F = Eigen::MatrixXd::Zero(CartDof_, CartDof_);

    wghts_.R_s = 0.0;
    wghts_.Q_s = 0.0;
    wghts_.Q_ds = 0.0;

    wghts_.Q_N = Eigen::MatrixXd::Zero(Ndof_, Ndof_);

    // Set points
    pathSetPnt_.q_des = Eigen::VectorXd::Zero(Ndof_);
    pathSetPnt_.dq_des = Eigen::VectorXd::Zero(Ndof_);
    pathSetPnt_.ddq_des = Eigen::VectorXd::Zero(Ndof_);
    pathSetPnt_.p_des = Eigen::VectorXd::Zero(7);
    pathSetPnt_.F_des = Eigen::VectorXd::Zero(CartDof_);

    pathSetPnt_.s_des = 0.0;
    pathSetPnt_.ds_des = 0.0;
    pathSetPnt_.dds_des = 0.0;

    // Set points Gradient
    pathSetPntGradient_.q_des = Eigen::VectorXd::Zero(Ndof_);
    pathSetPntGradient_.dq_des = Eigen::VectorXd::Zero(Ndof_);
    pathSetPntGradient_.ddq_des = Eigen::VectorXd::Zero(Ndof_);
    pathSetPntGradient_.p_des = Eigen::VectorXd::Zero(7);
    pathSetPntGradient_.F_des = Eigen::VectorXd::Zero(CartDof_);

    pathSetPntGradient_.s_des = 0.0;
    pathSetPntGradient_.ds_des = 0.0;
    pathSetPntGradient_.dds_des = 0.0;

    // Constraints
    constr_.q_min = Eigen::VectorXd::Zero(Ndof_);
    constr_.q_max = Eigen::VectorXd::Zero(Ndof_);
    constr_.dq_max = Eigen::VectorXd::Zero(Ndof_);
    constr_.p_min = Eigen::VectorXd::Zero(CartDof_);
    constr_.p_max = Eigen::VectorXd::Zero(CartDof_);
    constr_.p_min_elbow = Eigen::VectorXd::Zero(3);
    constr_.p_max_elbow = Eigen::VectorXd::Zero(3);
    constr_.v_max = Eigen::VectorXd::Zero(CartDof_);
    constr_.F_min = Eigen::VectorXd::Zero(CartDof_);
    constr_.F_max = Eigen::VectorXd::Zero(CartDof_);
    constr_.q_constr = false;
    constr_.dq_constr = false;
    constr_.F_constr = false;
    constr_.p_constr = false;
    constr_.p_constr_elbow = false;
    constr_.v_constr = false;

    // initialize MP from yaml file
    initializeFromFile(folder_path, mp_file_name);
  }

  std::shared_ptr<ManipulationPrimitive> GenericMPIC::clone() const
  {
    return std::make_shared<GenericMPIC>(*this);
  }

  GenericPath *GenericMPIC::getPathParam()
  {
    return &path_;
  }

  setPoint *GenericMPIC::getSetPoint(const double s, const Eigen::VectorXd &q, const Eigen::VectorXd &p, const Eigen::VectorXd &F)
  {
    linearPath(s);
    return &pathSetPnt_;
  }

  setPoint *GenericMPIC::getSetPointGradient(const double s)
  {
    linearPathGradient(s);
    return &pathSetPntGradient_;
  }

  weights *GenericMPIC::getWeights()
  {
    return &wghts_;
  }

  // optionally reduce speed at the beginning and end of the MP
  weights *GenericMPIC::getScaledWeights(const double s, const double s_f)
  {
    wghtsScaled_ = wghts_;

    if (s <= delta_s_loc_ || s >= 1 - delta_s_loc_)
    {
      wghtsScaled_.Q_ds = 5 * wghts_.Q_ds;
      wghtsScaled_.R = 10 * wghts_.R;
    }

    return &wghtsScaled_;
  }

  constraints *GenericMPIC::getConstraints()
  {
    return &constr_;
  }

  std::string GenericMPIC::getName() { return name_; }

  MPtype *GenericMPIC::getType() { return &type_; }

  bool GenericMPIC::evaluateMP(const double &s)
  {
    if (s >= path_.s[path_.nSamples - 1])
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  FinishFlag GenericMPIC::evaluateMP(const double &s, const Eigen::VectorXd &q, const Eigen::VectorXd &p, const Eigen::VectorXd &F)
  {
    Eigen::VectorXd F_des = Eigen::VectorXd::Zero(6);
    double s_loc = 0.0;
    unsigned index = 0;
    lrt_mpic::selectIndexOnPath(index, path_.s, s);
    lrt_mpic::calcLocalPathParam(s_loc, s, path_.s[index], path_.s[index + 1]);
    if (s_loc < 0.0 || isnan(s_loc))
    {
      s_loc = 0.0;
    }

    lrt_mpic::lerp(F_des[0], path_.F.F_x[index_], path_.F.F_x[index_ + 1], s_loc);
    lrt_mpic::lerp(F_des[1], path_.F.F_y[index_], path_.F.F_y[index_ + 1], s_loc);
    lrt_mpic::lerp(F_des[2], path_.F.F_z[index_], path_.F.F_z[index_ + 1], s_loc);
    lrt_mpic::lerp(F_des[3], path_.F.M_x[index_], path_.F.M_x[index_ + 1], s_loc);
    lrt_mpic::lerp(F_des[4], path_.F.M_y[index_], path_.F.M_y[index_ + 1], s_loc);
    lrt_mpic::lerp(F_des[5], path_.F.M_z[index_], path_.F.M_z[index_ + 1], s_loc);

    // Termiation
    //    - Pose conditions
    for (unsigned i = 0; i < CartDof_ + 1; i++)
    {
      // Transition
      switch (transition_cond_.pose_cond.selection[i])
      {
      case -1:
      {
        if (p[i] < transition_cond_.pose_cond.threshold[i])
        {
          return FinishFlag::finishedMP;
        }
        break;
      }
      case 0:
      {
        break;
      }
      case 1:
      {
        if (p[i] > transition_cond_.pose_cond.threshold[i])
        {
          return FinishFlag::finishedMP;
        }
        break;
      }
      default:
        break;
      }
      // Fault
      switch (error_cond_.pose_cond.selection[i])
      {
      case -1:
      {
        if (p[i] < error_cond_.pose_cond.threshold[i])
        {
          return FinishFlag::failedMP;
        }
        break;
      }
      case 0:
      {
        break;
      }
      case 1:
      {
        if (p[i] > error_cond_.pose_cond.threshold[i])
        {
          return FinishFlag::failedMP;
        }
        break;
      }
      default:
        break;
      }
    }

    //    - Wrench condition
    for (unsigned i = 0; i < CartDof_; i++)
    {
      // Transition
      switch (transition_cond_.wrench_cond.selection[i])
      {
      case -1:
      {
        if (F[i] < transition_cond_.wrench_cond.threshold[i])
        {
          return FinishFlag::finishedMP;
        }
        break;
      }
      case 0:
      {
        break;
      }
      case 1:
      {
        if (F[i] > transition_cond_.wrench_cond.threshold[i])
        {
          return FinishFlag::finishedMP;
        }
        break;
      }
      default:
        break;
      }
      // Fault
      switch (error_cond_.wrench_cond.selection[i])
      {
      case -1:
      {
        if (F[i] < error_cond_.wrench_cond.threshold[i])
        {
          return FinishFlag::failedMP;
        }
        break;
      }
      case 0:
      {
        break;
      }
      case 1:
      {
        if (F[i] > error_cond_.wrench_cond.threshold[i])
        {
          return FinishFlag::failedMP;
        }
        break;
      }
      default:
        break;
      }
    }
    //    - Progress condition
    if (transition_cond_.pose_cond.selection.isZero() && transition_cond_.wrench_cond.selection.isZero())
    {
      if (s >= path_.s[path_.nSamples - 1])
      {
        return FinishFlag::finishedMP;
      }
    }
    else
    {
      if (s >= path_.s[path_.nSamples - 1])
      {
        return FinishFlag::failedMP;
      }
    }

    return FinishFlag::runningMP;
  }

  double GenericMPIC::getInitialProgressParam() { return path_.s[0]; }
  double GenericMPIC::getTerminationProgressParam() { return path_.s[path_.nSamples - 1]; }

  ReactionStrategy GenericMPIC::getReactionStrategy() { return reaction_; }
  ExceptionStrategy GenericMPIC::getExceptionStrategy() { return exception_; }

  void GenericMPIC::print()
  {
    std::cout << "-------- MP of type " << mpTypeToString(type_) << " named " << name_ << std::endl;
    std::cout << "--- generic MPIC-MP --- " << std::endl;
    std::cout << "Parameters:" << std::endl;
    std::cout << "  Rs: " << wghts_.R_s << std::endl;
    std::cout << "  Qs: " << wghts_.Q_s << std::endl;
    std::cout << "  Qds: " << wghts_.Q_ds << std::endl;
    std::cout << "  R: " << wghts_.R.diagonal().transpose() << std::endl;
    std::cout << "  Qq: " << wghts_.Q_q.diagonal().transpose() << std::endl;
    std::cout << "  Qp: " << wghts_.Q_p.diagonal().transpose() << std::endl;
    std::cout << "  QF: " << wghts_.Q_F.diagonal().transpose() << std::endl;
    std::cout << "  nSamples: " << path_.nSamples << std::endl;

    std::cout << "Joint Position:" << std::endl;
    std::cout << "  q_1: " << path_.q.q_1.transpose() << std::endl;
    std::cout << "  q_2: " << path_.q.q_2.transpose() << std::endl;
    std::cout << "  q_3: " << path_.q.q_3.transpose() << std::endl;
    std::cout << "  q_4: " << path_.q.q_4.transpose() << std::endl;
    std::cout << "  q_5: " << path_.q.q_5.transpose() << std::endl;
    std::cout << "  q_6: " << path_.q.q_6.transpose() << std::endl;
    std::cout << "  q_7: " << path_.q.q_7.transpose() << std::endl;

    std::cout << "Pose:" << std::endl;
    std::cout << "  p_x: " << path_.pose.p_x.transpose() << std::endl;
    std::cout << "  p_y: " << path_.pose.p_y.transpose() << std::endl;
    std::cout << "  p_z: " << path_.pose.p_z.transpose() << std::endl;
    std::cout << "  q_0: " << path_.pose.q_0.transpose() << std::endl;
    std::cout << "  q_1: " << path_.pose.q_1.transpose() << std::endl;
    std::cout << "  q_2: " << path_.pose.q_2.transpose() << std::endl;
    std::cout << "  q_3: " << path_.pose.q_3.transpose() << std::endl;

    std::cout << "s: " << path_.s.transpose() << std::endl;

    std::cout << "Gripper:" << std::endl;
    std::cout << "  gripper_width: " << path_.gripper.width << std::endl;
    std::cout << "  gripper_speed: " << path_.gripper.speed << std::endl;
    std::cout << "  gripper_force: " << path_.gripper.force << std::endl;

    std::cout << "Transition:" << std::endl;
    std::cout << "  pose_selection: " << transition_cond_.pose_cond.selection.transpose() << std::endl;
    std::cout << "  pose_threshold: " << transition_cond_.pose_cond.threshold.transpose() << std::endl;
    std::cout << "  wrench_selection: " << transition_cond_.wrench_cond.selection.transpose() << std::endl;
    std::cout << "  wrench_threshold: " << transition_cond_.wrench_cond.threshold.transpose() << std::endl;

    std::cout << "Error:" << std::endl;
    std::cout << "  pose_selection: " << error_cond_.pose_cond.selection.transpose() << std::endl;
    std::cout << "  pose_threshold: " << error_cond_.pose_cond.threshold.transpose() << std::endl;
    std::cout << "  wrench_selection: " << error_cond_.wrench_cond.selection.transpose() << std::endl;
    std::cout << "  wrench_threshold: " << error_cond_.wrench_cond.threshold.transpose() << std::endl;
    std::cout << "  reaction_strategy:  " << reactionStrategyToString(reaction_) << std::endl;
    std::cout << "  exception_strategy: " << exceptionStrategyToString(exception_) << std::endl;
  }

  bool GenericMPIC::initializeFromFile(const std::string folder_path, const std::string mp_file_name)
  {
    // Load MP YAML file
    mp_node_ = YAML::LoadFile(folder_path + mp_file_name);

    std::cout << "[GenericMPIC::initialize]: MP of type: generic_mpic" << std::endl;

    // basic weights
    try
    {
      mp_param_.R = mp_node_["mp/param"]["R"].as<std::vector<double>>();
      mp_param_.Rs = mp_node_["mp/param"]["Rs"].as<std::vector<double>>();
      mp_param_.Qs = mp_node_["mp/param"]["Qs"].as<std::vector<double>>();
      mp_param_.Qds = mp_node_["mp/param"]["Qds"].as<std::vector<double>>();
    }
    catch (...)
    {
      std::cout << "[GenericMPIC::initialize]: Invalid basic weight parameter of MP: type generic MPIC-MP" << std::endl;
    }

    // control objective and weights
    try
    {
      mp_param_.nSamples = mp_node_["mp/param"]["nSamples"].as<std::vector<double>>();
    }
    catch (...)
    {
      mp_param_.nSamples = {1};
      std::cout << "[GenericMPIC::initialize]: nSamples of MP: type generic MPIC-MP reading failed!" << std::endl;
    }

    initPath(path_, mp_param_.nSamples[0]);
    try
    {
      // Pose condition
      error_cond_.pose_cond.selection = Eigen::Map<Eigen::Matrix<int, 7, 1>>(mp_node_["mp/error"]["Pose_condition"]["selection"].as<std::vector<int>>().data());
      error_cond_.pose_cond.threshold = Eigen::Map<Eigen::Matrix<double, 7, 1>>(mp_node_["mp/error"]["Pose_condition"]["threshold"].as<std::vector<double>>().data());
      // Wrench condition
      error_cond_.wrench_cond.selection = Eigen::Map<Eigen::Matrix<int, 6, 1>>(mp_node_["mp/error"]["Wrench_condition"]["selection"].as<std::vector<int>>().data());
      error_cond_.wrench_cond.threshold = Eigen::Map<Eigen::Matrix<double, 6, 1>>(mp_node_["mp/error"]["Wrench_condition"]["threshold"].as<std::vector<double>>().data());
      // FEP
      reaction_ = stringToReactionStrategy(mp_node_["mp/error"]["reaction"].as<std::string>());
      exception_ = stringToExceptionStrategy(mp_node_["mp/error"]["exception"].as<std::string>());
    }
    catch (...)
    {
      // Pose condition
      error_cond_.pose_cond.selection = Eigen::VectorXi::Zero(7);
      error_cond_.pose_cond.threshold = Eigen::VectorXd::Zero(7);
      // Wrench condition
      error_cond_.wrench_cond.selection = Eigen::VectorXi::Zero(6);
      error_cond_.wrench_cond.threshold = Eigen::VectorXd::Zero(6);
      // FEP
      reaction_ = ReactionStrategy::none;
      exception_ = ExceptionStrategy::none;
    }

    try
    {
      // Pose condition
      transition_cond_.pose_cond.selection = Eigen::Map<Eigen::Matrix<int, 7, 1>>(mp_node_["mp/Transition"]["Pose_condition"]["selection"].as<std::vector<int>>().data());
      transition_cond_.pose_cond.threshold = Eigen::Map<Eigen::Matrix<double, 7, 1>>(mp_node_["mp/Transition"]["Pose_condition"]["threshold"].as<std::vector<double>>().data());
      // Wrench condition
      transition_cond_.wrench_cond.selection = Eigen::Map<Eigen::Matrix<int, 6, 1>>(mp_node_["mp/Transition"]["Wrench_condition"]["selection"].as<std::vector<int>>().data());
      transition_cond_.wrench_cond.threshold = Eigen::Map<Eigen::Matrix<double, 6, 1>>(mp_node_["mp/Transition"]["Wrench_condition"]["threshold"].as<std::vector<double>>().data());
    }
    catch (...)
    {
      // Pose condition
      transition_cond_.pose_cond.selection = Eigen::VectorXi::Zero(7);
      transition_cond_.pose_cond.threshold = Eigen::VectorXd::Zero(7);
      // Wrench condition
      transition_cond_.wrench_cond.selection = Eigen::VectorXi::Zero(6);
      transition_cond_.wrench_cond.threshold = Eigen::VectorXd::Zero(6);
    }

    // joint
    try
    {
      mp_param_.Qq = mp_node_["mp/param"]["Qq"].as<std::vector<double>>();
    }
    catch (...)
    {
      mp_param_.Qq = {0, 0, 0, 0, 0, 0, 0};
    }

    try
    {
      mp_param_.q1 = mp_node_["mp/JointPosition"]["q_1"].as<std::vector<double>>();
      mp_param_.q2 = mp_node_["mp/JointPosition"]["q_2"].as<std::vector<double>>();
      mp_param_.q3 = mp_node_["mp/JointPosition"]["q_3"].as<std::vector<double>>();
      mp_param_.q4 = mp_node_["mp/JointPosition"]["q_4"].as<std::vector<double>>();
      mp_param_.q5 = mp_node_["mp/JointPosition"]["q_5"].as<std::vector<double>>();
      mp_param_.q6 = mp_node_["mp/JointPosition"]["q_6"].as<std::vector<double>>();
      mp_param_.q7 = mp_node_["mp/JointPosition"]["q_7"].as<std::vector<double>>();
    }
    catch (...)
    {
      for (unsigned i = 0; i < mp_param_.nSamples[0]; i++)
      {
        mp_param_.q1.push_back(0);
        mp_param_.q2.push_back(0);
        mp_param_.q3.push_back(0);
        mp_param_.q4.push_back(0);
        mp_param_.q5.push_back(0);
        mp_param_.q6.push_back(0);
        mp_param_.q7.push_back(0);
      }
    }

    // pose
    try
    {
      mp_param_.Qp = mp_node_["mp/param"]["Qp"].as<std::vector<double>>();
    }
    catch (...)
    {
      mp_param_.Qp = {0, 0, 0, 0, 0, 0};
    }

    try
    {
      mp_param_.px = mp_node_["mp/Pose"]["p_x"].as<std::vector<double>>();
      mp_param_.py = mp_node_["mp/Pose"]["p_y"].as<std::vector<double>>();
      mp_param_.pz = mp_node_["mp/Pose"]["p_z"].as<std::vector<double>>();
      mp_param_.quat_0 = mp_node_["mp/Pose"]["quat_0"].as<std::vector<double>>();
      mp_param_.quat_1 = mp_node_["mp/Pose"]["quat_1"].as<std::vector<double>>();
      mp_param_.quat_2 = mp_node_["mp/Pose"]["quat_2"].as<std::vector<double>>();
      mp_param_.quat_3 = mp_node_["mp/Pose"]["quat_3"].as<std::vector<double>>();
    }
    catch (...)
    {
      for (unsigned i = 0; i < path_.nSamples; i++)
      {
        mp_param_.px.push_back(0);
        mp_param_.py.push_back(0);
        mp_param_.pz.push_back(0);
        mp_param_.quat_0.push_back(0);
        mp_param_.quat_1.push_back(0);
        mp_param_.quat_2.push_back(0);
        mp_param_.quat_3.push_back(0);
      }
    }

    // force
    try
    {
      mp_param_.QF = mp_node_["mp/param"]["QF"].as<std::vector<double>>();
    }
    catch (...)
    {
      mp_param_.QF = {0, 0, 0, 0, 0, 0};
    }

    try
    {
      mp_param_.Fx = mp_node_["mp/Wrench"]["F_x"].as<std::vector<double>>();
      mp_param_.Fy = mp_node_["mp/Wrench"]["F_y"].as<std::vector<double>>();
      mp_param_.Fz = mp_node_["mp/Wrench"]["F_z"].as<std::vector<double>>();
      mp_param_.Mx = mp_node_["mp/Wrench"]["M_x"].as<std::vector<double>>();
      mp_param_.My = mp_node_["mp/Wrench"]["M_y"].as<std::vector<double>>();
      mp_param_.Mz = mp_node_["mp/Wrench"]["M_z"].as<std::vector<double>>();
    }
    catch (...)
    {
      for (unsigned i = 0; i < path_.nSamples; i++)
      {
        mp_param_.Fx.push_back(0);
        mp_param_.Fy.push_back(0);
        mp_param_.Fz.push_back(0);
        mp_param_.Mx.push_back(0);
        mp_param_.My.push_back(0);
        mp_param_.Mz.push_back(0);
      }
    }

    // gripper
    try
    {
      mp_param_.gripper_width = mp_node_["mp/Gripper"]["width"].as<std::vector<double>>();
      mp_param_.gripper_speed = mp_node_["mp/Gripper"]["speed"].as<std::vector<double>>();
      mp_param_.gripper_force = mp_node_["mp/Gripper"]["force"].as<std::vector<double>>();
    }
    catch (...)
    {
      mp_param_.gripper_width.push_back(0);
      mp_param_.gripper_speed.push_back(0);
      mp_param_.gripper_force.push_back(0);
    }

    // progress param
    try
    {
      mp_param_.s = mp_node_["mp/s"].as<std::vector<double>>();
    }
    catch (...)
    {
      std::cout << "[GenericMPIC::initialize]: Invalid progress parameter s of MP: type generic MPIC-MP" << std::endl;
    }

    if (mp_param_.R.size() != Ndof_ ||
        mp_param_.Qq.size() != Ndof_ ||
        mp_param_.Qp.size() != CartDof_ ||
        mp_param_.QF.size() != CartDof_ ||
        mp_param_.Rs.size() != 1 ||
        mp_param_.Qs.size() != 1 ||
        mp_param_.Qds.size() != 1)
    {
      std::cout << "[GenericMPIC::initialize]: Invalid size of MP parameter!" << std::endl;
      return false;
    }
    else
    {
      std::cout << "[GenericMPIC::initialize]: Setting params..." << std::endl;

      path_.nSamples = mp_param_.nSamples[0];

      path_.gripper.width = mp_param_.gripper_width[0];
      path_.gripper.speed = mp_param_.gripper_speed[0];
      path_.gripper.force = mp_param_.gripper_force[0];

      for (unsigned i = 0; i < path_.nSamples; ++i)
      {
        path_.s[i] = mp_param_.s[i];
        // JointPosition
        path_.q.q_1[i] = mp_param_.q1[i];
        path_.q.q_2[i] = mp_param_.q2[i];
        path_.q.q_3[i] = mp_param_.q3[i];
        path_.q.q_4[i] = mp_param_.q4[i];
        path_.q.q_5[i] = mp_param_.q5[i];
        path_.q.q_6[i] = mp_param_.q6[i];
        path_.q.q_7[i] = mp_param_.q7[i];
        // Pose
        path_.pose.p_x[i] = mp_param_.px[i];
        path_.pose.p_y[i] = mp_param_.py[i];
        path_.pose.p_z[i] = mp_param_.pz[i];
        path_.pose.q_0[i] = mp_param_.quat_0[i];
        path_.pose.q_1[i] = mp_param_.quat_1[i];
        path_.pose.q_2[i] = mp_param_.quat_2[i];
        path_.pose.q_3[i] = mp_param_.quat_3[i];
        // Wrench
        path_.F.F_x[i] = mp_param_.Fx[i];
        path_.F.F_y[i] = mp_param_.Fy[i];
        path_.F.F_z[i] = mp_param_.Fz[i];
        path_.F.M_x[i] = mp_param_.Mx[i];
        path_.F.M_y[i] = mp_param_.My[i];
        path_.F.M_z[i] = mp_param_.Mz[i];
      }

      for (unsigned i = 0; i < Ndof_; ++i)
      {
        wghts_.R(i, i) = mp_param_.R[i];
        wghts_.Q_q(i, i) = mp_param_.Qq[i];
      }

      for (unsigned i = 0; i < CartDof_; ++i)
      {
        wghts_.Q_p(i, i) = mp_param_.Qp[i];
        wghts_.Q_F(i, i) = mp_param_.QF[i];
      }

      wghts_.Q_s = mp_param_.Qs[0];
      wghts_.Q_ds = mp_param_.Qds[0];
      wghts_.R_s = mp_param_.Rs[0];

      wghtsScaled_ = wghts_;

    }
    std::cout << "[GenericMPIC::initialize]: Initialized" << std::endl;
    this->print();
    return true;
  }

  void GenericMPIC::linearPath(const double s)
  {
    lrt_mpic::selectIndexOnPath(index_, path_.s, s);

    lrt_mpic::calcLocalPathParam(sloc_, s, path_.s[index_], path_.s[index_ + 1]);

    // Joint Position
    lrt_mpic::lerp(pathSetPnt_.q_des[0], path_.q.q_1[index_], path_.q.q_1[index_ + 1], sloc_);
    lrt_mpic::lerp(pathSetPnt_.q_des[1], path_.q.q_2[index_], path_.q.q_2[index_ + 1], sloc_);
    lrt_mpic::lerp(pathSetPnt_.q_des[2], path_.q.q_3[index_], path_.q.q_3[index_ + 1], sloc_);
    lrt_mpic::lerp(pathSetPnt_.q_des[3], path_.q.q_4[index_], path_.q.q_4[index_ + 1], sloc_);
    lrt_mpic::lerp(pathSetPnt_.q_des[4], path_.q.q_5[index_], path_.q.q_5[index_ + 1], sloc_);
    lrt_mpic::lerp(pathSetPnt_.q_des[5], path_.q.q_6[index_], path_.q.q_6[index_ + 1], sloc_);
    lrt_mpic::lerp(pathSetPnt_.q_des[6], path_.q.q_7[index_], path_.q.q_7[index_ + 1], sloc_);

    // Cart position
    lrt_mpic::lerp(pathSetPnt_.p_des[0], path_.pose.p_x[index_], path_.pose.p_x[index_ + 1], sloc_);
    lrt_mpic::lerp(pathSetPnt_.p_des[1], path_.pose.p_y[index_], path_.pose.p_y[index_ + 1], sloc_);
    lrt_mpic::lerp(pathSetPnt_.p_des[2], path_.pose.p_z[index_], path_.pose.p_z[index_ + 1], sloc_);

    // Cart orientation
    Q0_ = Eigen::Quaterniond::Identity();
    Qf_ = Eigen::Quaterniond::Identity();
    Q_ = Eigen::Quaterniond::Identity();
    Q0_.w() = path_.pose.q_0[index_];
    Q0_.vec() << path_.pose.q_1[index_], path_.pose.q_2[index_], path_.pose.q_3[index_];
    Qf_.w() = path_.pose.q_0[index_ + 1];
    Qf_.vec() << path_.pose.q_1[index_ + 1], path_.pose.q_2[index_ + 1], path_.pose.q_3[index_ + 1];

    Q_ = Q0_.slerp(sloc_, Qf_);
    pathSetPnt_.p_des[3] = Q_.w();
    pathSetPnt_.p_des.tail(3) = Q_.vec();

    // Wrench
    lrt_mpic::lerp(pathSetPnt_.F_des[0], path_.F.F_x[index_], path_.F.F_x[index_ + 1], sloc_);
    lrt_mpic::lerp(pathSetPnt_.F_des[1], path_.F.F_y[index_], path_.F.F_y[index_ + 1], sloc_);
    lrt_mpic::lerp(pathSetPnt_.F_des[2], path_.F.F_z[index_], path_.F.F_z[index_ + 1], sloc_);
    lrt_mpic::lerp(pathSetPnt_.F_des[3], path_.F.M_x[index_], path_.F.M_x[index_ + 1], sloc_);
    lrt_mpic::lerp(pathSetPnt_.F_des[4], path_.F.M_y[index_], path_.F.M_y[index_ + 1], sloc_);
    lrt_mpic::lerp(pathSetPnt_.F_des[5], path_.F.M_z[index_], path_.F.M_z[index_ + 1], sloc_);
  }

  void GenericMPIC::linearPathGradient(const double s)
  {
    lrt_mpic::selectIndexOnPath(index_, path_.s, s);
    lrt_mpic::calcLocalPathParam(sloc_, s, path_.s[index_], path_.s[index_ + 1]);

    // Joint Position
    lrt_mpic::lerpGrad(pathSetPntGradient_.q_des[0], path_.q.q_1[index_], path_.q.q_1[index_ + 1], path_.s[index_], path_.s[index_ + 1]);
    lrt_mpic::lerpGrad(pathSetPntGradient_.q_des[1], path_.q.q_2[index_], path_.q.q_2[index_ + 1], path_.s[index_], path_.s[index_ + 1]);
    lrt_mpic::lerpGrad(pathSetPntGradient_.q_des[2], path_.q.q_3[index_], path_.q.q_3[index_ + 1], path_.s[index_], path_.s[index_ + 1]);
    lrt_mpic::lerpGrad(pathSetPntGradient_.q_des[3], path_.q.q_4[index_], path_.q.q_4[index_ + 1], path_.s[index_], path_.s[index_ + 1]);
    lrt_mpic::lerpGrad(pathSetPntGradient_.q_des[4], path_.q.q_5[index_], path_.q.q_5[index_ + 1], path_.s[index_], path_.s[index_ + 1]);
    lrt_mpic::lerpGrad(pathSetPntGradient_.q_des[5], path_.q.q_6[index_], path_.q.q_6[index_ + 1], path_.s[index_], path_.s[index_ + 1]);
    lrt_mpic::lerpGrad(pathSetPntGradient_.q_des[6], path_.q.q_7[index_], path_.q.q_7[index_ + 1], path_.s[index_], path_.s[index_ + 1]);

    // Cart position
    lrt_mpic::lerpGrad(pathSetPntGradient_.p_des[0], path_.pose.p_x[index_], path_.pose.p_x[index_ + 1], path_.s[index_], path_.s[index_ + 1]);
    lrt_mpic::lerpGrad(pathSetPntGradient_.p_des[1], path_.pose.p_y[index_], path_.pose.p_y[index_ + 1], path_.s[index_], path_.s[index_ + 1]);
    lrt_mpic::lerpGrad(pathSetPntGradient_.p_des[2], path_.pose.p_z[index_], path_.pose.p_z[index_ + 1], path_.s[index_], path_.s[index_ + 1]);

    // Cart orientation
    Q0_ = Eigen::Quaterniond::Identity();
    Qf_ = Eigen::Quaterniond::Identity();
    Q_ = Eigen::Quaterniond::Identity();
    Qtmp_ = Eigen::Quaterniond::Identity();
    Qlog_ = Eigen::Quaterniond::Identity();
    Q0_.w() = path_.pose.q_0[index_];
    Q0_.vec() << path_.pose.q_1[index_], path_.pose.q_2[index_], path_.pose.q_3[index_];
    Qf_.w() = path_.pose.q_0[index_ + 1];
    Qf_.vec() << path_.pose.q_1[index_ + 1], path_.pose.q_2[index_ + 1], path_.pose.q_3[index_ + 1];

    if (Qf_.norm() != 1)
    {
      Qf_ = Q0_;
      Q_ = Q0_;
    }
    else
    {
      Qtmp_ = Q0_.conjugate() * Qf_;

      lrt_mpic::QuaternionLogarithm(Qlog_, Qtmp_);
      Q_ = Q0_.slerp(sloc_, Qf_) * Qlog_;
    }

    pathSetPntGradient_.p_des[3] = Q_.w();
    pathSetPntGradient_.p_des.tail(3) = Q_.vec();

    // Wrench
    lrt_mpic::lerpGrad(pathSetPntGradient_.F_des[0], path_.F.F_x[index_], path_.F.F_x[index_ + 1], path_.s[index_], path_.s[index_ + 1]);
    lrt_mpic::lerpGrad(pathSetPntGradient_.F_des[1], path_.F.F_y[index_], path_.F.F_y[index_ + 1], path_.s[index_], path_.s[index_ + 1]);
    lrt_mpic::lerpGrad(pathSetPntGradient_.F_des[2], path_.F.F_z[index_], path_.F.F_z[index_ + 1], path_.s[index_], path_.s[index_ + 1]);
    lrt_mpic::lerpGrad(pathSetPntGradient_.F_des[3], path_.F.M_x[index_], path_.F.M_x[index_ + 1], path_.s[index_], path_.s[index_ + 1]);
    lrt_mpic::lerpGrad(pathSetPntGradient_.F_des[4], path_.F.M_y[index_], path_.F.M_y[index_ + 1], path_.s[index_], path_.s[index_ + 1]);
    lrt_mpic::lerpGrad(pathSetPntGradient_.F_des[5], path_.F.M_z[index_], path_.F.M_z[index_ + 1], path_.s[index_], path_.s[index_ + 1]);
  }

  void GenericMPIC::rePlanPath(const double s, const Eigen::VectorXd &q, const Eigen::VectorXd &p, const Eigen::VectorXd &F)
  {
    Eigen::VectorXd delta_pos = Eigen::VectorXd::Zero(7);
    delta_pos[0] = p[0] - path_.pose.p_x[0];
    delta_pos[1] = p[1] - path_.pose.p_y[0];
    delta_pos[2] = p[2] - path_.pose.p_z[0];
    delta_pos[3] = p[3] - path_.pose.q_0[0];
    delta_pos[4] = p[4] - path_.pose.q_1[0];
    delta_pos[5] = p[5] - path_.pose.q_2[0];
    delta_pos[6] = p[6] - path_.pose.q_3[0];

    Eigen::VectorXd delta_wrench = Eigen::VectorXd::Zero(6);
    delta_wrench[0] = F[0] - path_.F.F_x[0];
    delta_wrench[1] = F[1] - path_.F.F_y[0];
    delta_wrench[2] = F[2] - path_.F.F_z[0];
    delta_wrench[3] = F[3] - path_.F.M_x[0];
    delta_wrench[4] = F[4] - path_.F.M_y[0];
    delta_wrench[5] = F[5] - path_.F.M_z[0];

    for (unsigned i = 1; i < path_.nSamples; i++)
    {
      path_.pose.p_x[i - 1] = path_.pose.p_x[i - 1] + (delta_pos[0] * ((path_.nSamples - i) / path_.nSamples));
      path_.pose.p_y[i - 1] = path_.pose.p_y[i - 1] + (delta_pos[1] * ((path_.nSamples - i) / path_.nSamples));
      path_.pose.p_z[i - 1] = path_.pose.p_z[i - 1] + (delta_pos[2] * ((path_.nSamples - i) / path_.nSamples));
      path_.pose.q_0[i - 1] = path_.pose.q_0[i - 1] + (delta_pos[3] * ((path_.nSamples - i) / path_.nSamples));
      path_.pose.q_1[i - 1] = path_.pose.q_1[i - 1] + (delta_pos[4] * ((path_.nSamples - i) / path_.nSamples));
      path_.pose.q_2[i - 1] = path_.pose.q_2[i - 1] + (delta_pos[5] * ((path_.nSamples - i) / path_.nSamples));
      path_.pose.q_3[i - 1] = path_.pose.q_3[i - 1] + (delta_pos[6] * ((path_.nSamples - i) / path_.nSamples));

      path_.F.F_x[i - 1] = path_.F.F_x[i - 1] + (delta_wrench[0] * ((path_.nSamples - i) / path_.nSamples));
      path_.F.F_y[i - 1] = path_.F.F_y[i - 1] + (delta_wrench[1] * ((path_.nSamples - i) / path_.nSamples));
      path_.F.F_z[i - 1] = path_.F.F_z[i - 1] + (delta_wrench[2] * ((path_.nSamples - i) / path_.nSamples));
      path_.F.M_x[i - 1] = path_.F.M_x[i - 1] + (delta_wrench[3] * ((path_.nSamples - i) / path_.nSamples));
      path_.F.M_y[i - 1] = path_.F.M_y[i - 1] + (delta_wrench[4] * ((path_.nSamples - i) / path_.nSamples));
      path_.F.M_z[i - 1] = path_.F.M_z[i - 1] + (delta_wrench[5] * ((path_.nSamples - i) / path_.nSamples));
    }
  }

} // namespace lrt_mpic
