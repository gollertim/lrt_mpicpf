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

#include "mpic_problem_description.hpp"

/* square macro */
#define POW2(a) ((a) * (a))

#define DELTA_S 0.001

namespace lrt
{

  MPICProblemDescription::MPICProblemDescription(unsigned Ndof, unsigned CartDof, unsigned Nstates, unsigned Nctr, unsigned Neqc, unsigned Nieqc, const Eigen::VectorXd q_0_, const Eigen::VectorXd p_0, const std::vector<std::shared_ptr<lrt_mpic::ManipulationPrimitive>> MP_vec, unsigned Nhor, double Thor)
      : grampc::ProblemDescription(),
        Ndof_(Ndof),
        CartDof_(CartDof),
        Nx_(Nstates),
        Nu_(Ndof + 1),
        Neqc_(Neqc),
        Nieqc_(Nieqc), // 2*Ndof (Joint position) + 2 (Force Constraints)
        Nhor_(Nhor),
        Thor_(Thor),
        horizon_cache_motion_data_(Nhor, Thor, MotionData(Ndof)),
        horizon_cache_motion_reference_(Nhor, Thor, MotionReference(Ndof, CartDof))
  {
    iMP_ = 0;

    // jacobians
    jac_pinv_ = Eigen::MatrixXd::Zero(Ndof_, CartDof_);
    jac_transpose_ = Eigen::MatrixXd::Zero(Ndof_, CartDof_);

    // position for ellbow contraints states
    position_ = Eigen::VectorXd::Zero(3);
    // one dimension less in pose_des, due to consideration of only vectorial part
    pose_error_ = Eigen::VectorXd::Zero(6);
    orientation_des_.setIdentity();
    orientation_error_.setIdentity();

    // PD Controller gains
    ctrGains_.Kpd = Eigen::MatrixXd::Zero(Ndof_, Ndof_);
    ctrGains_.Dpd = Eigen::MatrixXd::Zero(Ndof_, Ndof_);
    ctrGains_.Dctr_inv = Eigen::MatrixXd::Zero(Ndof_, Ndof_);
    ctrGains_.K_D = Eigen::MatrixXd::Zero(Ndof_, Ndof_);

    // normalization matrices
    N_q_ = Eigen::MatrixXd::Identity(Ndof_, Ndof_);
    N_q_ = 1.0 / M_PI * N_q_;
    N_p_ = Eigen::MatrixXd::Identity(CartDof_, CartDof_);
    N_F_ = Eigen::MatrixXd::Identity(CartDof_, CartDof_);
    N_F_ = 1.0 / 1370.0 * N_F_;

    // Predicted cost
    cost_ = 0;

    // partial derivative of poseshared_ptr<Song> sp5(nullptr);Ndof_);
    dlp_ds_ = Eigen::VectorXd::Zero(CartDof_);
    dlF_ds_ = Eigen::VectorXd::Zero(CartDof_);

    // Environment
    Kges_ = Eigen::MatrixXd::Zero(CartDof_, CartDof_); // Overal Stiffness

    initSetPoint(sharedSetPnt_, Ndof_, CartDof_);

    // pinocchio
    ros::NodeHandle nh;
    std::string urdf_stream;
    if (!nh.getParam("/robot_description", urdf_stream))
    {
      ROS_ERROR("[MPICProblemDescription::MPICProblemDescription]: Failed to load urdf stream");
    }
    pinocchio::urdf::buildModelFromXML(urdf_stream, model_);
    data_ = new pinocchio::Data(model_);
    flange_id_ = model_.getFrameId("panda_link8");

    // update cache for state x0
    MotionData &motion_data = horizon_cache_motion_data_.get(0);
    motionDataFunction(motion_data, model_, data_, flange_id_, q_0_);

    // init MP vector
    for (int i = 0; i < 2; i++)
    {
      MP_vec_.push_back(MP_vec.at(i)->clone());
    }

    MotionReference &motion_reference = horizon_cache_motion_reference_.get(0);
    motionReferenceFunction(motion_reference, 0.0, q_0_, p_0, Eigen::VectorXd::Zero(CartDof_));

    s_f_ = 0.0;
  }
  void MPICProblemDescription::ocp_dim(typeInt *Nx, typeInt *Nu, typeInt *Np, typeInt *Neqc, typeInt *Nieqc, typeInt *Nteqc, typeInt *Ntieqc)
  {
    *Nx = Nx_;
    *Nu = Nu_;
    *Np = 0;
    *Neqc = Neqc_;
    *Nieqc = Nieqc_;
    *Nteqc = 0;
    *Ntieqc = 0;
  }

  void MPICProblemDescription::ffct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p)
  {
    // input states
    Eigen::Map<const Eigen::VectorXd> q(x, Ndof_);
    Eigen::Map<const Eigen::VectorXd> q_ref(x + Ndof_, Ndof_);
    Eigen::Map<const Eigen::VectorXd> F(x + 2 * Ndof_, CartDof_);
    Eigen::Map<const Eigen::VectorXd> s(x + 2 * Ndof_ + CartDof_, 1);
    Eigen::Map<const Eigen::VectorXd> ds(x + 2 * Ndof_ + CartDof_ + 1, 1);
    // input control
    Eigen::Map<const Eigen::VectorXd> dq_ref(u, Ndof_);
    Eigen::Map<const Eigen::VectorXd> dds(u + Ndof_, 1);
    // output
    Eigen::Map<Eigen::VectorXd> out_dq(out, Ndof_);
    Eigen::Map<Eigen::VectorXd> out_dq_ref(out + Ndof_, Ndof_);
    Eigen::Map<Eigen::VectorXd> out_dF(out + 2 * Ndof_, CartDof_);
    Eigen::Map<Eigen::VectorXd> out_s(out + 2 * Ndof_ + CartDof_, 1);
    Eigen::Map<Eigen::VectorXd> out_ds(out + 2 * Ndof_ + CartDof_ + 1, 1);

    // compute forward kinematics data and store in horizon cache
    MotionData &motion_data = horizon_cache_motion_data_.get(t);
    motionDataFunction(motion_data, model_, data_, flange_id_, q);

    // compute control reference and store in horizon cache
    MotionReference &motion_reference = horizon_cache_motion_reference_.get(t);
    motionReferenceFunction(motion_reference, s[0], q, motion_data.ee_pose_, F);

    // robot dynamic
    out_dq = dq_ref + ctrGains_.K_D * (q_ref - q);
    // double integrator
    out_dq_ref = dq_ref;

    // force dynamic only if force control is active
    if (motion_reference.wghts_.Q_F(0, 0) > 0 || motion_reference.wghts_.Q_F(1, 1) > 0 || motion_reference.wghts_.Q_F(2, 2) > 0)
    {
      out_dq += -ctrGains_.Dctr_inv * motion_data.ee_jac_.transpose() * F;
      out_dF = Kges_ * motion_data.ee_jac_ * dq_ref;
    }
    else
    {
      out_dF = Eigen::VectorXd::Zero(CartDof_);
    }

    // path dynamic
    out_s = ds;
    out_ds = dds;

    // data exchange for logging
    if (t == 0)
    {
      copySetPntData(sharedSetPnt_, motion_reference.setPnt_);
    }
  }
  void MPICProblemDescription::dfdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *vec, ctypeRNum *u, ctypeRNum *p)
  {
    // input states
    Eigen::Map<const Eigen::VectorXd> q(x, Ndof_);
    Eigen::Map<const Eigen::VectorXd> q_ref(x + Ndof_, Ndof_);
    Eigen::Map<const Eigen::VectorXd> F(x + 2 * Ndof_, CartDof_);
    Eigen::Map<const Eigen::VectorXd> s(x + 2 * Ndof_ + CartDof_, 1);
    Eigen::Map<const Eigen::VectorXd> lambda_q(vec, Ndof_);
    Eigen::Map<const Eigen::VectorXd> lambda_q_ref(vec + Ndof_, Ndof_);
    Eigen::Map<const Eigen::VectorXd> lambda_F(vec + 2 * Ndof_, CartDof_);
    Eigen::Map<const Eigen::VectorXd> lambda_s(vec + 2 * Ndof_ + CartDof_, 1);

    Eigen::Map<Eigen::VectorXd> out_df_q(out, Ndof_);
    Eigen::Map<Eigen::VectorXd> out_df_q_ref(out + Ndof_, Ndof_);
    Eigen::Map<Eigen::VectorXd> out_df_F(out + 2 * Ndof_, CartDof_);
    Eigen::Map<Eigen::VectorXd> out_df_s(out + 2 * Ndof_ + CartDof_, 1);
    Eigen::Map<Eigen::VectorXd> out_df_ds(out + 2 * Ndof_ + CartDof_ + 1, 1);

    const MotionData &fk_data = horizon_cache_motion_data_.get(t);
    const MotionReference &mr_data = horizon_cache_motion_reference_.get(t);

    out_df_q = -ctrGains_.K_D * lambda_q;
    out_df_q_ref = ctrGains_.K_D * lambda_q;

    // force dynamic
    if (mr_data.wghts_.Q_F(0, 0) > 0 || mr_data.wghts_.Q_F(1, 1) > 0 || mr_data.wghts_.Q_F(2, 2) > 0)
    { // if force control
      out_df_F = -fk_data.ee_jac_ * ctrGains_.Dctr_inv * lambda_q;
    }
    else
    {
      out_df_F = Eigen::VectorXd::Zero(CartDof_);
    }

    // path dynamic
    out_df_s[0] = 0.0;
    out_df_ds = lambda_s;
  }
  void MPICProblemDescription::dfdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *vec, ctypeRNum *u, ctypeRNum *p)
  {
    // input states
    Eigen::Map<const Eigen::VectorXd> q(x, Ndof_);
    Eigen::Map<const Eigen::VectorXd> F(x + 2 * Ndof_, CartDof_);
    Eigen::Map<const Eigen::VectorXd> s(x + 2 * Ndof_ + CartDof_, 1);
    Eigen::Map<const Eigen::VectorXd> ds(x + 2 * Ndof_ + CartDof_ + 1, 1);
    // vec
    Eigen::Map<const Eigen::VectorXd> lambda_q(vec, Ndof_);
    Eigen::Map<const Eigen::VectorXd> lambda_q_ref(vec + Ndof_, Ndof_);
    Eigen::Map<const Eigen::VectorXd> lambda_F(vec + 2 * Ndof_, CartDof_);
    Eigen::Map<const Eigen::VectorXd> lambda_ds(vec + 2 * Ndof_ + CartDof_ + 1, 1);
    // output
    Eigen::Map<Eigen::VectorXd> out_df_u(out, Ndof_);
    Eigen::Map<Eigen::VectorXd> out_df_dds(out + Ndof_, 1);

    const MotionData &fk_data = horizon_cache_motion_data_.get(t);
    const MotionReference &mr_data = horizon_cache_motion_reference_.get(t);

    out_df_u = lambda_q_ref + lambda_q;
    // force dynamic only if force control is active
    if (mr_data.wghts_.Q_F(0, 0) > 0 || mr_data.wghts_.Q_F(1, 1) > 0 || mr_data.wghts_.Q_F(2, 2) > 0)
    {
      out_df_u += fk_data.ee_jac_.transpose() * Kges_ * lambda_F;
    }
    // path dynamic
    out_df_dds = lambda_ds;
  }
  void MPICProblemDescription::dfdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *vec, ctypeRNum *u, ctypeRNum *p)
  {
  }

  void MPICProblemDescription::lfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes)
  {
    // input states
    Eigen::Map<const Eigen::VectorXd> q(x, Ndof_);
    Eigen::Map<const Eigen::VectorXd> q_ref(x + Ndof_, Ndof_);
    Eigen::Map<const Eigen::VectorXd> F(x + 2 * Ndof_, CartDof_);
    Eigen::Map<const Eigen::VectorXd> s(x + 2 * Ndof_ + CartDof_, 1);
    Eigen::Map<const Eigen::VectorXd> ds(x + 2 * Ndof_ + CartDof_ + 1, 1);

    // input control
    Eigen::Map<const Eigen::VectorXd> dq_ref(u, Ndof_);
    Eigen::Map<const Eigen::VectorXd> dds(u + Ndof_, 1);

    const MotionData &fk_data = horizon_cache_motion_data_.get(t);
    const MotionReference &mr_data = horizon_cache_motion_reference_.get(t);

    lrt_mpic::computePoseError(pose_error_, orientation_error_, fk_data.ee_pose_, mr_data.setPnt_.p_des, fk_data.ee_ori_, orientation_des_, fk_data.ee_rot_mat_);

    cost_ = 0.0;
    out[0] = cost_;

    weightedNorm(cost_, q, mr_data.setPnt_.q_des, mr_data.wghts_.Q_q, N_q_);
    out[0] += cost_;

    weightedNorm(cost_, pose_error_, mr_data.wghts_.Q_p, N_p_);
    out[0] += cost_;

    weightedNorm(cost_, F, mr_data.setPnt_.F_des, mr_data.wghts_.Q_F, N_F_);
    out[0] += cost_;

    weightedNorm(cost_, dq_ref, mr_data.setPnt_.dq_des, mr_data.wghts_.R, Eigen::MatrixXd::Identity(Ndof_, Ndof_));
    out[0] += cost_;

    // path dynamic
    if (fabs(s[0] - s_f_) > DELTA_S)
    {
      out[0] += mr_data.wghts_.Q_s * fabs(s[0] - s_f_) - 0.5 * DELTA_S;
    }
    else
    {
      out[0] += 0.5 * (1.0 / DELTA_S) * mr_data.wghts_.Q_s * (s[0] - s_f_) * (s[0] - s_f_);
    }
    out[0] += 0.5 * mr_data.wghts_.Q_ds * (ds[0] - mr_data.setPnt_.ds_des) * (ds[0] - mr_data.setPnt_.ds_des);
    out[0] += 0.5 * mr_data.wghts_.R_s * (dds[0] - 0.0) * (dds[0] - 0.0);
  }
  void MPICProblemDescription::dldx(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes)
  {
    // input states
    Eigen::Map<const Eigen::VectorXd> q(x, Ndof_);
    Eigen::Map<const Eigen::VectorXd> q_ref(x + Ndof_, Ndof_);
    Eigen::Map<const Eigen::VectorXd> F(x + 2 * Ndof_, CartDof_);
    Eigen::Map<const Eigen::VectorXd> s(x + 2 * Ndof_ + CartDof_, 1);
    Eigen::Map<const Eigen::VectorXd> ds(x + 2 * Ndof_ + CartDof_ + 1, 1);

    // output
    Eigen::Map<Eigen::VectorXd> out_dl_q(out, Ndof_);
    Eigen::Map<Eigen::VectorXd> out_dl_F(out + 2 * Ndof_, CartDof_);
    Eigen::Map<Eigen::VectorXd> out_dl_s(out + 2 * Ndof_ + CartDof_, 1);
    Eigen::Map<Eigen::VectorXd> out_dl_ds(out + 2 * Ndof_ + CartDof_ + 1, 1);

    const MotionData &fk_data = horizon_cache_motion_data_.get(t);
    const MotionReference &mr_data = horizon_cache_motion_reference_.get(t);

    lrt_mpic::computePoseError(pose_error_, orientation_error_, fk_data.ee_pose_, mr_data.setPnt_.p_des, fk_data.ee_ori_, orientation_des_, fk_data.ee_rot_mat_);

    // partial derivative of pose
    dp_dq_ = Eigen::VectorXd::Zero(Ndof_);
    dp_dq_ += fk_data.ee_jac_.transpose().block(0, 0, 7, 3) * mr_data.wghts_.Q_p.block(0, 0, 3, 3) * pose_error_.head(3);     // position
    dp_dq_ += (fk_data.ee_jac_.block(3, 0, 3, 7)).transpose() * mr_data.wghts_.Q_p.block(3, 3, 3, 3) * (pose_error_.tail(3)); // orientation

    // partial derivative of joint position and force
    dWeightedNorm_dx(dl_dq_, q, mr_data.setPnt_.q_des, mr_data.wghts_.Q_q, N_q_);
    out_dl_q = dl_dq_ + dp_dq_;

    dWeightedNorm_dx(dlq_ds_, q, mr_data.setPnt_.q_des, mr_data.wghts_.Q_q, N_q_);
    out_dl_s(0) = -1.0 * dlq_ds_.dot(mr_data.setPntGradient_.q_des);

    dWeightedNorm_dx(dlp_ds_, pose_error_, mr_data.wghts_.Q_p, N_p_);
    Eigen::VectorXd pose_grad_tmp_ = Eigen::VectorXd::Zero(CartDof_);
    pose_grad_tmp_.head(3) = mr_data.setPntGradient_.p_des.head(3);
    pose_grad_tmp_.tail(3) = mr_data.setPntGradient_.p_des.tail(3);
    out_dl_s(0) += -1 * dlp_ds_.dot(pose_grad_tmp_);

    dWeightedNorm_dx(dl_dF_, F, mr_data.setPnt_.F_des, mr_data.wghts_.Q_F, N_F_);
    out_dl_F = dl_dF_;

    dWeightedNorm_dx(dlF_ds_, F, mr_data.setPnt_.F_des, mr_data.wghts_.Q_F, N_F_);
    out_dl_s(0) += -1.0 * dlF_ds_.dot(mr_data.setPntGradient_.F_des);

    // partial derivative of progress parameter
    if (fabs(s[0] - s_f_) > DELTA_S)
    {
      if ((s[0] - s_f_) > 0.0)
      {
        out_dl_s[0] += mr_data.wghts_.Q_s;
      }
      else
      {
        out_dl_s[0] += -1.0 * mr_data.wghts_.Q_s;
      }
    }
    else
    {
      out_dl_s[0] += (1.0 / DELTA_S) * mr_data.wghts_.Q_s * (s[0] - s_f_);
    }
    out_dl_ds[0] += mr_data.wghts_.Q_ds * (ds[0] - mr_data.setPnt_.ds_des);
  }
  void MPICProblemDescription::dldu(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes)
  {
    // input control
    Eigen::Map<const Eigen::VectorXd> dq_ref(u, Ndof_);
    Eigen::Map<const Eigen::VectorXd> dds(u + Ndof_, 1);

    // output
    Eigen::Map<Eigen::VectorXd> out_dl_dq_ref(out, Ndof_);
    Eigen::Map<Eigen::VectorXd> out_dl_ds(out + Ndof_, 1);

    const MotionReference &mr_data = horizon_cache_motion_reference_.get(t);

    dWeightedNorm_dx(dl_du_, dq_ref, mr_data.setPnt_.dq_des, mr_data.wghts_.R, Eigen::MatrixXd::Identity(Ndof_, Ndof_));
    out_dl_dq_ref = dl_du_;
    out_dl_ds[0] = mr_data.wghts_.R_s * (dds[0] - 0.0);
  }
  void MPICProblemDescription::dldp(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes)
  {
  }
  void MPICProblemDescription::Vfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes)
  {
  }
  void MPICProblemDescription::dVdx(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes)
  {
  }
  void MPICProblemDescription::dVdp(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes)
  {
  }
  void MPICProblemDescription::dVdT(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes)
  {
  }
  void MPICProblemDescription::gfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p)
  {
  }
  void MPICProblemDescription::dgdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec)
  {
  }
  void MPICProblemDescription::dgdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec)
  {
  }
  void MPICProblemDescription::dgdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec)
  {
  }
  void MPICProblemDescription::hfct(typeRNum *out, typeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p)
  {
    // states
    Eigen::Map<const Eigen::VectorXd> q(x, Ndof_);
    Eigen::Map<const Eigen::VectorXd> q_ref(x + Ndof_, Ndof_);
    Eigen::Map<const Eigen::VectorXd> F(x + 2 * Ndof_, CartDof_);
    Eigen::Map<const Eigen::VectorXd> s(x + 2 * Ndof_ + CartDof_, 1);
    Eigen::Map<const Eigen::VectorXd> ds(x + 2 * Ndof_ + CartDof_ + 1, 1);
    Eigen::Map<const Eigen::VectorXd> dq_ref(u, Ndof_);

    // joint position
    Eigen::Map<Eigen::VectorXd> out_h_q_max(out, Ndof_);
    Eigen::Map<Eigen::VectorXd> out_h_q_min(out + Ndof_, Ndof_);
    // joint velocity
    Eigen::Map<Eigen::VectorXd> out_h_dq_max(out + 2 * Ndof_, Ndof_);
    Eigen::Map<Eigen::VectorXd> out_h_dq_min(out + 3 * Ndof_, Ndof_);
    // EE force
    Eigen::Map<Eigen::VectorXd> out_h_F_max(out + 4 * Ndof_, 3);
    Eigen::Map<Eigen::VectorXd> out_h_F_min(out + 4 * Ndof_ + 3, 3);
    // EE position
    Eigen::Map<Eigen::VectorXd> out_h_p_max(out + 4 * Ndof_ + 6, 3);
    Eigen::Map<Eigen::VectorXd> out_h_p_min(out + 4 * Ndof_ + 9, 3);
    // EE velocity
    Eigen::Map<Eigen::VectorXd> out_h_v_max(out + 4 * Ndof_ + 12, 3);
    Eigen::Map<Eigen::VectorXd> out_h_v_min(out + 4 * Ndof_ + 15, 3);
    // progress s
    Eigen::Map<Eigen::VectorXd> out_h_s_max(out + 4 * Ndof_ + 18, 1);
    // progress velocity ds
    Eigen::Map<Eigen::VectorXd> out_h_ds_max(out + 4 * Ndof_ + 19, 1);
    Eigen::Map<Eigen::VectorXd> out_h_ds_min(out + 4 * Ndof_ + 20, 1);

    const MotionData &fk_data = horizon_cache_motion_data_.get(t);
    const MotionReference &mr_data = horizon_cache_motion_reference_.get(t);

    // Joint position constraints
    if (mr_data.constr_.q_constr)
    {
      out_h_q_max = q - mr_data.constr_.q_max;
      out_h_q_min = mr_data.constr_.q_min - q;
    }
    else
    {
      out_h_q_max = Eigen::VectorXd::Zero(Ndof_);
      out_h_q_min = Eigen::VectorXd::Zero(Ndof_);
    }

    // EE force constraints
    if (mr_data.constr_.F_constr)
    {
      out_h_F_max = F - mr_data.constr_.F_max;
      out_h_F_min = mr_data.constr_.F_min - F;
    }
    else
    {
      out_h_F_max = Eigen::VectorXd::Zero(Ndof_);
      out_h_F_min = Eigen::VectorXd::Zero(Ndof_);
    }

    // EE/Ellbow position constraints
    if (mr_data.constr_.p_constr)
    {
      // forward kinematic
      if (mr_data.constr_.p_constr_elbow)
      {
        pinocchio::forwardKinematics(model_, *data_, q);
        pinocchio::updateFramePlacements(model_, *data_);
        pinocchio::FrameIndex elbow_id = model_.getFrameId("panda_link4");
        position_ = data_->oMf[elbow_id].translation();

        out_h_p_max = 2 * (position_ - mr_data.constr_.p_max);
        out_h_p_min = 2 * (mr_data.constr_.p_min - position_);
      }
      else
      {
        out_h_p_max = 2 * (fk_data.ee_pos_ - mr_data.constr_.p_max);
        out_h_p_min = 2 * (mr_data.constr_.p_min - fk_data.ee_pos_);
      }
    }
    else
    {
      out_h_p_max = Eigen::VectorXd::Zero(Ndof_);
      out_h_p_min = Eigen::VectorXd::Zero(Ndof_);
    }

    // EE velocity constraints
    if (mr_data.constr_.v_constr)
    {
      // position dynamic
      Eigen::VectorXd v_ee = fk_data.ee_jac_.block(0, 0, 3, 7) * dq_ref;

      out_h_v_max = v_ee - mr_data.constr_.v_max;
      out_h_v_min = -mr_data.constr_.v_max - v_ee;
    }
    else
    {
      out_h_v_max = Eigen::VectorXd::Zero(Ndof_);
      out_h_v_min = Eigen::VectorXd::Zero(Ndof_);
    }

    // progress s
    out_h_s_max[0] = s[0] - 2;
    // progress velocity
    out_h_ds_max[0] = ds[0] - 0.75;
    out_h_ds_min[0] = -0.75 - ds[0];
  }
  void MPICProblemDescription::dhdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec)
  {
    // --- x ---
    // states
    Eigen::Map<const Eigen::VectorXd> q(x, Ndof_);
    Eigen::Map<const Eigen::VectorXd> q_ref(x + Ndof_, Ndof_);
    Eigen::Map<const Eigen::VectorXd> F(x + 2 * Ndof_, CartDof_);
    Eigen::Map<const Eigen::VectorXd> s(x + 2 * Ndof_ + CartDof_, 1);
    Eigen::Map<const Eigen::VectorXd> ds(x + 2 * Ndof_ + CartDof_ + 1, 1);
    Eigen::Map<const Eigen::VectorXd> dq_ref(u, Ndof_);
    // --- vec ---
    // joint position
    Eigen::Map<const Eigen::VectorXd> h_q_max(vec, Ndof_);
    Eigen::Map<const Eigen::VectorXd> h_q_min(vec + Ndof_, Ndof_);
    // joint velocity
    Eigen::Map<const Eigen::VectorXd> h_dq_max(vec + 2 * Ndof_, Ndof_);
    Eigen::Map<const Eigen::VectorXd> h_dq_min(vec + 3 * Ndof_, Ndof_);
    // EE force
    Eigen::Map<const Eigen::VectorXd> h_F_max(vec + 4 * Ndof_, 3);
    Eigen::Map<const Eigen::VectorXd> h_F_min(vec + 4 * Ndof_ + 3, 3);
    // EE position
    Eigen::Map<const Eigen::VectorXd> h_p_max(vec + 4 * Ndof_ + 6, 3);
    Eigen::Map<const Eigen::VectorXd> h_p_min(vec + 4 * Ndof_ + 9, 3);
    // progress
    Eigen::Map<const Eigen::VectorXd> h_s_max(vec + 4 * Ndof_ + 12, 1);
    // progress velocity
    Eigen::Map<const Eigen::VectorXd> h_ds_max(vec + 4 * Ndof_ + 13, 1);
    Eigen::Map<const Eigen::VectorXd> h_ds_min(vec + 4 * Ndof_ + 14, 1);

    // --- out ---
    Eigen::Map<Eigen::VectorXd> out_dh_q(out, Ndof_);
    Eigen::Map<Eigen::VectorXd> out_dh_q_ref(out + Ndof_, Ndof_);
    Eigen::Map<Eigen::VectorXd> out_dh_F(out + 2 * Ndof_, CartDof_);
    Eigen::Map<Eigen::VectorXd> out_dh_s(out + 2 * Ndof_ + CartDof_, 1);
    Eigen::Map<Eigen::VectorXd> out_dh_ds(out + 2 * Ndof_ + CartDof_ + 1, 1);

    const MotionData &fk_data = horizon_cache_motion_data_.get(t);
    const MotionReference &mr_data = horizon_cache_motion_reference_.get(t);

    // joint position constraint
    if (mr_data.constr_.q_constr)
    {
      out_dh_q = h_q_max - h_q_min; //
    }
    else
    {
      out_dh_q = Eigen::VectorXd::Zero(Ndof_);
    }
    // joint velocity constraint
    //    if(mr_data.constr_.dq_constr){
    //      out_dh_dq_ref = h_dq_max - h_dq_min;//
    //    }else{
    //      out_dh_dq_ref =Eigen::VectorXd::Zero(Ndof_);
    //    }
    // force constraint
    if (mr_data.constr_.F_constr)
    {
      out_dh_F.block(0, 0, 3, 1) = h_F_max - h_F_min; //
      out_dh_F.block(3, 0, 3, 1) = Eigen::VectorXd::Zero(3);
    }
    else
    {
      out_dh_F = Eigen::VectorXd::Zero(6);
    }

    // Cartesian position constraint (end-effector or elbow)
    if (mr_data.constr_.p_constr)
    {
      if (mr_data.constr_.p_constr_elbow)
      {
        pinocchio::FrameIndex elbow_id = model_.getFrameId("panda_link4");
        pinocchio::computeFrameJacobian(model_, *data_, q, elbow_id, pinocchio::LOCAL_WORLD_ALIGNED, jac_);

        out_dh_q += jac_.block(0, 0, 3, 7).transpose() * (h_p_max - h_p_min);
      }
      else
      {
        out_dh_q += fk_data.ee_jac_.block(0, 0, 3, 7).transpose() * (h_p_max - h_p_min);
      }
    }
    else
    {
      out_dh_q += Eigen::VectorXd::Zero(Ndof_);
    }

    // progress s
    out_dh_s[0] = h_s_max[0];
    // progress velocity ds
    out_dh_ds[0] = (h_ds_max[0] - h_ds_min[0]);
  }

  void MPICProblemDescription::dhdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec)
  {
    Eigen::Map<const Eigen::VectorXd> q_ref(x + Ndof_, Ndof_);
    Eigen::Map<const Eigen::VectorXd> dq_ref(u, Ndof_);

    // EE velocity
    Eigen::Map<const Eigen::VectorXd> h_v_max(vec + 4 * Ndof_ + 12, 3);
    Eigen::Map<const Eigen::VectorXd> h_v_min(vec + 4 * Ndof_ + 15, 3);

    // --- out ---
    Eigen::Map<Eigen::VectorXd> out_dh_dq_ref(out, Ndof_);

    const MotionData &fk_data = horizon_cache_motion_data_.get(t);
    const MotionReference &mr_data = horizon_cache_motion_reference_.get(t);

    // Cartesian velocity constraint (end-effector)
    if (mr_data.constr_.v_constr)
    {
      out_dh_dq_ref = fk_data.ee_jac_.block(0, 0, 3, 7).transpose() * (h_v_max - h_v_min); //
    }
    else
    {
      out_dh_dq_ref = Eigen::VectorXd::Zero(Ndof_);
    }
  }
  void MPICProblemDescription::dhdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec)
  {
  }
  void MPICProblemDescription::gTfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p)
  {
  }
  void MPICProblemDescription::dgTdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec)
  {
  }
  void MPICProblemDescription::dgTdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec)
  {
  }
  void MPICProblemDescription::dgTdT_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec)
  {
  }
  void MPICProblemDescription::hTfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p)
  {
  }
  void MPICProblemDescription::dhTdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec)
  {
  }
  void MPICProblemDescription::dhTdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec)
  {
  }
  void MPICProblemDescription::dhTdT_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec)
  {
  }

  lrt_mpic::setPoint MPICProblemDescription::getActSetPoint()
  {
    return sharedSetPnt_;
  }

  void MPICProblemDescription::setDesiredProgressParam(const double &s_f)
  {
    s_f_ = s_f;
  }

  void MPICProblemDescription::setMPs(std::vector<std::shared_ptr<lrt_mpic::ManipulationPrimitive>> &MP_vec)
  {
    for (int i = 0; i < 2; i++)
    {
      if (MP_vec.at(i))
      {
        MP_vec_.at(i).reset();
        MP_vec_.at(i) = MP_vec.at(i)->clone();
      }
    }
  }

  void MPICProblemDescription::setControllerParam(const lrt_mpic::controlGain &ctrGain)
  {
    ctrGains_ = ctrGain;
    for (int i = 0; i < Ndof_; i++)
    {
      ctrGains_.Dctr_inv(i, i) = 1 / (ctrGains_.Dpd(i, i));
      ctrGains_.K_D(i, i) = ctrGains_.Kpd(i, i) / ctrGains_.Dpd(i, i);
    }
  }

  void MPICProblemDescription::setControllerOpt(const lrt_mpic::controlOptions &ctrOpt)
  {
    ctrOpt_ = ctrOpt;
  }

  void MPICProblemDescription::setKges(const Eigen::MatrixXd &Kges, const Eigen::MatrixXd Q_F)
  {
    Kges_ = Eigen::MatrixXd::Zero(CartDof_, CartDof_);
    Eigen::MatrixXd S = Eigen::MatrixXd::Zero(CartDof_, CartDof_);
    for (int i = 0; i < CartDof_; i++)
    {
      if (Q_F(i, i) > 0.0)
      {
        S(i, i) = 1;
      }
    }
    Kges_ = S * Kges;
  }

  void MPICProblemDescription::weightedNorm(double &cost, const Eigen::VectorXd &ist, const Eigen::VectorXd &soll, const Eigen::MatrixXd &weights, const Eigen::MatrixXd &normalization)
  {
    cost = 0.5 * (normalization * (ist - soll)).transpose() * weights * normalization * (ist - soll);
    return;
  }

  void MPICProblemDescription::weightedNorm(double &cost, const Eigen::VectorXd &error, const Eigen::MatrixXd &weights, const Eigen::MatrixXd &normalization)
  {
    cost = 0.5 * (normalization * error).transpose() * weights * normalization * error;
    return;
  }

  void MPICProblemDescription::dWeightedNorm_dx(Eigen::VectorXd &dl_dx, const Eigen::VectorXd &ist, const Eigen::VectorXd &soll, const Eigen::MatrixXd &weights, const Eigen::MatrixXd &normalization)
  {
    dl_dx = weights * normalization * normalization * (ist - soll);
    return;
  }

  void MPICProblemDescription::dWeightedNorm_dx(Eigen::VectorXd &dl_dx, const Eigen::VectorXd &error, const Eigen::MatrixXd &weights, const Eigen::MatrixXd &normalization)
  {
    dl_dx = weights * normalization * normalization * error;
    return;
  }

  void MPICProblemDescription::copySetPntData(lrt_mpic::setPoint &setPnt_out, const lrt_mpic::setPoint setPnt_in)
  {
    sharedSetPnt_.q_des = setPnt_in.q_des;
    sharedSetPnt_.p_des = setPnt_in.p_des;
    sharedSetPnt_.F_des = setPnt_in.F_des;
  }

  // Compute Cartesian pose from measured joint states
  void MPICProblemDescription::motionDataFunction(MotionData &md, const pinocchio::Model &model, pinocchio::Data *data, const pinocchio::FrameIndex &flange_id, const Eigen::VectorXd &q)
  {
    pinocchio::forwardKinematics(model, *data, q);
    pinocchio::updateFramePlacements(model, *data);
    md.ee_rot_mat_ = data->oMf[flange_id].rotation();
    md.ee_pos_ = data->oMf[flange_id].translation();
    md.ee_ori_ = md.ee_rot_mat_;
    md.ee_pose_.head(3) = md.ee_pos_;
    md.ee_pose_[3] = md.ee_ori_.w();
    md.ee_pose_.tail(3) = md.ee_ori_.vec();
    pinocchio::computeFrameJacobian(model_, *data_, q, flange_id_, pinocchio::LOCAL_WORLD_ALIGNED, md.ee_jac_);
  }

  void MPICProblemDescription::motionReferenceFunction(MotionReference &mr, const double &s, const Eigen::VectorXd &q, const Eigen::VectorXd &p, const Eigen::VectorXd &F)
  {
    double s_loc;
    if (MP_vec_.at(0)->evaluateMP(s))
    {
      iMP_ = 1;
      s_loc = s - 1;
    }
    else
    {
      iMP_ = 0;
      s_loc = s;
    }

    if (!MP_vec_.at(iMP_))
    {
      std::cout << "[MPICProblemDescription::motionReferenceFunction]: Error! MP_vec_.at(iMP_) does not exist. iMP_ = " << iMP_ << std::endl;
    }
    else
    {
      mr.constr_ = *(MP_vec_.at(iMP_)->getConstraints());
      mr.wghts_ = *(MP_vec_.at(iMP_)->getWeights());
      // mr.wghts_ = *(MP_vec_.at(iMP_)->getScaledWeights(s_loc,s_f_));      // reduce speed at the beginning and end of the MP
      mr.setPnt_ = *(MP_vec_.at(iMP_)->getSetPoint(s_loc, q, p, F));
      mr.setPntGradient_ = *(MP_vec_.at(iMP_)->getSetPointGradient(s_loc));
    }
  }

}
