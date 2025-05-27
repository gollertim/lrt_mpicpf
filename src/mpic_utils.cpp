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

#include "mpic_utils.h"
#include <ros/ros.h>

namespace lrt_mpic
{

  void initPath(GenericPath &path, const unsigned nSamples)
  {
    path.nSamples = nSamples;

    path.s = Eigen::VectorXd::Zero(nSamples);

    // Joint Position
    path.q.q_1 = Eigen::VectorXd::Zero(nSamples);
    path.q.q_2 = Eigen::VectorXd::Zero(nSamples);
    path.q.q_3 = Eigen::VectorXd::Zero(nSamples);
    path.q.q_4 = Eigen::VectorXd::Zero(nSamples);
    path.q.q_5 = Eigen::VectorXd::Zero(nSamples);
    path.q.q_6 = Eigen::VectorXd::Zero(nSamples);
    path.q.q_7 = Eigen::VectorXd::Zero(nSamples);
    // Pose
    path.pose.p_x = Eigen::VectorXd::Zero(nSamples);
    path.pose.p_y = Eigen::VectorXd::Zero(nSamples);
    path.pose.p_z = Eigen::VectorXd::Zero(nSamples);
    path.pose.q_0 = Eigen::VectorXd::Zero(nSamples);
    path.pose.q_1 = Eigen::VectorXd::Zero(nSamples);
    path.pose.q_2 = Eigen::VectorXd::Zero(nSamples);
    path.pose.q_3 = Eigen::VectorXd::Zero(nSamples);
    // Wrench
    path.F.F_x = Eigen::VectorXd::Zero(nSamples);
    path.F.F_y = Eigen::VectorXd::Zero(nSamples);
    path.F.F_z = Eigen::VectorXd::Zero(nSamples);
    path.F.M_x = Eigen::VectorXd::Zero(nSamples);
    path.F.M_y = Eigen::VectorXd::Zero(nSamples);
    path.F.M_z = Eigen::VectorXd::Zero(nSamples);
    // Gripper
    path.gripper.width = 0;
    path.gripper.speed = 0;
  }

  // Set points
  void initSetPoint(setPoint &setPnt, const unsigned Ndof, const unsigned CartDof)
  {
    setPnt.q_des = Eigen::VectorXd::Zero(Ndof);
    setPnt.dq_des = Eigen::VectorXd::Zero(Ndof);
    setPnt.ddq_des = Eigen::VectorXd::Zero(Ndof);
    setPnt.p_des = Eigen::VectorXd::Zero(CartDof + 1); // quaternions
    setPnt.v_des = Eigen::VectorXd::Zero(CartDof);
    setPnt.F_des = Eigen::VectorXd::Zero(CartDof);
    setPnt.s_des = 1.0;
    setPnt.ds_des = 0;
    setPnt.dds_des = 0;
  }

  // Local progress parameter: [0,1]
  void calcLocalPathParam(double &sloc, const double s, const double s_0, const double s_f)
  {
    sloc = (s - s_0) / (s_f - s_0);
  }

  void selectIndexOnPath(unsigned &index, const Eigen::VectorXd &s_vec, const double &s)
  {
    for (unsigned j = 0; j < s_vec.size(); j++)
    {
      if (s > s_vec[j])
      {
        index = j;
      };
    }
  }

  // Linear interpolation for path segment
  void lerp(double &x, const double &x_0, const double x_des, const double &sloc)
  {
    x = x_0 + (x_des - x_0) * sloc;
  }

  // Gradient of linear path segment
  void lerpGrad(double &grad, const double &x_0, const double x_des, const double s_0, const double s_f)
  {
    grad = (x_des - x_0) / (s_f - s_0);
  }

  // Logarithmus naturalis of a quaternion
  void QuaternionLogarithm(Eigen::Quaterniond &Qlog, const Eigen::Quaterniond &Q)
  {

    double normQ = Q.norm();
    double normIm = sqrt(Q.x() * Q.x() + Q.y() * Q.y() + Q.z() * Q.z());

    double phi = acos(Q.w() / normQ);

    Qlog.w() = log(normQ);
    Qlog.x() = phi * Q.x() / normIm;
    Qlog.y() = phi * Q.y() / normIm;
    Qlog.z() = phi * Q.z() / normIm;
  }

  // Death band filter
  double deathFilter(double in, double band)
  {
    double out;
    if (fabs(in) <= band)
    {
      out = 0.0;
    }
    else if (in > band)
    {
      out = in - band;
    }
    else if (in < band)
    {
      out = in + band;
    }
    return out;
  }

  // Pseudo inversion
  void pseudoInverse(const Eigen::MatrixXd &M_, Eigen::MatrixXd &M_pinv_, bool damped)
  {
    double lambda_ = damped ? 0.2 : 0.0;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
    Eigen::MatrixXd S_ = M_; // copying the dimensions of M_, its content is not needed.
    S_.setZero();

    for (int i = 0; i < sing_vals_.size(); i++)
      S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);

    M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
  }

  // Cartesian pose error
  void computePoseError(Eigen::VectorXd &pose_error, Eigen::Quaterniond &orientation_error, const Eigen::VectorXd &pose, const Eigen::VectorXd &pose_des, const Eigen::Quaterniond &orientation, Eigen::Quaterniond &orientation_des, const Eigen::Matrix3d &rot_mat)
  {
    pose_error.head(3) = pose.head(3) - pose_des.head(3);
    orientation_des.w() = pose_des[3];
    orientation_des.vec() = pose_des.tail(3);
    orientation_error = orientation_des.inverse() * orientation;
    pose_error.tail(3) = rot_mat * (orientation_error).vec();
  }

  // Compute Cartesian pose from measured joint states
  void computePose(Eigen::VectorXd &pose, Eigen::Vector3d position, Eigen::Quaterniond &orientation, Eigen::Matrix3d &rot_mat, const pinocchio::Model &model, pinocchio::Data *data, const pinocchio::FrameIndex &flange_id, const Eigen::VectorXd &q)
  {
    // forward kinematics
    pinocchio::forwardKinematics(model, *data, q);
    pinocchio::updateFramePlacements(model, *data);
    rot_mat = data->oMf[flange_id].rotation();
    position = data->oMf[flange_id].translation();
    orientation = rot_mat;
    pose.head(3) = position;
    pose[3] = orientation.w();
    pose.tail(3) = orientation.vec();
  }

}
