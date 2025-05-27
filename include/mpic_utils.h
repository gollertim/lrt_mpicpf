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

#ifndef LRT_CONTROLLER_UTILS_H
#define LRT_CONTROLLER_UTILS_H

#define PINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
#define PINOCCHIO_URDFDOM_USE_STD_SHARED_PTR

#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include "pinocchio/algorithm/joint-configuration.hpp"
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/frames-derivatives.hpp>

#include <Eigen/Dense>
#include <mutex>
#include <vector>
#include <iostream>
#include <fstream>

namespace lrt_mpic {

  /* Struct with vectors for each joint position */
  struct JointPosition{
    Eigen::VectorXd q_1;
    Eigen::VectorXd q_2;
    Eigen::VectorXd q_3;
    Eigen::VectorXd q_4;
    Eigen::VectorXd q_5;
    Eigen::VectorXd q_6;
    Eigen::VectorXd q_7;
  };

  /* Struct with vectors for each component of the cartesian pose with orientation as quaternions */
  struct Pose{
    Eigen::VectorXd p_x;
    Eigen::VectorXd p_y;
    Eigen::VectorXd p_z;
    Eigen::VectorXd q_0;
    Eigen::VectorXd q_1;
    Eigen::VectorXd q_2;
    Eigen::VectorXd q_3;
  };

  /* Struct with vectors for each component of the interaction wrench */
  struct Wrench{
    Eigen::VectorXd F_x;
    Eigen::VectorXd F_y;
    Eigen::VectorXd F_z;
    Eigen::VectorXd M_x;
    Eigen::VectorXd M_y;
    Eigen::VectorXd M_z;
  };

  /* Struct for gripper states */
  struct Gripper{
    double width, speed, force = 0.0;
  };

  /* Struct with individual elements of a path */
  struct GenericPath{
    unsigned nSamples;  // Number of samples
    Eigen::VectorXd s;  // Progress samples
    Pose pose;          // Cartesian pose
    JointPosition q;    // Joint positions
    Wrench F;           // Interaction wrench
    Gripper gripper;    // Gripper state
  };

  /* Weighting matrices */
  struct weights{
    Eigen::MatrixXd Q_q, Q_dq, Q_N, R;
    Eigen::MatrixXd Q_p, Q_v, Q_F;
    double Q_s, Q_ds, R_s;

    weights(){}
    weights(const unsigned Ndof, const unsigned CartDof)
      : Q_q(Eigen::MatrixXd::Zero(Ndof,Ndof)),        // Joint positions
      Q_dq(Eigen::MatrixXd::Zero(Ndof,Ndof)),         // Joint velocisties
      Q_N(Eigen::MatrixXd::Zero(Ndof,Ndof)),          // Nullspace
      R(Eigen::MatrixXd::Zero(Ndof,Ndof)),            // Input
      Q_p(Eigen::MatrixXd::Zero(CartDof,CartDof)),    // Cartesian Pose
      Q_v(Eigen::MatrixXd::Zero(CartDof,CartDof)),    // Cartesian velocity
      Q_F(Eigen::MatrixXd::Zero(CartDof,CartDof)),    // Interaction wrench
      Q_s(0.0),                                       // Path progress
      Q_ds(0.0),                                      // Progress velocity
      R_s(0.0)                                        // Input (progress accelearation)
    {}
  };
  // Constraints
  struct constraints{
    Eigen::VectorXd q_min, q_max;                 // Joint positions
    Eigen::VectorXd dq_max;                       // Joint velocities
    Eigen::VectorXd p_min, p_max;                 // Cartesian pose
    Eigen::VectorXd p_min_elbow, p_max_elbow;     // Elbow position
    Eigen::VectorXd v_max;                        // Cartesian velocity
    Eigen::VectorXd F_min, F_max;                 // Interaction wrench
    bool q_constr, dq_constr, p_constr, p_constr_elbow, v_constr, F_constr; // Flags for selecting active constraints

    constraints(){}
    constraints(const unsigned Ndof, const unsigned CartDof)
      : q_min(Eigen::VectorXd::Zero(Ndof)),
      q_max(Eigen::VectorXd::Zero(Ndof)),
      dq_max(Eigen::VectorXd::Zero(Ndof)),
      p_min(Eigen::VectorXd::Zero(CartDof)),
      p_max(Eigen::VectorXd::Zero(CartDof)),
      p_min_elbow(Eigen::VectorXd::Zero(3)),
      p_max_elbow(Eigen::VectorXd::Zero(3)),
      v_max(Eigen::VectorXd::Zero(CartDof)),
      F_min(Eigen::VectorXd::Zero(CartDof)),
      F_max(Eigen::VectorXd::Zero(CartDof)),
      q_constr(false),
      dq_constr(false),
      p_constr(false),
      p_constr_elbow(false),
      v_constr(false),
      F_constr(false)
    {}
  };

  // Set points
  struct setPoint{
    Eigen::VectorXd q_des, dq_des, ddq_des;
    Eigen::VectorXd p_des, v_des, F_des;
    double s_des, ds_des, dds_des;

    setPoint(){}
    setPoint(const unsigned Ndof, const unsigned CartDof)
      : q_des(Eigen::VectorXd::Zero(Ndof)),                 // Joint positions
      dq_des(Eigen::VectorXd::Zero(Ndof)),                  // Joint velocities
      ddq_des(Eigen::VectorXd::Zero(Ndof)),                 // Joint accelerations
      p_des(Eigen::VectorXd::Zero(CartDof)),                // Cartesian pose
      v_des(Eigen::VectorXd::Zero(CartDof)),                // Cartesian velocity
      F_des(Eigen::VectorXd::Zero(CartDof)),                // Interaction wrench
      s_des(0.0),                                           // Desired progress parameter
      ds_des(0.0),                                          // Desired progress velocity
      dds_des(0.0)                                          // desired progress acceleration
    {}
  };

  // Gains of the low level PD controller
  struct controlGain{
    Eigen::MatrixXd Kpd, Dpd, K_D, Dctr_inv;
  };

  // Optional settings
  struct controlOptions{
    bool updateWeights;       // Update weights at the beginning of each MPC cycle
    bool updateSetPoint;      // Update setpoint at the beginning of each MPC cycle
    int updateFreqMPIC;       // MPIC cycle time
    bool writeStates;         // Activate data logging
    bool writeHor=false;      // Activate logging of predicted data (only implemetned for wrench prediction)
    int writeFreq;            // Logging frequency
    bool pubStates;           // Activate publishing robot states
    int pubFreq;              // Publisher frequency
    bool skill;               // Activate task control state machine
    bool repeatTask;          // Repeat task after is has been finished
    bool extFaultDetect;      // Activcate external fault detection
    bool fts;                 // use additional force/torque sensor
  };


  // Initialization of Path struct
  void initPath(GenericPath& path, const unsigned nSamples);

  // Initialization of setPoint struct
  void initSetPoint(setPoint& setPnt, const unsigned Ndof, const unsigned CartDof);

  // Local progress parameter: [0,1]
  void calcLocalPathParam(double &sloc, const double s, const double s_0, const double s_f);

  // select index on path
  void selectIndexOnPath(unsigned & index, const Eigen::VectorXd &s_vec, const double &s);

  // Linear interpolation for path segment
  void lerp(double &x, const double &x_0, const double x_des, const double &sloc);

  // Gradient of linear path segment
  void lerpGrad(double &grad, const double &x_0, const double x_des, const double s_0, const double s_f);

  // Logarithmus naturalis of a quaternion
  void QuaternionLogarithm(Eigen::Quaterniond &Qlog, const Eigen::Quaterniond &Q);

  // Death band filter
  double deathFilter(double in, double band);

  // Pseudo inversion
  void pseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_, bool damped = true);

  // Cartesian pose error
  void computePoseError(Eigen::VectorXd &pose_error, Eigen::Quaterniond &orientation_error, const Eigen::VectorXd &pose, const Eigen::VectorXd &pose_des, const Eigen::Quaterniond &orientation, Eigen::Quaterniond &orientation_des, const Eigen::Matrix3d &rot_mat);

  // Compute Cartesian pose from measured joint states
  void computePose(Eigen::VectorXd &pose, Eigen::Vector3d position, Eigen::Quaterniond &orientation, Eigen::Matrix3d &rot_mat, const pinocchio::Model &model, pinocchio::Data *data, const pinocchio::FrameIndex &flange_id, const Eigen::VectorXd &q);


}  // namespace lrt_controller
#endif // LRT_CONTROLLER_UTILS_H
