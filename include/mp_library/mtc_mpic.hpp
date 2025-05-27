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

#ifndef MTC_MPIC_H
#define MTC_MPIC_H

#include <mpic_utils.h>
#include "../mp_framework.h"
#include "../mpic_transition.h"

#include "../../libs/yaml-cpp/include/yaml-cpp/yaml.h"

namespace lrt_mpic
{
  struct MtcMPICParam
  {
    /* MPC weights */
    std::vector<double> R;
    std::vector<double> Qq;
    std::vector<double> Qp;
    std::vector<double> QF;
    std::vector<double> Rs;
    std::vector<double> Qs;
    std::vector<double> Qds;

    /* Number of discrete samples used to define the reference path */
    std::vector<double> nSamples;

    /* Constraints */
    std::vector<double> qmin;
    std::vector<double> qmax;
    std::vector<double> pmin;
    std::vector<double> pmax;
    std::vector<double> Fmin;
    std::vector<double> Fmax;

    /* Vectors to store the samples of the reference path */
    std::vector<double> q1, q2, q3, q4, q5, q6, q7;
    std::vector<double> px, py, pz, quat_0, quat_1, quat_2, quat_3;
    std::vector<double> Fx, Fy, Fz, Mx, My, Mz;

    /* Gripper states */
    std::vector<double> gripper_width;
    std::vector<double> gripper_speed;
    std::vector<double> gripper_force;

    /* Vector of progress parameter samples */
    std::vector<double> s;
  };

  /**
     * This class is the "move to contact" MP
  */
  class MtcMPIC : public ManipulationPrimitive
  {
  public:
    /* Constructors */
    MtcMPIC();
    MtcMPIC(const std::string folder_path, const std::string mp_file_name);

    /* Clone MP */
    std::shared_ptr<ManipulationPrimitive> clone() const override;

    /* Replan the reference path from current position */
    void rePlanPath(const double s, const Eigen::VectorXd &q, const Eigen::VectorXd &p, const Eigen::VectorXd &F) override;

    /* Evaluate termination conditions of current MP */
    bool evaluateMP(const double &s) override;
    FinishFlag evaluateMP(const double &s, const Eigen::VectorXd &q, const Eigen::VectorXd &p, const Eigen::VectorXd &F) override;
    
    /* Get current value of prograss parameter */
    GenericPath *getPathParam() override;

    /* Get current setpoint for the MPC */
    setPoint *getSetPoint(const double s, const Eigen::VectorXd &q, const Eigen::VectorXd &p, const Eigen::VectorXd &F) override;
    
    /* Get current gradient (setpoint/dx) for the MPC */
    setPoint *getSetPointGradient(const double s) override;
    
    /* Get weighting matrices of the MP */
    weights *getWeights() override;

    /* Get weighting matrices of the MP, Qds is scaled at the beginning and end of the MP which causes slower robot motions */
    weights *getScaledWeights(const double s, const double s_f) override;
    
    /* Get constraints */
    constraints *getConstraints() override;

    /* Get MP name */
    std::string getName() override;

    /* Get MP type */
    MPtype *getType() override;

    /* Get initial progress of the reference path */
    double getInitialProgressParam() override;

    /* Get terminal progress of the reference path */
    double getTerminationProgressParam() override;

    /* Get reaction strategy */
    ReactionStrategy getReactionStrategy() override;

    /* Get exception strategy */
    ExceptionStrategy getExceptionStrategy() override;

    /* Print MP parameterization */
    void print() override;

  private:
  /* Init MP from .yaml-file */
    bool initializeFromFile(const std::string folder_path, const std::string mp_file_name) override;

    /* Linear interpolation between two samples of the reference path */
    void linearPath(const double s);

    /* Gradient between two samples of the reference path */
    void linearPathGradient(const double s);

    /* Yaml object to store the MP parameters from a .yaml-file */
    YAML::Node mp_node_;

    /* Numder of robot joints (Ndof) and Cartesian degrees-of-freedom (CartDof) */
    unsigned Ndof_, CartDof_;

    /* MP name */
    std::string name_;

    /* MP type */
    MPtype type_;

    /* MP parameterization struct */
    MtcMPICParam mp_param_;

    /* Local progress parameter which 0 <= sloc_ <= 1 */
    double sloc_;

    /* Current index along the samples of the reference path (needed for linear interpolation between two neighboring samples) */
    unsigned index_;

    /* Force threshold value for contact detection */
    double f_th_;

    /* Maximum distance for searching contact further than final position of the reference path */
    double delta_z_;

    /* Success conditions */
    TerminationCondition transition_cond_;

    /* Fault conditions */
    TerminationCondition error_cond_;

    /* Reaction strategy */
    ReactionStrategy reaction_;

    /* Exception strategy */
    ExceptionStrategy exception_;

    /* Reference path */
    GenericPath path_;

    /* Current set point along the reference path */
    setPoint pathSetPnt_;

    /* Gradient of current set point along the reference path */
    setPoint pathSetPntGradient_;

    /* Weighting matrices */
    weights wghts_;

    /* Scaled weighting matrices */
    weights wghtsScaled_;

    /* Marks small environment around desired progress for switching to quadratic weighting of the progress */
    double delta_s_loc_;

    /* Constraints */
    constraints constr_;

    /* Quaternions */
    Eigen::Quaterniond Q0_;
    Eigen::Quaterniond Qf_;
    Eigen::Quaterniond Q_;
    Eigen::Quaterniond Qtmp_;
    Eigen::Quaterniond Qlog_;
  };
} // namespace lrt_mpic
#endif // MTC_MPIC_H