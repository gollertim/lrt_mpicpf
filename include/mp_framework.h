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

#ifndef MP_FRAMEWORK_H
#define MP_FRAMEWORK_H

#include <Eigen/Dense>
#include <mutex>
#include <vector>
#include <iostream>
#include <fstream>
#include "mpic_utils.h"
#include "mpic_exception.h"
#include "mpic_statemachine.h"
#include "mp_library/mp_types.h"

#include "../../libs/yaml-cpp/include/yaml-cpp/yaml.h"

namespace lrt_mpic
{
  /**
     * This class is the basic structure for all MPs
  */
  class ManipulationPrimitive
  {
  public:
    /* Clone MP */
    virtual std::shared_ptr<ManipulationPrimitive> clone() const = 0;

    /* Replan the reference path from current position */
    virtual void rePlanPath(const double s, const Eigen::VectorXd &q, const Eigen::VectorXd &p, const Eigen::VectorXd &F) = 0;

    /* Evaluate termination conditions of current MP */
    virtual bool evaluateMP(const double &s) = 0;
    virtual FinishFlag evaluateMP(const double &s, const Eigen::VectorXd &q, const Eigen::VectorXd &p, const Eigen::VectorXd &F) = 0;
    
    /* Get current value of prograss parameter */
    virtual GenericPath *getPathParam() = 0;

    /* Get current setpoint for the MPC */
    virtual setPoint *getSetPoint(const double s, const Eigen::VectorXd &q, const Eigen::VectorXd &p, const Eigen::VectorXd &F) = 0;
    
    /* Get current gradient (setpoint/dx) for the MPC */
    virtual setPoint *getSetPointGradient(const double s) = 0;
    
    /* Get weighting matrices of the MP */
    virtual weights *getWeights() = 0;

    /* Get weighting matrices of the MP, Qds is scaled at the beginning and end of the MP which causes slower robot motions */
    virtual weights *getScaledWeights(const double s, const double s_f) = 0;

    /* Get constraints */
    virtual constraints *getConstraints() = 0;

    /* Get MP name */
    virtual std::string getName() = 0;

    /* Get MP type */
    virtual MPtype *getType() = 0;

    /* Get initial progress of the reference path */
    virtual double getInitialProgressParam() = 0;

    /* Get terminal progress of the reference path */
    virtual double getTerminationProgressParam() = 0;

    /* Get reaction strategy */
    virtual ReactionStrategy getReactionStrategy() = 0;

    /* Get exception strategy */
    virtual ExceptionStrategy getExceptionStrategy() = 0;

    /* Print MP parameterization */
    virtual void print() = 0;

  private:
    /* Init MP from .yaml-file */
    virtual bool initializeFromFile(const std::string folder_path, const std::string mp_file_name) = 0;
  };
} // namespace lrt_mpic
#endif // MP_FRAMEWORK_H
