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

#ifndef MPIC_TRANSITION_H
#define MPIC_TRANSITION_H

#include <Eigen/Dense>

namespace lrt_mpic
{

  /* This struct implements fault conditions */
  struct Condition
  {
    Eigen::VectorXi selection;
    Eigen::VectorXd threshold;

    Condition() {}
    Condition(const unsigned NDof)
        : selection(Eigen::VectorXi::Zero(NDof)),
          threshold(Eigen::VectorXd::Zero(NDof))
    {
    }
  };

  /* This struct implements success conditions regarding the current pose and wrench */
  struct TerminationCondition
  {
    Condition pose_cond;
    Condition wrench_cond;

    TerminationCondition() {}
    TerminationCondition(const unsigned poseDof, const unsigned wrenchDof)
        : pose_cond(poseDof),
          wrench_cond(wrenchDof)
    {
    }
  };

} // namespace lrt_mpic
#endif // MPIC_TRANSITION_H