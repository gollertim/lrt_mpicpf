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

#ifndef MPIC_EXCEPTION_H
#define MPIC_EXCEPTION_H

#include <string>

namespace lrt_mpic
{
    /**
     * Types of recovery strategies
    */
    enum class ReactionStrategy
    {
        none,
        backward,
    };

    /**
     * Types of exception strategies
    */
    enum class ExceptionStrategy
    {
        none,
        restartMP,
        restartSkill,
        restartTask,
        spiralMotion,
        lissajousMotion,
    };

    /* Conversion functions */
    std::string reactionStrategyToString(const ReactionStrategy &strategy);
    ReactionStrategy stringToReactionStrategy(const std::string &strategy);
    std::string exceptionStrategyToString(const ExceptionStrategy &strategy);
    ExceptionStrategy stringToExceptionStrategy(const std::string &strategy);

} // namespace lrt_mpic
#endif // MPIC_EXCEPTION_H