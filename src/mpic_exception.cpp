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

#include "mpic_exception.h"

#include <map>

namespace lrt_mpic{

static std::map<std::string, ReactionStrategy> str2react{
    {"none", ReactionStrategy::none},
    {"backward", ReactionStrategy::backward},
};
static std::map<ReactionStrategy, std::string> react2string{
    {ReactionStrategy::none, "none"},
    {ReactionStrategy::backward, "backward"},
};
static std::map<std::string, ExceptionStrategy> str2except{
    {"none", ExceptionStrategy::none},
    {"restartMP", ExceptionStrategy::restartMP},
    {"restartSkill", ExceptionStrategy::restartSkill},
    {"restartTask", ExceptionStrategy::restartTask},
    {"spiralMotion", ExceptionStrategy::spiralMotion},
    {"lissajousMotion", ExceptionStrategy::lissajousMotion},
};
static std::map<ExceptionStrategy, std::string> except2str{
    {ExceptionStrategy::none, "none"},
    {ExceptionStrategy::restartMP, "restartMP"},
    {ExceptionStrategy::restartSkill, "restartSkill"},
    {ExceptionStrategy::restartTask, "restartTask"},
    {ExceptionStrategy::spiralMotion, "spiralMotion"},
    {ExceptionStrategy::lissajousMotion, "lissajousMotion"},
};


std::string reactionStrategyToString (const ReactionStrategy& strategy){
    return react2string[strategy];
}

ReactionStrategy stringToReactionStrategy (const std::string& strategy){
    return str2react[strategy];
}

std::string exceptionStrategyToString (const ExceptionStrategy& strategy){
    return except2str[strategy];
}

ExceptionStrategy stringToExceptionStrategy (const std::string& strategy){
    return str2except[strategy];
}
}// namespace lrt_mpic