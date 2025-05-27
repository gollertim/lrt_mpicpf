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

#include "mpic_state.h"

#include <map>

namespace lrt_mpic
{

    static std::map<TaskState, std::string> state2str{
        {TaskState::init, "init"},
        {TaskState::start, "start"},
        {TaskState::runTask, "runTask"},
        {TaskState::stop, "stop"},
        {TaskState::reaction, "reaction"},
        {TaskState::exception, "exception"},
    };

    static std::map<TaskCommand, std::string> command2str{
        {TaskCommand::proceed, "proceed"},
        {TaskCommand::advanceToNextMP, "advanceToNextMP"},
        {TaskCommand::beginReaction, "beginReaction"},
        {TaskCommand::beginException, "beginException"},
        {TaskCommand::moveToNextMP, "moveToNextMP"},
        {TaskCommand::prepareStartConditon, "prepareStartConditon"},
        {TaskCommand::stopExecution, "stopExecution"},
        {TaskCommand::restartSkill, "restartSkill"},
        {TaskCommand::restartSkillMoveTo, "restartSkillMoveTo"},
        {TaskCommand::restartMP, "restartMP"},
    };

    std::string taskStateToString(TaskState state)
    {
        return state2str[state];
    }

    std::string taskCommandToString(TaskCommand command)
    {
        return command2str[command];
    }

} // namespace lrt_mpic