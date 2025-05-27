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

#ifndef MPIC_STATE_H
#define MPIC_STATE_H

#include <string>

namespace lrt_mpic
{

    /*
     * @brief state of current task progression
     */
    enum class TaskState
    {
        init,      // log: state = 0
        start,     //      state = 1
        runTask,   //      state = 2
        stop,      //      state = 3
        reaction,  //      state = 4
        exception, //      state = 5
    };

    /*
     * @brief command that governs task progression
     */
    enum class TaskCommand
    {
        proceed,
        advanceToNextMP,
        beginReaction,
        beginException,
        moveToNextMP,
        prepareStartConditon,
        stopExecution,
        restartMP,
        restartSkill,
        restartSkillMoveTo,
    };

    /* Conversion functions */
    std::string taskStateToString(TaskState state);
    std::string taskCommandToString(TaskCommand command);

} // namespace lrt_mpic
#endif // MPIC_STATE_H