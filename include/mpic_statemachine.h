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

#ifndef MPIC_STATEMACHINE_H
#define MPIC_STATEMACHINE_H

#include "mpic_state.h"
#include "mpic_finflag.h"

#include "mpic_exception.h"

namespace lrt_mpic
{

    /*
     * Characteristics of the current MP
    */
    typedef struct
    {
        FinishFlag finished_;
        bool tooFarFromNextMP_;
        bool hasSuccessor_;
        ReactionStrategy reaction_;
        ExceptionStrategy exception_;
    } TaskMeasurement;

    /**
     * This class implements the state machine for task control
     */
    class TaskStatemachine
    {
    public:
        TaskStatemachine();
        ~TaskStatemachine();
        /* Get current state */
        TaskState getCurrentState();
        /* Get last state */
        TaskState getLastState();
        /* Start task execution */
        void startTask();
        /* Evaluate termination conditions */
        TaskCommand evaluateTask(const TaskMeasurement &meas);

        struct
        {
            bool restart_;
        } opt_{};

    private:
        /* Evaluate the progress of the error recovery */
        TaskCommand evaluateTaskException(const TaskMeasurement &meas);
        /* Terminate task execution */
        TaskCommand endTask();
        /* Current state of the state machine */
        TaskState currentState_;
        /* Last state of the state machine */
        TaskState lastState_;
    };

} // namespace lrt_mpic
#endif // MPIC_STATEMACHINE_H