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
#include "mpic_finflag.h"
#include "mpic_statemachine.h"
#include <iostream>

namespace lrt_mpic{

TaskStatemachine::TaskStatemachine(){
    currentState_ = TaskState::init;
    lastState_ = TaskState::init;
}

TaskState TaskStatemachine::getCurrentState(){
    return currentState_;
}

TaskState TaskStatemachine::getLastState(){
    return lastState_;
}

void TaskStatemachine::startTask(){
    lastState_ = currentState_;
    currentState_ = TaskState::start;
}

TaskCommand TaskStatemachine::evaluateTask(const TaskMeasurement& meas){
    lastState_ = currentState_;
    switch(currentState_){
    case TaskState::init :
        currentState_ = TaskState::stop;
        return TaskCommand::stopExecution;

    case TaskState::start :
        if(meas.tooFarFromNextMP_){
            currentState_ = TaskState::exception;
            return TaskCommand::moveToNextMP;
        }else{
            currentState_ = TaskState::runTask;
            return TaskCommand::advanceToNextMP;
        }

    case TaskState::stop : 
        currentState_ = TaskState::stop;
        return TaskCommand::stopExecution;

    case TaskState::runTask :
        switch(meas.finished_){
        case FinishFlag::runningMP :
            currentState_ = TaskState::runTask;
            return TaskCommand::proceed;

        case FinishFlag::finishedMP :
            if(meas.hasSuccessor_){
                currentState_ = TaskState::runTask;
                return TaskCommand::advanceToNextMP;
            }else{
                return endTask();
            }

        case FinishFlag::failedMP :
            if(meas.reaction_ != ReactionStrategy::none){
                currentState_ = TaskState::reaction;
                return TaskCommand::beginReaction;
            }
            return evaluateTaskException(meas);

        default:
            break;
        }
        break;

    case TaskState::reaction :
        switch(meas.finished_){
        case FinishFlag::runningMP :
            currentState_ = TaskState::reaction;
            return TaskCommand::proceed;

        case FinishFlag::finishedMP :
            return evaluateTaskException(meas);

        case FinishFlag::failedMP :
            currentState_ = TaskState::reaction;
            return TaskCommand::proceed;

        default:
            break;
        }
        break;

    case TaskState::exception :
        switch(meas.finished_){
        case FinishFlag::runningMP :
            currentState_ = TaskState::exception;
            return TaskCommand::proceed;

        case FinishFlag::finishedMP :
            if(meas.hasSuccessor_){
                currentState_ = TaskState::runTask;
                return TaskCommand::proceed;
            }else{
                return endTask();
            }

        case FinishFlag::failedMP :
            currentState_ = TaskState::start;
            return TaskCommand::restartSkillMoveTo;

        default:
            break;
        }
        break;

    default:
        break;
    }

}

TaskCommand TaskStatemachine::evaluateTaskException(const TaskMeasurement& meas){
    switch(meas.exception_){
    case ExceptionStrategy::restartTask :
        currentState_ = TaskState::start;
        return TaskCommand::prepareStartConditon;

    case ExceptionStrategy::restartSkill :
        if(meas.tooFarFromNextMP_){
            currentState_ = TaskState::exception;
            return TaskCommand::restartSkillMoveTo;
        }else{
            currentState_ = TaskState::runTask;
            return TaskCommand::restartSkill;
        }

    case ExceptionStrategy::restartMP :
        currentState_ = TaskState::runTask;
        return TaskCommand::restartMP;

    case ExceptionStrategy::spiralMotion :
        currentState_ = TaskState::exception;
        return TaskCommand::beginException;

    case ExceptionStrategy::lissajousMotion :
        currentState_ = TaskState::exception;
        return TaskCommand::beginException;

    default:
        break;
    }
}

TaskCommand TaskStatemachine::endTask(){
    if(opt_.restart_){
        currentState_ = TaskState::start;
        return TaskCommand::prepareStartConditon;
    }else{
        currentState_ = TaskState::stop;
        return TaskCommand::stopExecution;
    }
}

} //namespace lrt_mpic