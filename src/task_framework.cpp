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

#include "task_framework.h"

#include <filesystem>

namespace lrt_mpic
{

  Task::Task(unsigned Ndof, unsigned CartDof)
  {
    Ndof_ = Ndof;
    CartDof_ = CartDof;

    NSkills_ = 0;
    iSkill_ = 0;

    taskFinished_ = false;
  }
  void Task::readTaskParam(const std::string folder_path, const std::string task_name)
  {
    // Load task.yaml file
    const std::string loc_folder_path = folder_path + task_name + "/";
    task_node_ = YAML::LoadFile(loc_folder_path + task_name + ".yaml");

    NSkills_ = (task_node_["task/param"]["NSkills"].as<std::vector<double>>()).at(0);
    std::cout << "[Task::readTaskParam]: The task consists of " << NSkills_ << " Skills." << std::endl;
    std::cout << "[Task::readTaskParam]: The task is build up by following Skills:" << std::endl;

    // for-loop -> Skill initialization
    for (unsigned iSkill_ = 0; iSkill_ < NSkills_; ++iSkill_)
    {
      std::string skill_folder_string = task_node_["task/skill_" + std::to_string(iSkill_ + 1)]["folder_name"].as<std::string>();
      // Create skill sequence
      task_.push_back(std::make_shared<Skill>(Ndof_, CartDof_));
      task_.at(iSkill_)->readSkillParam(loc_folder_path, skill_folder_string);
    } // for-loop
  }

  void Task::initTask(const double s_0, const Eigen::VectorXd &q, const Eigen::VectorXd &p, const Eigen::VectorXd &F)
  {
    taskFinished_ = false;
    taskStarted_ = false;
    iSkill_ = 0;
    task_.at(iSkill_)->initSkill(s_0, q, p, F);
  }

  void Task::evalTask(const double s, const Eigen::VectorXd &q, const Eigen::VectorXd &p, const Eigen::VectorXd &F)
  {
    task_.at(iSkill_)->evalSkill(s, q, p, F);

    if (*task_.at(iSkill_)->isFinished() && !taskFinished_)
    {
      if (iSkill_ >= NSkills_ - 1)
      {
        taskFinished_ = true;
        std::cout << "[Task::evalTask]: Task is processed. ; NSkills_ = " << NSkills_ << " ; iSkill_ = " << iSkill_ << std::endl;
      }
      else
      {
        iSkill_ = iSkill_ + 1;
        task_.at(iSkill_)->initSkill(s, q, p, F);
      }
      // }else if(task_.at(iSkill_)->getInitialProgressParam() > s){
      //    iSkill_ -= 1;
      //    taskFinished_ = false;
    }
  }

  void Task::resetTask()
  {
    taskFinished_ = false;
    taskStarted_ = false;
    iSkill_ = 0;
    task_.at(iSkill_)->resetSkill();
  }

  void Task::advanceToNextMP()
  {
    if (!taskStarted_)
    {
      taskStarted_ = true;
    }
    task_.at(iSkill_)->advanceToNextMP();
    if (*task_.at(iSkill_)->isFinished())
    {
      iSkill_ += 1;
      if (iSkill_ >= NSkills_)
      {
        taskFinished_ = true;
      }
      else
      {
        task_.at(iSkill_)->resetSkill();
        task_.at(iSkill_)->advanceToNextMP();
      }
    }
  }

  bool Task::hasNextMP()
  { // a skill without MPs will terminate Task
    return (task_.at(iSkill_)->hasNextMP()) || ((iSkill_ < NSkills_ - 1) && (task_.at(iSkill_ + 1)->getNMP() > 0));
  }

  int &Task::getActSkillNo()
  {
    return iSkill_;
  }

  int &Task::getNSkills()
  {
    return NSkills_;
  }

  const double Task::getDesiredProgressParam()
  {
    return task_.at(iSkill_)->getSkillTerminationProgressParam();
  }

  std::shared_ptr<Skill> Task::getActSkill()
  {
    return task_.at(iSkill_);
  }

  std::shared_ptr<Skill> Task::getSkill(const unsigned i)
  {
    return task_.at(i);
  }

  bool *Task::isFinished()
  {
    return &taskFinished_;
  }

  bool &Task::isStarted()
  {
    return taskStarted_;
  }

  void Task::printYamlNode(YAML::Node &node)
  {
    YAML::Emitter emitter;
    emitter << node;
    std::cout << "--------------------------------------------" << std::endl;
    std::cout << emitter.c_str() << std::endl;
  }

}