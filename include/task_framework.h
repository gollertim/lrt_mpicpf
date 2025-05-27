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

#ifndef TASK_FRAMEWORK_H
#define TASK_FRAMEWORK_H

#include <Eigen/Dense>
#include <mutex>
#include <vector>
#include <iostream>
#include <fstream>

#include <skill_framework.h>

#include "../../libs/yaml-cpp/include/yaml-cpp/yaml.h"

namespace lrt_mpic {
  class Task{
  public:
    // Constructor to initialize the Task with degrees of freedom
    Task(unsigned Ndof, unsigned CartDof);

    // Reads task parameters from a YAML file
    void readTaskParam(const std::string folder_path, const std::string task_name);

    // Initializes the task with the given parameters
    void initTask(const double s_0, const Eigen::VectorXd &q, const Eigen::VectorXd &p, const Eigen::VectorXd &F);

    // Evaluates the task at a given progress parameter
    void evalTask(const double s, const Eigen::VectorXd &q, const Eigen::VectorXd &p, const Eigen::VectorXd &F);

    // Resets the task to its initial state
    void resetTask();

    // Advances to the next motion primitive (MP) in the task
    void advanceToNextMP();

    // Checks if there are more motion primitives to execute
    bool hasNextMP();

    // Returns a reference to the current active skill number
    int& getActSkillNo();

    // Returns a reference to the total number of skills
    int& getNSkills();

    // Returns the desired progress parameter
    const double getDesiredProgressParam();

    // Returns a shared pointer to the currently active skill
    std::shared_ptr<Skill> getActSkill();

    // Returns a shared pointer to a specific skill by index
    std::shared_ptr<Skill> getSkill(const unsigned i);

    // Returns a pointer to the task finished status
    bool* isFinished();

    // Returns a reference to the task started status
    bool& isStarted();

  private:
  // Helper function to print a YAML node (for debugging purposes)
    void printYamlNode(YAML::Node &node);

    unsigned Ndof_, CartDof_; // Degrees of freedom
    int NSkills_; // Number of skills in the task
    int iSkill_; // Index of the currently active skill
    bool taskFinished_; // Flag indicating if the task is finished
    bool taskStarted_; // Flag indicating if the task has started

    YAML::Node task_node_; // YAML node containing task parameters

    std::vector<std::shared_ptr<Skill>> task_; // List of skills in the task
  };

}  // namespace lrt_mpic
#endif // SKILL_FRAMEWORK_H
