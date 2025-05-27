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

#ifndef SKILL_FRAMEWORK_H
#define SKILL_FRAMEWORK_H

#include <Eigen/Dense>
#include <mutex>
#include <vector>
#include <iostream>
#include <fstream>

#include "../../libs/yaml-cpp/include/yaml-cpp/yaml.h"

#include <mpic_utils.h>
#include <mp_framework.h>
#include "mp_library/mp_types.h"
#include "mp_library/generic_mpic.hpp"
#include "mp_library/mtc_mpic.hpp"

namespace lrt_mpic
{
  /**
   * @class Skill
   *  Represents a skill composed of multiple manipulation primitives (MPs).
   */

  /**
   *  Constructor for the Skill class.
   */
  class Skill
  {
  public:
    Skill(unsigned Ndof, unsigned CartDof);

    /**
     *  Reads skill parameters from a specified folder and skill name.
     */
    void readSkillParam(const std::string folder_path, const std::string skill_name);

    /**
     *  Initializes the skill with the given parameters.
     */
    void initSkill(const double s_0, const Eigen::VectorXd &q, const Eigen::VectorXd &p, const Eigen::VectorXd &F);

    /**
     *  Evaluates the skill at a given progress parameter.
     */
    void evalSkill(const double s, const Eigen::VectorXd &q, const Eigen::VectorXd &p, const Eigen::VectorXd &F);

    /**
     *  Resets the skill to its initial state.
     */
    void resetSkill();

    /**
     *  Advances to the next MP in the skill.
     */
    void advanceToNextMP();

    /**
     *  Checks if there is a next MP available.
     */
    bool hasNextMP();

    /**
     *  Gets the currently active MP.
     */
    std::shared_ptr<ManipulationPrimitive> getActMP();

    /**
     *  Gets a specific MP by index.
     */
    std::shared_ptr<ManipulationPrimitive> getMP(const unsigned i);

    /**
     *  Gets the index of the currently active MP.
     */
    int &getActMPNo();

    /**
     *  Gets the type of the currently active MP.
     */
    MPtype *getActMPType();

    /**
     *  Gets the path parameters of the currently active MP.
     */
    GenericPath *getActPathParam();

    /**
     *  Gets the weights of the currently active MP.
     */
    weights *getActWeights();

    /**
     *  Gets the constraints of the currently active MP.
     */
    constraints *getActConstraints();

    /**
     *  Gets the set point of the currently active MP at a given progress parameter.
     */
    setPoint *getActSetPoint(const double s, const Eigen::VectorXd &q, const Eigen::VectorXd &p, const Eigen::VectorXd &F);

    /**
     *  Gets the gradient of the set point of the currently active MP at a given progress parameter.
     */
    setPoint *getActSetPointGradient(const double s);

    /**
     *  Gets the termination progress parameter of the currently active MP.
     */
    double getActTerminationProgressParam();

    /**
     *  Gets the total number of manipulation primitives (MPs) in the skill.
     */
    int getNMP();

    /**
     *  Gets the initial progress parameter of the skill.
     */
    double getInitialProgressParam();

    /**
     *  Gets the termination progress parameter of the skill.
     */
    double getTerminationProgressParam();

    /**
     *  Gets the initial progress parameter of the entire skill.
     */
    double getSkillInitialProgressParam();

    /**
     *  Gets the termination progress parameter of the entire skill.
     */
    double getSkillTerminationProgressParam();

    /**
     *  Checks if the skill is finished.
     */
    bool *isFinished();

    /**
     *  Checks if the skill has started.
     */
    bool &isStarted();

  private:
    unsigned Ndof_, CartDof_;
    int NMP_; // amount of MPs
    int iMP_; // actual MP
    bool skillFinished_;
    bool skillStarted_;

    YAML::Node skill_node_;

    std::vector<std::shared_ptr<ManipulationPrimitive>> skill_;
  };

} // namespace lrt_mpic
#endif // SKILL_FRAMEWORK_H
