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

#include "skill_framework.h"

namespace lrt_mpic
{

  Skill::Skill(unsigned Ndof, unsigned CartDof)
  {
    Ndof_ = Ndof;
    CartDof_ = CartDof;

    NMP_ = 0;
    iMP_ = 0;

    skillFinished_ = false;
  }
  void Skill::readSkillParam(const std::string folder_path, const std::string skill_name)
  {
    // Load skill YAML file
    const std::string loc_folder_path = folder_path + skill_name + "/";
    skill_node_ = YAML::LoadFile(loc_folder_path + skill_name + ".yaml");

    NMP_ = (skill_node_["skill/param"]["NMP"].as<std::vector<double>>()).at(0);
    std::cout << "[Skill::readSkillParam]: The skill consists of " << NMP_ << " MPs." << std::endl;
    std::cout << "[Skill::readSkillParam]: The skill is build up by following MPs:" << std::endl;

    // for-loop -> MP initialization
    for (unsigned iMP = 0; iMP < NMP_; ++iMP)
    {
      std::string mp_file_string = skill_node_["skill/mp_" + std::to_string(iMP + 1)]["file_name"].as<std::string>();
      // get MP type
      std::string mp_type_str = YAML::LoadFile(loc_folder_path + "mp/" + mp_file_string)["mp/param"]["type"].as<std::string>();
      MPtype mp_type = stringToMPtype(mp_type_str);

      if (mp_type == MPtype::none)
      {
        std::cout << "[Skill::readSkillParam]: type " << mp_type_str << " of MP " << iMP + 1 << "is unknown!" << std::endl;
      }

      switch (mp_type)
      {
      case MPtype::generic_mpic:
        skill_.push_back(std::make_shared<GenericMPIC>(loc_folder_path + "mp/", mp_file_string));
        break;

      case MPtype::move_to_contact:
        skill_.push_back(std::make_shared<MtcMPIC>(loc_folder_path + "mp/", mp_file_string));
        break;

      default:
        return;
      }

      // if(!skill_.at(iMP)->initialize(loc_folder_path + "mp/", mp_file_string)){
      //   std::cout << "[Skill::readSkillParam]: Initialization of MP: " << iMP+1 << "failed!" << std::endl;
      // }else{
      //   std::cout << "[Skill::readSkillParam]: MP initialized successfully!" << std::endl;
      // }
    } // for-loop
  }

  void Skill::initSkill(const double s_0, const Eigen::VectorXd &q, const Eigen::VectorXd &p, const Eigen::VectorXd &F)
  {
    skillFinished_ = false;
    skillStarted_ = false;
    iMP_ = 0;
  }

  void Skill::evalSkill(const double s, const Eigen::VectorXd &q, const Eigen::VectorXd &p, const Eigen::VectorXd &F)
  {
    if (skill_.at(iMP_)->evaluateMP(s, q, p, F) == FinishFlag::finishedMP && !skillFinished_)
    {
      if (iMP_ >= NMP_ - 1)
      {
        skillFinished_ = true;
      }
      else
      {
        iMP_ = iMP_ + 1;
      }
    }
    else if (skill_.at(iMP_)->getInitialProgressParam() > s)
    {
      if (iMP_ > 0)
      {
        iMP_ -= 1;
      }
      else
      {
        iMP_ = 0;
      }
      skillFinished_ = false;
    }
  }

  void Skill::resetSkill()
  {
    skillFinished_ = false;
    skillStarted_ = false;
    iMP_ = 0;
  }

  void Skill::advanceToNextMP()
  {
    if (!skillStarted_)
    {
      skillStarted_ = true;
      iMP_ = 0;
    }
    else
    {
      iMP_ += 1;
    }
    if (iMP_ >= NMP_)
    {
      skillFinished_ = true;
    }
  }

  bool Skill::hasNextMP()
  {
    return iMP_ < NMP_ - 1;
  }

  MPtype *Skill::getActMPType()
  {
    return skill_.at(iMP_)->getType();
  }

  GenericPath *Skill::getActPathParam()
  {
    return skill_.at(iMP_)->getPathParam();
  }

  int Skill::getNMP() { return NMP_; }

  setPoint *Skill::getActSetPoint(const double s, const Eigen::VectorXd &q, const Eigen::VectorXd &p, const Eigen::VectorXd &F)
  {
    return skill_.at(iMP_)->getSetPoint(s, q, p, F);
  }

  setPoint *Skill::getActSetPointGradient(const double s)
  {
    return skill_.at(iMP_)->getSetPointGradient(s);
  }

  weights *Skill::getActWeights()
  {
    return skill_.at(iMP_)->getWeights();
  }

  // weights* Skill::getActScaledWeights(const double s){
  //   return skill_.at(iMP_)->getScaledWeights(s);
  // }

  constraints *Skill::getActConstraints()
  {
    return skill_.at(iMP_)->getConstraints();
  }

  int &Skill::getActMPNo()
  {
    return iMP_;
  }

  std::shared_ptr<ManipulationPrimitive> Skill::getActMP()
  {
    return skill_.at(iMP_);
  }

  std::shared_ptr<ManipulationPrimitive> Skill::getMP(const unsigned i)
  {
    return skill_.at(i);
  }

  // for current MP
  double Skill::getInitialProgressParam()
  {
    return skill_.at(iMP_)->getInitialProgressParam();
  }

  // for current MP
  double Skill::getTerminationProgressParam()
  {
    return skill_.at(iMP_)->getTerminationProgressParam();
  }

  // for Skill
  double Skill::getSkillInitialProgressParam()
  {
    return skill_.at(0)->getInitialProgressParam();
  }

  // for Skill
  double Skill::getSkillTerminationProgressParam()
  {
    return skill_.at(NMP_ - 1)->getTerminationProgressParam();
  }

  // bool Skill::evalTermination(){
  //   return true;
  // }

  bool *Skill::isFinished()
  {
    return &skillFinished_;
  }

  bool &Skill::isStarted()
  {
    return skillStarted_;
  }

}