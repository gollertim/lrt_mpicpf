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

#ifndef MP_TYPES_H
#define MP_TYPES_H

#include <string>

namespace lrt_mpic
{
  /*
    Types for Manipulation Primitives
  */
  enum class MPtype
  {
    none,
    generic_mpic,
    move_to_contact
  };

  /* Convert type to string*/
  std::string mpTypeToString(const MPtype &mp);
  
  /* Convert String to type*/
  MPtype stringToMPtype(const std::string &mp);
}
#endif // MP_TYPES_H