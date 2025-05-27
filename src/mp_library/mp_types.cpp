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

#include "mp_library/mp_types.h"

#include <map>

namespace lrt_mpic
{

    static std::map<std::string, MPtype> str2mp{
        {"none", MPtype::none},
        {"generic_mpic", MPtype::generic_mpic},
        {"move_to_contact", MPtype::move_to_contact}};
    static std::map<MPtype, std::string> mp2str{
        {MPtype::none, "none"},
        {MPtype::generic_mpic, "generic_mpic"},
        {MPtype::move_to_contact, "move_to_contact"}};

    std::string mpTypeToString(const MPtype &mp)
    {
        return mp2str[mp];
    }

    MPtype stringToMPtype(const std::string &mp)
    {
        try
        {
            return str2mp.at(mp);
        }
        catch (...)
        {
            throw std::out_of_range("[stringToMPType]: unknown MP Type");
        }
    }
}