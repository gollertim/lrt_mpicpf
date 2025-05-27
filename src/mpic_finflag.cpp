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

#include "mpic_finflag.h"

#include <map>

namespace lrt_mpic
{
    static std::map<FinishFlag, std::string> finflag2str{
        {FinishFlag::runningMP, "runningMP"},
        {FinishFlag::finishedMP, "finishedMP"},
        {FinishFlag::failedMP, "failedMP"},
    };

    static std::map<std::string, FinishFlag> str2finflag{
        {"runningMP", FinishFlag::runningMP},
        {"finishedMP", FinishFlag::finishedMP},
        {"failedMP", FinishFlag::failedMP},
    };

    std::string finishFlagToString(FinishFlag flag)
    {
        return finflag2str[flag];
    }

    FinishFlag stringToFinishFlag(std::string text)
    {
        return str2finflag[text];
    }
} // namespace lrt_mpic