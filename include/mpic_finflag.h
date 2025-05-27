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

#ifndef MPIC_FINFLAG_H
#define MPIC_FINFLAG_H

#include <string>

namespace lrt_mpic
{
    /**
     * Flags indicating the current state of an MP
     */
    enum class FinishFlag
    {
        runningMP,
        finishedMP,
        failedMP
    };

    /* Conversion functions */
    std::string finishFlagToString(FinishFlag flag);
    FinishFlag stringToFinishFlag(std::string text);

} // namespace lrt_mpic
#endif // MPIC_FINFLAG_H