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

#ifndef HORIZON_CACHE_HPP
#define HORIZON_CACHE_HPP

#include <cmath>
#include <vector>

// #include "types.hpp"

namespace lrt
{

    /**
     * This class allows to store arbitrary data for each sampling point along the horizon
     * The current implementation assumes a uniform discretization with fixed horizon
     */
    template <typename T>
    class HorizonCache
    {
    public:
        // constructor for uniform discretization of interval [0, Thor] with Nhor points
        HorizonCache(unsigned Nhor, double Thor, const T &init = T())
            : delta_((Nhor - 1) / Thor),
              cache_(Nhor, init)
        {
        }

        // read-write access to the cached element at time T
        T &get(double t)
        {
            int idx = std::round(t * delta_);
            return cache_[idx];
        }

        // read-only access to the cached element at time T
        const T &get(double t) const
        {
            int idx = std::round(t * delta_);
            return cache_[idx];
        }

    private:
        double delta_;
        std::vector<T> cache_;
    };

}

#endif // HORIZON_CACHE_HPP
