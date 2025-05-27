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

#include "grampc_util_cpp.h"

namespace lrt
{
    void grampc_setopt_int(grampc::Grampc &solver, const std::string &optName, const ros::NodeHandle &nh)
    {
        int optValue;
        if (nh.getParamCached(optName, optValue))
        {
            solver.setopt_int(optName.c_str(), (int)optValue);
        }
    }

    void grampc_setopt_real(grampc::Grampc &solver, const std::string &optName, const ros::NodeHandle &nh)
    {
        double optValue;
        if (nh.getParamCached(optName, optValue))
        {
            solver.setopt_real(optName.c_str(), optValue);
        }
    }

    void grampc_setopt_string(grampc::Grampc &solver, const std::string &optName, const ros::NodeHandle &nh)
    {
        std::string optValue;
        if (nh.getParamCached(optName, optValue))
        {
            solver.setopt_string(optName.c_str(), optValue.c_str());
        }
    }

    void grampc_setopt_vector(grampc::Grampc &solver, const std::string &optName, const ros::NodeHandle &nh, size_t size)
    {
        std::vector<double> optValue;
        if (nh.getParamCached(optName, optValue))
        {
            if (optValue.size() != size)
            {
                ROS_ERROR_STREAM("Option '" << optName << "' requires " << size << " elements.");
            }
            else
            {
                solver.setopt_real_vector(optName.c_str(), &optValue[0]);
            }
        }
    }

    void grampc_setparam_real(grampc::Grampc &solver, const std::string &paramName, const ros::NodeHandle &nh)
    {
        double paramValue;
        if (nh.getParamCached(paramName, paramValue))
        {
            solver.setparam_real(paramName.c_str(), paramValue);
        }
    }

    void grampc_setparam_vector(grampc::Grampc &solver, const std::string paramName, const ros::NodeHandle &nh, size_t size)
    {
        std::vector<double> paramValue;
        if (nh.getParamCached(paramName, paramValue))
        {
            if (paramValue.size() != size)
            {
                ROS_ERROR_STREAM("Parameter '" << paramName << "' requires " << size << " elements.");
            }
            else
            {
                solver.setparam_real_vector(paramName.c_str(), &paramValue[0]);
            }
        }
    }

    void grampc_setopt(grampc::Grampc &solver, const ros::NodeHandle &nh)
    {
        grampc_setopt_int(solver, "Nhor", nh);
        grampc_setopt_int(solver, "MaxGradIter", nh);
        grampc_setopt_int(solver, "MaxMultIter", nh);
        grampc_setopt_string(solver, "ShiftControl", nh);

        grampc_setopt_string(solver, "TimeDiscretization", nh);

        grampc_setopt_string(solver, "IntegralCost", nh);
        grampc_setopt_string(solver, "TerminalCost", nh);
        grampc_setopt_string(solver, "IntegratorCost", nh);

        grampc_setopt_string(solver, "Integrator", nh);
        grampc_setopt_real(solver, "IntegratorRelTol", nh);
        grampc_setopt_real(solver, "IntegratorAbsTol", nh);

        grampc_setopt_string(solver, "LineSearchType", nh);
        grampc_setopt_real(solver, "LineSearchMax", nh);
        grampc_setopt_real(solver, "LineSearchMin", nh);
        grampc_setopt_real(solver, "LineSearchInit", nh);
        grampc_setopt_real(solver, "LineSearchIntervalFactor", nh);
        grampc_setopt_real(solver, "LineSearchAdaptFactor", nh);
        grampc_setopt_real(solver, "LineSerachIntervalTol", nh);

        grampc_setopt_string(solver, "ControlOptim", nh);
        grampc_setopt_string(solver, "ParamOptim", nh);
        grampc_setopt_real(solver, "ParamOptimLineSearchFactor", nh);
        grampc_setopt_string(solver, "TimeOptim", nh);
        grampc_setopt_real(solver, "TimeOptimLineSearchFactor", nh);

        grampc_setopt_string(solver, "ScaleProblem", nh);
        grampc_setopt_vector(solver, "xScale", nh, solver.getParameters()->Nx);
        grampc_setopt_vector(solver, "xOffset", nh, solver.getParameters()->Nx);
        grampc_setopt_vector(solver, "uScale", nh, solver.getParameters()->Nu);
        grampc_setopt_vector(solver, "uOffset", nh, solver.getParameters()->Nu);
        grampc_setopt_vector(solver, "pScale", nh, solver.getParameters()->Np);
        grampc_setopt_vector(solver, "pOffset", nh, solver.getParameters()->Np);
        grampc_setopt_real(solver, "TScale", nh);
        grampc_setopt_real(solver, "TOffset", nh);

        grampc_setopt_string(solver, "EqualityConstraints", nh);
        grampc_setopt_string(solver, "InequalityConstraints", nh);
        grampc_setopt_string(solver, "TerminalEqualityConstraints", nh);
        grampc_setopt_string(solver, "TerminalInequalityConstraints", nh);
        grampc_setopt_string(solver, "ConstraintsHandling", nh);
        grampc_setopt_vector(solver, "lag0", nh, solver.getParameters()->Nc);
        grampc_setopt_vector(solver, "pen0", nh, solver.getParameters()->Nc);
        grampc_setopt_real(solver, "GradientRelTol", nh);
        grampc_setopt_vector(solver, "ConstraintsAbsTol", nh, solver.getParameters()->Nc);

        grampc_setopt_real(solver, "LagrangeMax", nh);
        grampc_setopt_real(solver, "LagrangeDampingFactor", nh);

        grampc_setopt_real(solver, "PenaltyMax", nh);
        grampc_setopt_real(solver, "PenaltyMin", nh);
        grampc_setopt_real(solver, "PenaltyIncreaseFactor", nh);
        grampc_setopt_real(solver, "PenaltyDecreaseFactor", nh);
        grampc_setopt_real(solver, "PenaltyIncreaseThreshold", nh);

        grampc_setopt_string(solver, "ConvergenceTest", nh);
        grampc_setopt_real(solver, "ConvergenceGradientRelTol", nh);
    }

    void grampc_setparam(grampc::Grampc &solver, const ros::NodeHandle &nh)
    {
        grampc_setparam_vector(solver, "xk", nh, solver.getParameters()->Nx);
        grampc_setparam_vector(solver, "x0", nh, solver.getParameters()->Nx);
        grampc_setparam_vector(solver, "xdes", nh, solver.getParameters()->Nx);

        grampc_setparam_vector(solver, "u0", nh, solver.getParameters()->Nu);
        grampc_setparam_vector(solver, "udes", nh, solver.getParameters()->Nu);
        grampc_setparam_vector(solver, "umin", nh, solver.getParameters()->Nu);
        grampc_setparam_vector(solver, "umax", nh, solver.getParameters()->Nu);

        grampc_setparam_vector(solver, "p0", nh, solver.getParameters()->Np);
        grampc_setparam_vector(solver, "pmin", nh, solver.getParameters()->Np);
        grampc_setparam_vector(solver, "pmax", nh, solver.getParameters()->Np);

        grampc_setparam_real(solver, "Thor", nh);
        grampc_setparam_real(solver, "Tmax", nh);
        grampc_setparam_real(solver, "Tmin", nh);

        grampc_setparam_real(solver, "dt", nh);
        grampc_setparam_real(solver, "t0", nh);
    }

    void grampc_printsolution(grampc::Grampc &solver, std::ostream &out)
    {
        for (int i = 0; i < solver.getOptions()->Nhor; ++i)
        {
            grampc_printsolution(solver, out, i);
        }
    }

    void grampc_printsolution(grampc::Grampc &solver, std::ostream &out, int step)
    {
        // time
        out << solver.getParameters()->t0 + solver.getWorkspace()->t[step] << "\t";
        // states
        for (int j = 0; j < solver.getParameters()->Nx; ++j)
        {
            out << solver.getWorkspace()->x[step * solver.getParameters()->Nx + j] << "\t";
        }
        // adjoint states
        for (int j = 0; j < solver.getParameters()->Nx; ++j)
        {
            out << solver.getWorkspace()->adj[step * solver.getParameters()->Nx + j] << "\t";
        }
        // controls
        for (int j = 0; j < solver.getParameters()->Nu; ++j)
        {
            out << solver.getWorkspace()->u[step * solver.getParameters()->Nu + j] << "\t";
        }
        out << std::endl;
    }

    bool grampc_converged(grampc::Grampc &solver, bool verbose)
    {
        int Nhor = solver.getOptions()->Nhor;
        int Nu = solver.getParameters()->Nu;
        int Np = solver.getParameters()->Np;
        int Nc = solver.getParameters()->Nc;
        const double *u = solver.getWorkspace()->u;
        const double *uprev = solver.getWorkspace()->uprev;
        const double *p = solver.getWorkspace()->p;
        const double *pprev = solver.getWorkspace()->pprev;
        const double *T = &solver.getWorkspace()->T;
        const double *Tprev = &solver.getWorkspace()->Tprev;
        const double *cfct = solver.getWorkspace()->cfct;

        // controls converged?
        double threshold = solver.getOptions()->ConvergenceGradientRelTol;
        for (int i = 0; i < Nhor; ++i)
        {
            for (int j = 0; j < Nu; ++j)
            {
                if (std::fabs(u[i * Nu + j] - uprev[i * Nu + j]) > threshold)
                {
                    if (verbose)
                    {
                        ROS_INFO_STREAM("Control " << (j + 1) << " not converged in step " << (i + 1));
                    }
                    return false;
                }
            }
        }
        // parameters converged?
        for (int j = 0; j < Np; ++j)
        {
            if (std::fabs(p[j] - pprev[j]) > threshold)
            {
                if (verbose)
                {
                    ROS_INFO_STREAM("Parameter " << (j + 1) << " not converged");
                }
                return false;
            }
        }
        // time converged?
        if (std::fabs(T[0] - Tprev[0]) > threshold)
        {
            if (verbose)
            {
                ROS_INFO_STREAM("Time not converged");
            }
            return false;
        }
        return true;
    }

}
