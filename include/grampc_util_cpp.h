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

#ifndef GRAMPC_UTIL_CPP_H
#define GRAMPC_UTIL_CPP_H

#include <grampc.hpp>
#include <ros/ros.h>
#include <Eigen/Dense>

namespace lrt
{
    /**
     * @brief Set an integer option for the GRAMPC solver from a ROS parameter.
     * @param solver Reference to the GRAMPC solver.
     * @param optName Name of the option to set.
     * @param nh ROS NodeHandle for parameter retrieval.
     */
    void grampc_setopt_int(grampc::Grampc &solver, const std::string &optName, const ros::NodeHandle &nh);

    /**
     * @brief Set a real-valued option for the GRAMPC solver from a ROS parameter.
     * @param solver Reference to the GRAMPC solver.
     * @param optName Name of the option to set.
     * @param nh ROS NodeHandle for parameter retrieval.
     */
    void grampc_setopt_real(grampc::Grampc &solver, const std::string &optName, const ros::NodeHandle &nh);

    /**
     * @brief Set a string option for the GRAMPC solver from a ROS parameter.
     * @param solver Reference to the GRAMPC solver.
     * @param optName Name of the option to set.
     * @param nh ROS NodeHandle for parameter retrieval.
     */
    void grampc_setopt_string(grampc::Grampc &solver, const std::string &optName, const ros::NodeHandle &nh);

    /**
     * @brief Set a vector option for the GRAMPC solver from a ROS parameter.
     * @param solver Reference to the GRAMPC solver.
     * @param optName Name of the option to set.
     * @param nh ROS NodeHandle for parameter retrieval.
     * @param size Expected size of the vector.
     */
    void grampc_setopt_vector(grampc::Grampc &solver, const std::string &optName, const ros::NodeHandle &nh, size_t size);

    /**
     * @brief Set an integer parameter for the GRAMPC solver from a ROS parameter.
     * @param solver Reference to the GRAMPC solver.
     * @param paramName Name of the parameter to set.
     * @param nh ROS NodeHandle for parameter retrieval.
     */
    void grampc_setparam_int(grampc::Grampc &solver, const std::string &paramName, const ros::NodeHandle &nh);

    /**
     * @brief Set a real-valued parameter for the GRAMPC solver from a ROS parameter.
     * @param solver Reference to the GRAMPC solver.
     * @param paramName Name of the parameter to set.
     * @param nh ROS NodeHandle for parameter retrieval.
     */
    void grampc_setparam_real(grampc::Grampc &solver, const std::string &paramName, const ros::NodeHandle &nh);

    /**
     * @brief Set a vector parameter for the GRAMPC solver from a ROS parameter.
     * @param solver Reference to the GRAMPC solver.
     * @param paramName Name of the parameter to set.
     * @param nh ROS NodeHandle for parameter retrieval.
     * @param size Expected size of the vector.
     */
    void grampc_setparam_vector(grampc::Grampc &solver, const std::string paramName, const ros::NodeHandle &nh, size_t size);

    /**
     * @brief Set multiple options for the GRAMPC solver from ROS parameters.
     * @param solver Reference to the GRAMPC solver.
     * @param nh ROS NodeHandle for parameter retrieval.
     */
    void grampc_setopt(grampc::Grampc &solver, const ros::NodeHandle &nh);

    /**
     * @brief Set multiple parameters for the GRAMPC solver from ROS parameters.
     * @param solver Reference to the GRAMPC solver.
     * @param nh ROS NodeHandle for parameter retrieval.
     */
    void grampc_setparam(grampc::Grampc &solver, const ros::NodeHandle &nh);

    /**
     * @brief Print the solution of the GRAMPC solver to an output stream.
     * @param solver Reference to the GRAMPC solver.
     * @param out Output stream to print the solution to.
     */
    void grampc_printsolution(grampc::Grampc &solver, std::ostream &out);

    /**
     * @brief Print the solution of the GRAMPC solver for a specific step to an output stream.
     * @param solver Reference to the GRAMPC solver.
     * @param out Output stream to print the solution to.
     * @param step Step index of the solution to print.
     */
    void grampc_printsolution(grampc::Grampc &solver, std::ostream &out, int step);

    /**
     * @brief Check if the GRAMPC solver has converged.
     * @param solver Reference to the GRAMPC solver.
     * @param verbose If true, print additional information about convergence.
     * @return True if the solver has converged, false otherwise.
     */
    bool grampc_converged(grampc::Grampc &solver, bool verbose = false);
}

#endif // GRAMPC_UTIL_CPP_H
