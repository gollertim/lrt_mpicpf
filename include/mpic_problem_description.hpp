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

#ifndef MPIC_PROBLEM_DESCRIPTION_HPP
#define MPIC_PROBLEM_DESCRIPTION_HPP

#define PINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
#define PINOCCHIO_URDFDOM_USE_STD_SHARED_PTR

#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include "pinocchio/algorithm/joint-configuration.hpp"
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/frames-derivatives.hpp>

#include <memory>
#include <vector>
#include <ros/ros.h>
#include <urdf/model.h>

#include "grampc.hpp"
#include "mpic_utils.h"
#include "mp_framework.h"
#include "mp_library/mp_types.h"
#include "mp_library/generic_mpic.hpp"
#include "horizon_cache.hpp"

namespace grampc
{
    typedef std::shared_ptr<Grampc> GrampcPtr;
}

namespace lrt
{
    /**
     * Reference values for motion problem
     */
    struct MotionReference
    {
        lrt_mpic::weights wghts_;
        lrt_mpic::constraints constr_;
        lrt_mpic::setPoint setPnt_;
        lrt_mpic::setPoint setPntGradient_;

        MotionReference(const unsigned Ndof, const unsigned CartDof)
            : wghts_(Ndof, CartDof),
              constr_(Ndof, CartDof),
              setPnt_(Ndof, CartDof),
              setPntGradient_(Ndof, CartDof)
        {
        }
    };

    /**
     * Kinematic quantities that need to be computed for the motion problem
     */
    struct MotionData
    {
        Eigen::Vector3d ee_pos_;
        Eigen::Quaterniond ee_ori_;
        Eigen::Matrix3d ee_rot_mat_;
        Eigen::VectorXd ee_pose_;
        Eigen::MatrixXd ee_jac_;

        MotionData(unsigned Ndof)
            : ee_pos_(Eigen::Vector3d::Zero()),
              ee_ori_(Eigen::Quaterniond::Identity()),
              ee_rot_mat_(Eigen::MatrixXd::Identity(3, 3)),
              ee_pose_(Eigen::VectorXd::Zero(6)),
              ee_jac_(6, Ndof)
        {
        }
    };

    /** Problem description for GRAMPC:
     */
    class MPICProblemDescription : public grampc::ProblemDescription
    {
    public:
        MPICProblemDescription(unsigned Ndof, unsigned CartDof, unsigned Nstates, unsigned Nctr, unsigned Neqc, unsigned Nieqc, const Eigen::VectorXd q_0, const Eigen::VectorXd p_0, const std::vector<std::shared_ptr<lrt_mpic::ManipulationPrimitive>> MP_vec, unsigned Nhor, double Thor);
        /** OCP dimensions: states (Nx), controls (Nu), parameters (Np), equalities (Ng),
        inequalities (Nh), terminal equalities (NgT), terminal inequalities (NhT) **/
        void ocp_dim(typeInt *Nx, typeInt *Nu, typeInt *Np, typeInt *Neqc, typeInt *Nieqc, typeInt *Nteqc, typeInt *Ntieqc) override;
        
        /** System function f(t,x,u,p,userparam)
        ------------------------------------ **/
        void ffct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p) override;
        /** Jacobian df/dx multiplied by vector vec, i.e. (df/dx)^T*vec or vec^T*(df/dx) **/
        void dfdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *vec, ctypeRNum *u, ctypeRNum *p) override;
        /** Jacobian df/du multiplied by vector vec, i.e. (df/du)^T*vec or vec^T*(df/du) **/
        void dfdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *vec, ctypeRNum *u, ctypeRNum *p) override;
        /** Jacobian df/dp multiplied by vector vec, i.e. (df/dp)^T*vec or vec^T*(df/dp) **/
        void dfdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *vec, ctypeRNum *u, ctypeRNum *p) override;

        /** Integral cost l(t,x(t),u(t),p,xdes,udes,userparam)
        -------------------------------------------------- **/
        void lfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes) override;
        /** Gradient dl/dx **/
        void dldx(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes) override;
        /** Gradient dl/du **/
        void dldu(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes) override;
        /** Gradient dl/dp **/
        void dldp(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes) override;

        /** Terminal cost V(T,x,p) */
        void Vfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes) override;
        /** Gradient dV/dx **/
        void dVdx(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes) override;
        /** Gradient dV/dp **/
        void dVdp(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes) override;
        /** Gradient dV/dT **/
        void dVdT(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes) override;

        /** Equality constraints g(t,x,u,p) = 0 */
        void gfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p) override;
        /** Jacobian dg/dx multiplied by vector vec, i.e. (dg/dx)^T*vec or vec^T*(dg/dx) **/
        void dgdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec) override;
        /** Jacobian dg/du multiplied by vector vec, i.e. (dg/du)^T*vec or vec^T*(dg/du) **/
        void dgdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec) override;
        /** Jacobian dg/dp multiplied by vector vec, i.e. (dg/dp)^T*vec or vec^T*(dg/dp) **/
        void dgdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec) override;

        /** Inequality constraints h(t,x,u,p) < 0 */
        void hfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p) override;
        /** Jacobian dh/dx multiplied by vector vec, i.e. (dh/dx)^T*vec or vec^T*(dg/dx) **/
        void dhdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec) override;
        /** Jacobian dh/du multiplied by vector vec, i.e. (dh/du)^T*vec or vec^T*(dg/du) **/
        void dhdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec) override;
        /** Jacobian dh/dp multiplied by vector vec, i.e. (dh/dp)^T*vec or vec^T*(dg/dp) **/
        void dhdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec) override;

        /** Terminal equality constraints gT(T,x,p) = 0 */
        void gTfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p) override;
        /** Jacobian dgT/dx multiplied by vector vec, i.e. (dgT/dx)^T*vec or vec^T*(dgT/dx) **/
        void dgTdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec) override;
        /** Jacobian dgT/dp multiplied by vector vec, i.e. (dgT/dp)^T*vec or vec^T*(dgT/dp) **/
        void dgTdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec) override;
        /** Jacobian dgT/dT multiplied by vector vec, i.e. (dgT/dT)^T*vec or vec^T*(dgT/dT) **/
        void dgTdT_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec) override;

        /** Terminal inequality constraints hT(T,x,p) < 0 */
        void hTfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p) override;
        /** Jacobian dhT/dx multiplied by vector vec, i.e. (dhT/dx)^T*vec or vec^T*(dhT/dx) **/
        void dhTdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec) override;
        /** Jacobian dhT/dp multiplied by vector vec, i.e. (dhT/dp)^T*vec or vec^T*(dhT/dp) **/
        void dhTdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec) override;
        /** Jacobian dhT/dT multiplied by vector vec, i.e. (dhT/dT)^T*vec or vec^T*(dhT/dT) **/
        void dhTdT_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec) override;

        /** Additional functions required for the MPIC and task control
        ------------------------------------------------------- **/
        /** Set functions for onliine reconfiguration **/
        void setMPs(std::vector<std::shared_ptr<lrt_mpic::ManipulationPrimitive>> &MP_vec);
        /** Set desired valkue of the path progress parameter **/
        void setDesiredProgressParam(const double &s_f);
        /** Set combined stiffness of robot and environment **/
        void setKges(const Eigen::MatrixXd &Kges, const Eigen::MatrixXd Q_F);
        /** Set parameters of the low-level PD controller **/
        void setControllerParam(const lrt_mpic::controlGain &ctrGain);
        /** Set controller options **/
        void setControllerOpt(const lrt_mpic::controlOptions &ctrOpt);
        /** Get desired joint positions, Cartesian pose, interaction wrench for the current optimization step **/
        lrt_mpic::setPoint getActSetPoint();

    private:
        /** Weighted norm of desired "soll" and actual "ist" value **/
        void weightedNorm(double &cost, const Eigen::VectorXd &ist, const Eigen::VectorXd &soll, const Eigen::MatrixXd &weights, const Eigen::MatrixXd &normalization);
        void weightedNorm(double &cost, const Eigen::VectorXd &error, const Eigen::MatrixXd &weights, const Eigen::MatrixXd &normalization);
        
        /** Jacobian of weighted norm of desired "soll" and actual "ist" value **/
        void dWeightedNorm_dx(Eigen::VectorXd &dl_dx, const Eigen::VectorXd &ist, const Eigen::VectorXd &soll, const Eigen::MatrixXd &weights, const Eigen::MatrixXd &normalization);
        void dWeightedNorm_dx(Eigen::VectorXd &dl_dx, const Eigen::VectorXd &error, const Eigen::MatrixXd &weights, const Eigen::MatrixXd &normalization);

        /** Copy data from setPnt_in to setPnt_out **/
        void copySetPntData(lrt_mpic::setPoint &setPnt_out, const lrt_mpic::setPoint setPnt_in);

        /** Cache for forward kinematics computations **/
        HorizonCache<MotionData> horizon_cache_motion_data_;
        void motionDataFunction(MotionData &md, const pinocchio::Model &model, pinocchio::Data *data, const pinocchio::FrameIndex &flange_id, const Eigen::VectorXd &q);

        /** Cache for motion reference **/
        HorizonCache<MotionReference> horizon_cache_motion_reference_;
        void motionReferenceFunction(MotionReference &md, const double &s, const Eigen::VectorXd &q, const Eigen::VectorXd &p, const Eigen::VectorXd &F);

        int Ndof_;    // number of robot joints
        int CartDof_; // number of cartesian degrees of freedom (DOF)
        int Nx_;      // number of MPC states
        int Nu_;      // number of MPC inputs
        int Neqc_;    // number of MPC equality constraints
        int Nieqc_;   // number of MPC inequality constraints
        int Nhor_;    // number of MPC discretization steps
        double Thor_; // length of MPC prediction horizon in seconds

        ros::NodeHandle nh_;                                                   // ROS node handle
        unsigned iMP_;                                                         // Index of current MP
        std::vector<std::shared_ptr<lrt_mpic::ManipulationPrimitive>> MP_vec_; // MP vector
        lrt_mpic::weights *wghts_;                                             // MPC weights
        lrt_mpic::constraints *constr_ptr_;                                    // MPC constraints
        lrt_mpic::setPoint *setPnt_;                                           // contorl objective
        lrt_mpic::setPoint sharedSetPnt_;                                      // for data exchange with ROS-Controller
        lrt_mpic::setPoint *setPntGradient_;                                   // gradient of control objective
        lrt_mpic::controlGain ctrGains_;                                       // gains of PD controller
        lrt_mpic::controlOptions ctrOpt_;                                      // optional settings for MPC and task control
        Eigen::MatrixXd Kges_;                                                 // overall stiffness

        Eigen::VectorXd q_0_;                            // initial joint configuration
        Eigen::MatrixXd jac_, jac_transpose_, jac_pinv_; // Jacobian, its transposed and pseudo inverse
        Eigen::VectorXd pose_error_;                     // Cartesian pose error
        Eigen::VectorXd pose_;                           // Cartesian pose
        Eigen::Vector3d position_;                       // Cartesian position
        Eigen::Quaterniond orientation_;                 // Cartesian orientation as quaternioin
        Eigen::Matrix3d rot_mat_;                        // orientation as rotation matrix
        Eigen::Quaterniond orientation_des_;             // desired orientation as quaternion
        Eigen::Quaterniond orientation_error_;           // orientation error as quaternion
        double s_f_;                                     // desired progress parameter

        // kinematic model
        pinocchio::Data *data_;
        pinocchio::Model model_;
        pinocchio::FrameIndex flange_id_;

        // normalization matrices
        Eigen::MatrixXd N_q_;
        Eigen::MatrixXd N_p_;
        Eigen::MatrixXd N_F_;

        double cost_;           // MPC cost
        Eigen::VectorXd dp_dq_; // partial derivative of pose

        // partial derivatives of cost function
        Eigen::VectorXd dl_dq_;
        Eigen::VectorXd dl_dF_;
        Eigen::VectorXd dlq_ds_;
        Eigen::VectorXd dlp_ds_;
        Eigen::VectorXd dlF_ds_;
        Eigen::VectorXd dl_du_;
    };
    typedef std::shared_ptr<MPICProblemDescription> MPICProblemDescriptionPtr;
}
#endif // MPIC_PROBLEM_DESCRIPTION_HPP