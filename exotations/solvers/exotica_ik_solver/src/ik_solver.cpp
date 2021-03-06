/*
 * Copyright (c) 2016, University of Edinburgh
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met: 
 * 
 *  * Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer. 
 *  * Redistributions in binary form must reproduce the above copyright 
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the distribution. 
 *  * Neither the name of  nor the names of its contributors may be used to 
 *    endorse or promote products derived from this software without specific 
 *    prior written permission. 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE. 
 *
 */

#include <exotica_ik_solver/ik_solver.h>

#include <lapack/cblas.h>
#include "f2c.h"
#undef small
#undef large
#include <lapack/clapack.h>

REGISTER_MOTIONSOLVER_TYPE("IKSolver", exotica::IKSolver)

namespace exotica
{
IKSolver::IKSolver() = default;

IKSolver::~IKSolver() = default;

Eigen::MatrixXd inverseSymPosDef(const Eigen::Ref<const Eigen::MatrixXd>& A_)
{
    Eigen::MatrixXd Ainv_ = A_;
    double* AA = Ainv_.data();
    integer info;
    integer nn = A_.rows();
    // Compute Cholesky
    dpotrf_((char*)"L", &nn, AA, &nn, &info);
    if (info != 0)
    {
        throw_pretty("Can't invert matrix. Cholesky decomposition failed: " << info << std::endl
                                                                            << A_);
    }
    // Invert
    dpotri_((char*)"L", &nn, AA, &nn, &info);
    if (info != 0)
    {
        throw_pretty("Can't invert matrix: " << info << std::endl
                                             << A_);
    }
    Ainv_.triangularView<Eigen::Upper>() = Ainv_.transpose();
    return Ainv_;
}

void IKSolver::Instantiate(IKSolverInitializer& init)
{
    parameters_ = init;
}

void IKSolver::specifyProblem(PlanningProblem_ptr pointer)
{
    if (pointer->type() != "exotica::UnconstrainedEndPoseProblem")
    {
        throw_named("This IKSolver can't solve problem of type '" << pointer->type() << "'!");
    }
    MotionSolver::specifyProblem(pointer);
    prob_ = std::static_pointer_cast<UnconstrainedEndPoseProblem>(pointer);

    C_ = Eigen::MatrixXd::Identity(prob_->Cost.JN, prob_->Cost.JN) * parameters_.C;
    if (parameters_.C == 0.0)
        Cinv_ = Eigen::MatrixXd::Zero(prob_->Cost.JN, prob_->Cost.JN);
    else
        Cinv_ = C_.inverse();

    W_ = prob_->W;
    Winv_ = W_.inverse();

    if (parameters_.Alpha.size() != 1 && prob_->N != parameters_.Alpha.size())
        throw_named("Alpha must have length of 1 or N.");
}

void IKSolver::Solve(Eigen::MatrixXd& solution)
{
    prob_->resetCostEvolution(getNumberOfMaxIterations() + 1);

    Timer timer;

    if (!prob_) throw_named("Solver has not been initialized!");
    const Eigen::VectorXd q0 = prob_->applyStartState();

    if (prob_->N != q0.rows()) throw_named("Wrong size q0 size=" << q0.rows() << ", required size=" << prob_->N);

    const bool UseNullspace = prob_->qNominal.rows() == prob_->N;

    solution.resize(1, prob_->N);

    Eigen::VectorXd q = q0;
    double error = INFINITY;
    for (int i = 0; i < getNumberOfMaxIterations(); i++)
    {
        prob_->Update(q);
        Eigen::VectorXd yd = prob_->Cost.S * prob_->Cost.ydiff;

        error = prob_->getScalarCost();

        prob_->setCostEvolution(i, error);

        if (error < parameters_.Tolerance)
        {
            if (debug_)
                HIGHLIGHT_NAMED("IKSolver", "Reached tolerance (" << error << " < " << parameters_.Tolerance << ")");
            break;
        }

        Eigen::MatrixXd Jinv = PseudoInverse(prob_->Cost.S * prob_->Cost.J);
        Eigen::VectorXd qd = Jinv * yd;
        if (UseNullspace)
            qd += (Eigen::MatrixXd::Identity(prob_->N, prob_->N) - Jinv * prob_->Cost.S * prob_->J) *
                  (q - prob_->qNominal);

        ScaleToStepSize(qd);

        if (parameters_.Alpha.size() == 1)
        {
            q -= qd * parameters_.Alpha[0];
        }
        else
        {
            q -= qd.cwiseProduct(parameters_.Alpha);
        }

        if (qd.norm() < parameters_.Convergence)
        {
            if (debug_)
                HIGHLIGHT_NAMED("IKSolver", "Reached convergence (" << qd.norm() << " < " << parameters_.Convergence
                                                                    << ")");
            break;
        }
    }

    solution.row(0) = q;

    planning_time_ = timer.getDuration();
}

Eigen::MatrixXd IKSolver::PseudoInverse(Eigen::MatrixXdRefConst J)
{
    Eigen::MatrixXd Jpinv;

    if (J.cols() < J.rows())
    {
        //(Jt*C^-1*J+W)^-1*Jt*C^-1
        if (parameters_.C != 0)
        {
            Jpinv = inverseSymPosDef(J.transpose() * Cinv_ * J + W_) * J.transpose() * Cinv_;
        }
        else
        {
            Jpinv = inverseSymPosDef(J.transpose() * J + W_) * J.transpose();
        }
    }
    else
    {
        //W^-1*Jt(J*W^-1*Jt+C)
        Jpinv = Winv_ * J.transpose() * inverseSymPosDef(J * Winv_ * J.transpose() + C_);
    }

    return Jpinv;
}

void IKSolver::ScaleToStepSize(Eigen::VectorXdRef xd)
{
    double max_vel = xd.cwiseAbs().maxCoeff();
    if (max_vel > parameters_.MaxStep)
    {
        xd = xd * parameters_.MaxStep / max_vel;
    }
}
}
