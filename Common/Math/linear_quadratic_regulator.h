#ifndef LINEARQUADRATICREGULATOR_H
#define LINEARQUADRATICREGULATOR_H

#include <QMainWindow>
/**
 * @brief Eigen矩阵库
 */
#include <Eigen/Dense>

namespace math
{
    /**
     * @brief Solver for discrete-time linear quadratic problem.
     * @param A The system dynamic matrix
     * @param B The control matrix
     * @param Q The cost matrix for system state
     * @param R The cost matrix for control output
     * @param tolerance The numerical tolerance for solving Discrete
     *        Algebraic Riccati equation (DARE)
     * @param max_num_iteration The maximum iterations for solving ARE
     * @param ptr_K The feedback control matrix (pointer)
     */
    void SolveLQR(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                  const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
                  const double tolerance, const uint16_t max_num_iteration,
                  Eigen::MatrixXd *ptr_k);
}

#endif // LINEARQUADRATICREGULATOR_H
