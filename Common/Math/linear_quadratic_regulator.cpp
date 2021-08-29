#include "linear_quadratic_regulator.h"

using Matrix = Eigen::MatrixXd;

namespace math{

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
void SolveLQR(const Matrix &A, const Matrix &B,
                                        const Matrix &Q, const Matrix &R,
                                        const double tolerance, const uint16_t max_num_iteration,
                                        Matrix *ptr_k)
{
    if(A.rows() != A.cols() || A.rows() != B.rows() || Q.rows() != Q.cols() ||
       Q.rows() != A.rows() || R.rows() != R.cols() || R.rows() != B.cols() )
    {
//        qDebug() << "LQR solver: one or more matrices have incompatible dimensions.";
        return;
    }
    // 求转置矩阵
    Matrix AT = A.transpose();
    Matrix BT = B.transpose();

    // 求解离散时间代数 Riccati 方程(DARE)
    Matrix P = Q;
    uint16_t num_iteration = 0;//初始化迭代计数
    double diff = 1.7976931348623158e+308;
    while((num_iteration++ < max_num_iteration) && (diff > tolerance))
    {
        Matrix P_next =
            AT * P * A -
            (AT * P * B) * (R + BT * P * B).inverse() * (BT * P * A) + Q;
        // 检查P和P_next之间的差值
        diff = fabs((P_next - P).maxCoeff());
        P = P_next;
    }
    *ptr_k = (R + BT * P * B).inverse() * BT * P * A;
}

}




