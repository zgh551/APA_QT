#ifndef PATHLINE_H
#define PATHLINE_H

#include "Common/Math/math_utils.h"
#include "Common/Math/fresnel.h"
#include "Common/Math/vector_2d.h"
#include "math.h"

namespace math {
    /**
     * @brief Computation of the end point on a clothoid
     * @param x_i,y_i,theta_i,kappa_i: initial configuration
     * @param sigma: sharpness of clothoid
     * @param direction: driving direction {-1.0, 1.0}
     * @param length: length of clothoid (positive)
     * @param x_f,y_f,theta_f,kappa_f: final configuration on clothoid
     */
    void clothoid_to_end(double x_i, double y_i, double theta_i, double kappa_i,
                         double sigma, double direction, double length,
                         double *x_f, double *y_f, double *theta_f, double *kappa_f);


    /**
     * @brief Computation of the end point on a circular arc
     * @param x_i,y_i,theta_i: initial configuration
     * @param kappa: the curvature of the circular arc
     * @param direction: {-1.0, 1.0}
     * @param length: the lenght of the arc (positive)
     * @param x_f,y_f,theta_f: final configuration on circular arc
     */
    void circular_arc_to_end(double x_i, double y_i, double theta_i,
                             double kappa, double direction, double length,
                             double *x_f, double *y_f, double *theta_f);

    /**
     * @brief straight_line_to_end
     * @param x_i,y_i: initial configuration
     * @param theta: angle of straight line
     * @param direction: driving direction {-1.0, 1.0}
     * @param length: line length (positive)
     * @param x_f,y_f,theta_f,kappa_f: final configuration on straight line
     */
    void straight_line_to_end(double x_i, double y_i,
                              double theta, double direction, double length,
                              double *x_f, double *y_f);
}

#endif // PATHLINE_H
