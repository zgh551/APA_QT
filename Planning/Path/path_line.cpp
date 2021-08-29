#include "path_line.h"

namespace  math{
/**
 * @brief Computation of the end point on a clothoid
 * @param x_i,y_i,theta_i,kappa_i: initial configuration
 * @param sigma: sharpness of clothoid
 * @param direction: driving direction {-1.0, 1.0}
 * @param length: length of clothoid (positive)
 * @param x_f,y_f,theta_f,kappa_f: final configuration on clothoid
 */
void clothoid_to_end( double x_i, double y_i, double theta_i, double kappa_i,
                                double sigma, double direction, double length,
                                double *x_f, double *y_f, double *theta_f, double *kappa_f)
{
    double sgn_sigma = math::sgn(sigma);
    double abs_sigma = fabs(sigma);
    double sqrt_sigma_inv = 1 / sqrt(abs_sigma);

    double delta_i = 0.5 * direction * kappa_i * kappa_i / sigma; // from init point to i point the one clothoid change orientation
    double rotate_theta = theta_i - delta_i;

    double lenght_i = MV_PI_SQRT_INV * sqrt_sigma_inv * sgn_sigma * kappa_i;
    double lenght_f = MV_PI_SQRT_INV * sqrt_sigma_inv * (abs_sigma * length + sgn_sigma * kappa_i);

    double fresnel_s_i;
    double fresnel_c_i;
    double fresnel_s_f;
    double fresnel_c_f;
    math::Fresnel(lenght_i,fresnel_s_i,fresnel_c_i);
    math::Fresnel(lenght_f,fresnel_s_f,fresnel_c_f);

    Vector2d clothoid_init_point,clothoid_final_point;
    Vector2d init_point,final_point;
    clothoid_init_point  = Vector2d(direction * fresnel_c_i,sgn_sigma * fresnel_s_i);
    clothoid_final_point = Vector2d(direction * fresnel_c_f,sgn_sigma * fresnel_s_f);
    init_point = Vector2d(x_i,y_i);

    final_point = init_point + (clothoid_final_point - clothoid_init_point).rotate(rotate_theta) * MV_PI_SQRT * sqrt_sigma_inv;

    *x_f = final_point.getX();
    *y_f = final_point.getY();

    //
    *theta_f = math::NormallizeAngle(theta_i +
                                     kappa_i * direction * length +
                                     0.5 * sigma * direction * length * length);

    // k = sigma * s
    *kappa_f = kappa_i + sigma * length;
}

/**
 * @brief Computation of the end point on a circular arc
 * @param x_i,y_i,theta_i,kappa_i: initial configuration
 * @param kappa: the curvature of the circular arc
 * @param direction: {-1.0, 1.0}
 * @param length: the lenght of the arc (positive)
 * @param x_f,y_f,theta_f,kappa_f: final configuration on clothoid
 */
void circular_arc_to_end( double x_i, double y_i, double theta_i,
                                    double kappa, double direction, double length,
                                    double *x_f, double *y_f, double *theta_f)
{
    *theta_f = math::NormallizeAngle(theta_i + direction * kappa * length);
    *x_f = x_i + (1 / kappa) * ( -sin(theta_i) + sin(*theta_f) );
    *y_f = y_i + (1 / kappa) * (  cos(theta_i) - cos(*theta_f) );
}

/**
 * @brief straight_line_to_end
 * @param x_i,y_i: initial configuration
 * @param theta: angle of straight line
 * @param direction: driving direction {-1.0, 1.0}
 * @param length: line length (positive)
 * @param x_f,y_f,theta_f,kappa_f: final configuration on straight line
 */
void straight_line_to_end( double x_i, double y_i,
                           double theta, double direction, double length,
                           double *x_f, double *y_f)
{
    *x_f = x_i + direction * length * cos(theta);
    *y_f = y_i + direction * length * sin(theta);
}

}
