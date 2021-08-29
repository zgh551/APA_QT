#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include "Common/Math/math_utils.h"
#include "Common/Math/vector_2d.h"

class Configuration
{
public:
    /**
     * @brief Configuration init
     * @param x: init x axis position
     * @param y: init y axis position
     * @param psi: init psi angle
     * @param kappa: init curvature
     */
    Configuration(double x = 0.0, double y = 0.0, double psi = 0.0, double kappa = 0.0);

    /**
     * @brief calculate the distance betwen two configuration point
     * @param q: the other configuration point
     * @return the distance betwen two configuration point
     */
    double distance(const Configuration &q) const;

    /**
     * @brief judgment two configuration point whether aligned
     * @param q: the other configuration point
     * @return true: aligned ; false: no aligned
     */
    bool aligned(const Configuration &q) const;

    /**
     * @brief udgment two configuration point whether equal
     * @param q: the other configuration point
     * @return true: equal ; false: no equal
     */
    bool equal(const Configuration &q) const;

    double getX(void) const;

    double getY(void) const;

    double getPsi(void) const;

    double getKappa(void) const;

private:
    /**
     * @brief x: the x axis position
     */
    double _x;

    /**
     * @brief y: the y axis position
     */
    double _y;

    /**
     * @brief psi: orientation in rad between [0, 2*pi)
     */
    double _psi;

    /**
     * @brief kappa: the curvature of circle arc
     */
    double _kappa;
};



#endif // CONFIGURATION_H
