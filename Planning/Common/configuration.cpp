#include "configuration.h"

using namespace math;

/**
 * @brief Configuration init
 * @param x: init x axis position
 * @param y: init y axis position
 * @param psi: init psi angle
 * @param kappa: init curvature
 */
Configuration::Configuration(double x, double y, double psi, double kappa)
{
    _x = x;
    _y = y;
    _psi = math::TwoPiNormallizeAngle( psi );
    _kappa = kappa;
}

/**
 * @brief calculate the distance betwen two configuration point
 * @param q: the other configuration point
 * @return the distance betwen two configuration point
 */
double Configuration::distance(const Configuration &q)const
{
    return (Vector2d(_x,_y) - Vector2d(q._x,q._y)).Length();
}

/**
 * @brief judgment two configuration point whether aligned
 * @param q: the other configuration point
 * @return true: aligned ; false: no aligned
 */
bool Configuration::aligned(const Configuration &q)const
{
    if( fabs(q._psi - _psi) > math::getEpsilon() )
    {
        return false;
    }
    double angle = math::TwoPiNormallizeAngle((Vector2d(_x,_y) - Vector2d(q._x,q._y)).Angle());
    return fabs(angle - _psi) <= math::getEpsilon();
}

/**
 * @brief udgment two configuration point whether equal
 * @param q: the other configuration point
 * @return true: equal ; false: no equal
 */
bool Configuration::equal(const Configuration &q)const
{
    if( fabs(q._psi - _psi) > math::getEpsilon() )
    {
        return false;
    }
    if( distance(q) > math::getEpsilon() )
    {
        return false;
    }
    return true;
}

double Configuration::getX()const{ return _x; }
double Configuration::getY()const{ return _y; }
double Configuration::getPsi()const{ return _psi; }
double Configuration::getKappa()const{ return _kappa; }
