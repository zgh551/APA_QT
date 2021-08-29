#ifndef HC_CC_CIRCLE_H
#define HC_CC_CIRCLE_H

#include <QMainWindow>
#include <limits>
#include "Planning/Common/configuration.h"
#include "Common/Math/fresnel.h"

class HC_CC_Circle_Param
{
public:
    /**
     * @brief set the circle Parameter
     * @param kappa: max curvature
     * @param sigma: the max sharpness of clothoid
     * @param radius: the radius of the outer circle
     * @param mu: Angle between the initial orientation and the tangent to the circle at the initial position
     */
    void setParam(double kappa, double sigma, double radius, double mu);

    double getKappa(void) const { return _kappa; }

    double getKappaInv(void) const { return  _kappa_inv; }

    double getSigma(void) const { return _sigma; }

    double getRadius(void) const { return _radius; }

    double getMu(void) const { return _mu; }

    double getSinMu(void) const { return _sin_mu; }

    double getCosMu(void) const { return _cos_mu; }

    double getDeltaMin(void) const { return _delta_min; }

    double getLenghtMin(void) const { return _lenght_min; }
protected:
    /**
     * @brief _kappa: Max. curvature
     */
    double _kappa;

    /**
     * @brief _kappa_inv: inverse of max. curvature
     */
    double _kappa_inv;

    /**
     * @brief _sigma: max. sharpness
     */
    double _sigma;

    /**
     * @brief _radius: Radius of the outer circle
     */
    double _radius;

    /**
     * @brief _mu: Angle between the initial orientation and the tangent to the circle at the initial position
     */
    double _mu;

    /**
     * @brief _sin_mu: sine of mu
     */
    double _sin_mu;

    /**
     * @brief _cos_mu: cosine of mu
     */
    double _cos_mu;

    /**
     * @brief _delta_min: Minimal deflection(偏转)
     */
    double _delta_min;

    /**
     * @brief _lenght_min: the min lenght of clothoid
     */
    double _lenght_min;
};

class HC_CC_Circle: public HC_CC_Circle_Param
{
public:
    /**
     * @brief Init the hc or cc circle param Constructor
     * @param start: the start point configuration
     * @param left: Turning direction -> [left: true, right: false]
     * @param forward: Driving direction -> [forward: true, backward: false]
     * @param regular: Type of the circle -> [regular: true, irregular: false]
     * @param param: the circle param
     */
    HC_CC_Circle(const Configuration &start,bool left, bool forward, bool regular,const HC_CC_Circle_Param &param);

    /**
     * @brief Init the hc or cc circle param Constructor
     * @param x_c: the x axis point of the center of circle
     * @param y_c: the y axis point of the center of circle
     * @param left: Turning direction -> [left: true, right: false]
     * @param forward: Driving direction -> [forward: true, backward: false]
     * @param regular: Type of the circle -> [regular: true, irregular: false]
     * @param param: the circle param
     */
    HC_CC_Circle(double x_c, double y_c, bool left, bool forward, bool regular,HC_CC_Circle_Param &param);

    /**
     * @brief Computation of the distance between the canters of two circles
     * @param c :the other circle
     * @return the distance between current circle and another circle
     */
    double CenterDistance(const HC_CC_Circle &c) const;

    /**
     * @brief judgment configuration point whether on the hc or cc circle
     * @param q :the configuration point
     * @return true: on the circle ; false : not on the circle
     */
    bool isConfigurationOn_HC_CC_Circle(const Configuration &q) const;

    /**
     * @brief Computation of deflection(angle between start configuration of circle and configuration q)
     * @param q :Configuration point
     * @return the deflection angle
     */
    double deflection(const Configuration &q) const;

    /**
     * @brief Computation of a rs-turn's circular deflection
     * @param delta :the deflection of rs-turn
     * @return the deflection angle(rad)
     */
    double rs_circular_deflection(double delta) const;

    /**
     * @brief Computation of the Length of a rs-turn betwen current circle and configuration point
     * @param q :the configuration point q
     * @return the lenght of rs-turn
     */
    double rs_turn_lenght(const Configuration &q) const;

    /**
     * @brief Computation of a hc-turn's circular deflection
     * @param delta :the deflection of hc-turn
     * @return the deflection angle(rad)
     */
    double hc_circular_deflection(double delta) const;

    /**
     * @brief Computation of the Length of a hc-turn betwen current circle and configuration point
     * @param q :the configuration point q
     * @return the lenght of hc-turn
     */
    double hc_turn_lenght(const Configuration &q) const;

    /**
     * @brief Computation of an elementary path's sharpness
     * @param q :the configuration point
     * @param delta :the deflection angle
     * @param sigma :the path's sharpness
     * @return whether exist elementary path
     */
    bool cc_elementary_path_sharpness(const Configuration &q, double delta, double &sigma) const;

    /**
     * @brief Computation of a cc-turn's circular deflection
     * @param delta: the deflection of cc-turn
     * @return the deflection angle(rad)
     */
    double cc_circular_deflection(double delta) const;

    /**
     * @brief Computation of the Length of a cc-turn betwen current circle and configuration point
     * @param q: the configuration point q
     * @return the lenght of cc-turn
     */
    double cc_turn_lenght(const Configuration &q) const;

    /** Read only the variable **/
    Configuration getStart(void);

    bool getLeft(void);

    bool getForward(void);

    bool getRegular(void);

    double getCenter_x(void);

    double getCenter_y(void);
private:
    /**
     * @brief _start: Start point configuration
     */
    Configuration _start;

    /**
     * @brief _left: Turning direction -> [left: true, right: false]
     */
    bool _left;

    /**
     * @brief _forward: Driving direction -> [forward: true, backward: false]
     */
    bool _forward;

    /**
     * @brief _regular: Type of the circle -> [regular: true, irregular: false]
     */
    bool _regular;

    /**
     * @brief _x_c: x axis position at the center of the circle
     */
    double _x_c;

    /**
     * @brief _y_c: y axis position at the center of the circle
     */
    double _y_c;
};

#endif // HC_CC_CIRCLE_H
