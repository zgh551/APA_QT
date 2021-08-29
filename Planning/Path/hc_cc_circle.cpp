#include "hc_cc_circle.h"

using namespace math;

/**
 * @brief set the circle Parameter
 * @param kappa: max curvature
 * @param sigma: the max sharpness of clothoid
 * @param radius: the radius of the outer circle
 * @param mu: Angle between the initial orientation and the tangent to the circle at the initial position
 */
void HC_CC_Circle_Param::setParam(double kappa, double sigma, double radius, double mu)
{
    /**
     * @brief _kappa: Max. curvature
     */
    _kappa = kappa;

    /**
     * @brief _kappa_inv: inverse of max. curvature
     */
    _kappa_inv = 1 / kappa;

    /**
     * @brief _sigma: max. sharpness
     */
    _sigma = sigma;

    /**
     * @brief _radius: Radius of the outer circle
     */
    _radius = radius;

    /**
     * @brief _mu: Angle between the initial orientation and the tangent to the circle at the initial position
     */
    _mu = mu;

    /**
     * @brief _sin_mu: sine of mu
     */
    _sin_mu = sin(mu);

    /**
     * @brief _cos_mu: cosine of mu
     */
    _cos_mu = cos(mu);

    /**
     * @brief _delta_min: Minimal deflection(偏转)
     */
    _delta_min = 0.5 * kappa * kappa / sigma;


    _lenght_min = kappa / sigma;
}

// 描述HC 和 CC圆的性质
HC_CC_Circle::HC_CC_Circle(const Configuration &start,bool left, bool forward, bool regular,const HC_CC_Circle_Param &param)
{
    _start = start;
    _left  = left;
    _forward = forward;
    _regular = regular;

    _radius = param.getRadius();
    _mu     = param.getMu();
    _sin_mu = param.getSinMu();
    _cos_mu = param .getCosMu();
    _delta_min = param.getDeltaMin();

    // the center of circle calculate
    double delta_x = _radius * _sin_mu;
    double delta_y = _radius * _cos_mu;

    if(_left)
    {
        _kappa     = param.getKappa();
        _kappa_inv = param.getKappaInv();
        _sigma     = param.getSigma();

        if(_forward)
        {
            math::change_to_global_frame(_start.getX(),_start.getY(),_start.getPsi(),
                                         delta_x, delta_y,
                                           &_x_c,   &_y_c);
        }
        else
        {
            math::change_to_global_frame(_start.getX(),_start.getY(),_start.getPsi(),
                                         -delta_x, delta_y,
                                            &_x_c,   &_y_c);
        }
    }
    else
    {
        _kappa     = -param.getKappa();
        _kappa_inv = -param.getKappaInv();
        _sigma     = -param.getSigma();
        if(_forward)
        {
            math::change_to_global_frame(_start.getX(),_start.getY(),_start.getPsi(),
                                         delta_x, -delta_y,
                                           &_x_c,    &_y_c);
        }
        else
        {
            math::change_to_global_frame(_start.getX(),_start.getY(),_start.getPsi(),
                                         -delta_x, -delta_y,
                                            &_x_c,    &_y_c);
        }
    }
}

/**
 * @brief Init the hc or cc circle param Constructor
 * @param x_c: the x axis point of the center of circle
 * @param y_c: the y axis point of the center of circle
 * @param left: Turning direction -> [left: true, right: false]
 * @param forward: Driving direction -> [forward: true, backward: false]
 * @param regular: Type of the circle -> [regular: true, irregular: false]
 * @param param: the circle param
 */
HC_CC_Circle::HC_CC_Circle(double x_c, double y_c, bool left, bool forward, bool regular,HC_CC_Circle_Param &param)
{
    _start = Configuration(0, 0, 0, 0);
    _left  = left;
    _forward = forward;
    _regular = regular;

    _x_c = x_c;
    _y_c = y_c;

    _radius = param.getRadius();
    _mu     = param.getMu();
    _sin_mu = param.getSinMu();
    _cos_mu = param .getCosMu();
    _delta_min = param.getDeltaMin();

    if(_left)
    {
        _kappa     = param.getKappa();
        _kappa_inv = param.getKappaInv();
        _sigma     = param.getSigma();
    }
    else
    {
        _kappa     = -param.getKappa();
        _kappa_inv = -param.getKappaInv();
        _sigma     = -param.getSigma();
    }
}

/**
 * @brief Computation of the distance between the canters of two circles
 * @param c :the other circle
 * @return the distance between current circle and another circle
 */
double HC_CC_Circle::CenterDistance(const HC_CC_Circle &c) const
{
    return math::PointDistance(c._x_c, c._y_c, this->_x_c, this->_y_c);
}

/**
 * @brief judgment configuration point whether on the hc or cc circle
 * @param q :the configuration point
 * @return true: on the circle ; false : not on the circle
 */
bool HC_CC_Circle::isConfigurationOn_HC_CC_Circle(const Configuration &q) const
{
    double distance = math::PointDistance(this->_x_c, this->_y_c, q.getX(), q.getY());

    if(fabs(distance - this->_radius) > math::getEpsilon())
    {
        return false;
    }

    double angle = atan2(q.getY() - this->_y_c, q.getX() - this->_x_c);
    if(this->_left && this->_forward)
    {
        angle = angle + MV_PI2 - this->_mu;
    }
    else if(this->_left && !this->_forward)
    {
        angle = angle + MV_PI2 + this->_mu;
    }
    else if(!this->_left && this->_forward)
    {
        angle = angle - MV_PI2 + this->_mu;
    }
    else if(!this->_left && !this->_forward)
    {
        angle = angle - MV_PI2 - this->_mu;
    }
    angle = math::TwoPiNormallizeAngle(angle);
    return fabs(q.getPsi() - angle) < math::getEpsilon();
}

/**
 * @brief Computation of deflection(angle between start configuration of circle and configuration q)
 * @param q: Configuration point
 * @return the deflection angle
 */
double HC_CC_Circle::deflection(const Configuration &q) const
{
    double psi_c = this->_start.getPsi();
    double psi_q = q.getPsi();
    if( this->_left && this->_forward )
    {
        return math::TwoPiNormallizeAngle(psi_q - psi_c);
    }
    else if( this->_left && !this->_forward )
    {
        return math::TwoPiNormallizeAngle(psi_c - psi_q);
    }
    else if( !this->_left && this->_forward )
    {
        return math::TwoPiNormallizeAngle(psi_c - psi_q);
    }
    else if( !this->_left && !this->_forward )
    {
        return math::TwoPiNormallizeAngle(psi_q - psi_c);
    }
    else
    {
        return -1;
    }
}

/**
 * @brief Computation of a rs-turn's circular deflection
 * @param delta: the deflection of rs-turn
 * @return the deflection angle(rad)
 */
double HC_CC_Circle::rs_circular_deflection(double delta) const
{
    if( this->_regular )
    {
        return delta;
    }
    else
    {
        return (delta <= MV_PI) ? delta : delta - MV_2PI;
    }
}

/**
 * @brief Computation of the Length of a rs-turn betwen current circle and configuration point
 * @param q: the configuration point q
 * @return the lenght of rs-turn
 */
double HC_CC_Circle::rs_turn_lenght(const Configuration &q) const
{
    Q_ASSERT( fabs(fabs(this->_kappa) - fabs(q.getKappa()))                 < math::getEpsilon()&&
              fabs(fabs(this->_sigma) - std::numeric_limits<double>::max()) < math::getEpsilon());
    double delta = this->deflection(q);
    return fabs(this->_kappa_inv * this->rs_circular_deflection(delta));
}

/**
 * @brief Computation of a hs-turn's circular deflection
 * @param delta: the deflection of hs-turn
 * @return the deflection angle(rad)
 */
double HC_CC_Circle::hc_circular_deflection(double delta) const
{
    double delta_min_twopified = math::TwoPiNormallizeAngle(this->_delta_min);

    if( this->_regular )
    {
        if( delta < delta_min_twopified)
        {
            return MV_2PI + delta - delta_min_twopified;
        }
        else
        {
            return delta - delta_min_twopified;
        }
    }
    else
    {
        double delta_arc1, delta_arc2;
        if( delta < delta_min_twopified)
        {
            delta_arc1 = delta - delta_min_twopified; // negative
            delta_arc2 = delta_arc1 + MV_2PI;          // positive
        }
        else
        {
            delta_arc1 = delta - delta_min_twopified; // positive
            delta_arc2 = delta_arc1 - MV_2PI;          // negative
        }
        return ( fabs(delta_arc1) < fabs(delta_arc2) ) ?  delta_arc1 : delta_arc2;
    }
}

/**
 * @brief Computation of the Length of a hs-turn betwen current circle and configuration point
 * @param q: the configuration point q
 * @return the lenght of hs-turn
 */
double HC_CC_Circle::hc_turn_lenght(const Configuration &q) const
{
    Q_ASSERT( fabs( fabs(this->_kappa) - fabs(q.getKappa())) < math::getEpsilon());
    double delta = this->deflection(q);
    return fabs(this->_lenght_min) + fabs(this->_kappa_inv * this->hc_circular_deflection(delta));
}

/**
 * @brief Computation of an elementary path's sharpness
 * @param q :the configuration point
 * @param delta :the deflection angle
 * @param sigma :the path's sharpness
 * @return whether exist elementary path
 */
bool HC_CC_Circle::cc_elementary_path_sharpness(const Configuration &q, double delta, double &sigma) const
{
    double frenel_s, frenel_c;
    double distance = this->_start.distance(q);
    if(delta < 4.5948 && distance > math::getEpsilon())
    {
        double s = sqrt(delta / MV_PI);
        math::Fresnel(s, frenel_s, frenel_c);
        double d1 = cos(0.5 * delta) * frenel_c + sin(0.5 * delta) * frenel_s;
        sigma = 4 * MV_PI * pow(d1, 2) / pow(distance, 2);
        if(!this->_left)
        {
            sigma = -sigma;
        }
        return true;
    }
    return false;
}

/**
 * @brief Computation of a cc-turn's circular deflection
 * @param delta: the deflection of cc-turn
 * @return the deflection angle(rad)
 */
double HC_CC_Circle::cc_circular_deflection(double delta) const
{
    double two_delta_min_twopified = math::TwoPiNormallizeAngle(2 * this->_delta_min);
    if(this->_regular)
    {
        if( delta < two_delta_min_twopified )
        {
            return MV_2PI + delta - two_delta_min_twopified;
        }
        else
        {
            return delta - two_delta_min_twopified;
        }
    }
    else
    {
        double delta_arc1, delta_arc2;
        if (delta < two_delta_min_twopified)
        {
          delta_arc1 = delta - two_delta_min_twopified;  // negative
          delta_arc2 = delta_arc1 + MV_2PI;              // positive
        }
        else
        {
          delta_arc1 = delta - two_delta_min_twopified;  // positive
          delta_arc2 = delta_arc1 - MV_2PI;              // negative
        }
        return (fabs(delta_arc1) < fabs(delta_arc2)) ? delta_arc1 : delta_arc2;
    }
}

/**
 * @brief Computation of the Length of a cc-turn betwen current circle and configuration point
 * @param q: the configuration point q
 * @return the lenght of cc-turn
 */
double HC_CC_Circle::cc_turn_lenght(const Configuration &q) const
{
    Q_ASSERT(fabs(q.getKappa()) < math::getEpsilon() );
    double delta = this->deflection(q);

    double lenght_default = 2 * this->_lenght_min + fabs( this->_kappa_inv * this->cc_circular_deflection(delta));
    if(delta < math::getEpsilon())// delta = 0
    {
        return 2 * this->_radius * this->_sin_mu;
    }
    else if(delta < 2 * this->_delta_min)// 0 < delta < 2 * delta_min
    {
        double sigma_e;

        if( this->cc_elementary_path_sharpness(q,delta, sigma_e))
        {
            double length_elementary = 2 * sqrt( delta / fabs(sigma_e));
            return ( length_elementary < lenght_default ) ? length_elementary : lenght_default;
        }
        else
        {
            return lenght_default;
        }
    }
    else // delta >= 2 * delta_min
    {
        return lenght_default;
    }
}

Configuration HC_CC_Circle::getStart(void){ return _start; }

bool HC_CC_Circle::getLeft(void){ return _left; }

bool HC_CC_Circle::getForward(void){ return _forward; }

bool HC_CC_Circle::getRegular(void){ return _regular; }

double HC_CC_Circle::getCenter_x(void){ return _x_c; }

double HC_CC_Circle::getCenter_y(void){ return _y_c; }
