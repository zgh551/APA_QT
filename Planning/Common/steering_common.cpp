#include "steering_common.h"

namespace steering {
/**
 * @brief Checks whether two states are equal
 * @param state1 :one state
 * @param state2 :the another state
 * @return return the result
 */
bool StateEqual(const State &state1, const State &state2)
{
    if( fabs(state2.kappa - state1.kappa) > math::getEpsilon() )
    {
        return false;
    }
    else if( fabs(math::TwoPiNormallizeAngle(state2.psi)  -
                  math::TwoPiNormallizeAngle(state1.psi)) >
                  math::getEpsilon())
    {
        return false;
    }
    else if( math::PointDistance(state1.x, state1.y, state2.x, state2.y) > math::getEpsilon() )
    {
        return false;
    }
    else
    {
        return true;
    }
}

/**
 * @brief Reverses a control
 * @param the reversed control
 */
void ReverseControl(Control &control)
{
    control.delta_s = -control.delta_s;
    control.kappa   =  control.kappa + fabs(control.delta_s) * control.sigma;
    control.sigma   = -control.sigma;
}

/**
 * @brief Subtracts control2 from control1
 * @param control1
 * @param control2
 * @return return the control1 - control2
 */
Control SubtractControl(const Control &control1, const Control &control2)
{
    // 两组控制可以相减的前提是控制角速度方向一致
    Q_ASSERT(fabs(math::sgn(control1.delta_s) * control1.sigma  -
                  math::sgn(control2.delta_s) * control2.sigma) <
                  math::getEpsilon() );

    Control control;
    control.delta_s = control1.delta_s - control2.delta_s;
    control.kappa = control1.kappa;
    control.sigma = control1.sigma;
    return control;
}

/**
 * @brief Appends controls with 0 input
 * @param controls :the update control sets
 */
void EmptyControls( vector<Control> &controls )
{
    Control control;
    control.delta_s = 0.0;
    control.kappa = 0.0;
    control.sigma = 0.0;
    controls.push_back(control);
}

/**
 * @brief Appends controls with a straight line
 * @param q1 :the start configuration point of the straight line
 * @param q2 :the end configuration point of the straight line
 * @param controls :output the result to the controls
 */
void StraightControls(const Configuration &q1, const Configuration &q2, vector<Control> &controls)
{
    double length = math::PointDistance(q1.getX(), q1.getY(), q2.getX(), q2.getY());
    double dot_product = cos(q1.getPsi()) * ( q2.getX() - q1.getX() ) +
                         sin(q1.getPsi()) * ( q2.getY() - q1.getY() ) ;
    double d = math::sgn(dot_product);

    Control control_straight_line;
    control_straight_line.delta_s = d * length;
    control_straight_line.kappa = 0.0;
    control_straight_line.sigma = 0.0;
    controls.push_back(control_straight_line);
    return;
}

static double Direction(bool forward, bool order)
{
    if(forward && order){ return 1.0; }
    else if( forward && !order){ return -1.0; }
    else if(!forward &&  order){ return -1.0; }
    else if(!forward && !order){ return  1.0; }
    else { return 0.0; }
}

/**
 * @brief Appends controls with a rs-turn
 * @param c :the circle of rs-turn
 * @param q :the configuration q
 * @param order :the order of control[true-> positive oder, false->negative oder]
 * @param controls :output the control commond
 */
void RS_TurnControls(HC_CC_Circle &c, const Configuration &q, bool order, vector<Control> &controls)
{
    Q_ASSERT(fabs(fabs(c.getKappa()) - fabs(q.getKappa())) < math::getEpsilon() &&
             fabs(fabs(c.getSigma()) - numeric_limits<double>::max()) < math::getEpsilon());

    double delta = c.deflection(q);
    double length_arc = fabs(c.getKappaInv()) * c.rs_circular_deflection(delta);
    double d = Direction(c.getForward(), order);

    Control control_arc;
    control_arc.delta_s = d * length_arc;
    control_arc.kappa = c.getKappa();
    control_arc.sigma = 0.0;
    controls.push_back( control_arc );
    return;
}

/**
 * @brief Appends controls with a hc-turn
 * @param c :the circle of hc-turn
 * @param q :the configuration q
 * @param order :the order of control[true-> positive oder, false->negative oder]
 * @param controls :output the control commond
 */
void HC_TurnControls(HC_CC_Circle &c, const Configuration &q, bool order, vector<Control> &controls)
{
    Q_ASSERT(fabs(fabs(c.getKappa()) - fabs(q.getKappa())) < math::getEpsilon());
    double delta = c.deflection(q);
    double length_min = fabs(c.getKappa() / c.getSigma());
    double length_arc = fabs(c.getKappaInv()) * c.hc_circular_deflection(delta);
    double d = Direction(c.getForward(), order);

    Control control_arc, control_clothoid;
    if(order)
    {
        control_clothoid.delta_s = d * length_min;
        control_clothoid.kappa = 0.0;
        control_clothoid.sigma = c.getSigma();
        controls.push_back(control_clothoid);
    }

    control_arc.delta_s = d * length_arc;
    control_arc.kappa = c.getKappa();
    control_arc.sigma = 0.0;
    controls.push_back(control_arc);

    if(!order)
    {
        control_clothoid.delta_s = d * length_min;
        control_clothoid.kappa = c.getKappa();
        control_clothoid.sigma = -c.getSigma();
        controls.push_back(control_clothoid);
    }
    return;
}

/**
 * @brief Appends controls with a cc-turn
 * @param c :the circle of cc-turn
 * @param q :the configuration q
 * @param order :the order of control[true-> positive oder, false->negative oder]
 * @param controls :output the control commond
 */
void CC_TurnControls(HC_CC_Circle &c, const Configuration &q, bool order, vector<Control> &controls)
{
    Q_ASSERT(fabs(q.getKappa()) < math::getEpsilon());
    double delta = c.deflection(q);

    if(delta < math::getEpsilon())// delta == 0
    {
        if(order)
        {
            StraightControls(c.getStart(), q, controls);
        }
        else
        {
            StraightControls(q, c.getStart(), controls);
        }
    }
    else if(delta < 2 * c.getDeltaMin()) // 0 < delta < 2 * delta_min
    {
        vector<Control> controls_elementary, controls_default;
        if(CC_ElementaryControls(c, q, delta, order, controls_elementary))
        {
            CC_DefaultControls(c, q, delta, order, controls_default);
            double length_elementary =
                    accumulate(controls_elementary.begin(), controls_elementary.end(),0.0,
                    [](double sum, const Control &control) {return sum + fabs(control.delta_s);});

            double length_default =
                    accumulate(controls_default.begin(), controls_default.end(),0.0,
                    [](double sum, const Control &control) {return sum + fabs(control.delta_s);});

            (length_elementary < length_default) ?
            controls.insert(controls.end(), controls_elementary.begin(), controls_elementary.end()) :
            controls.insert(controls.end(), controls_default.begin(), controls_default.end());
            return;
        }
        else
        {
            CC_DefaultControls(c, q, delta, order, controls);
            return;
        }
    }
    else // delta >= 2 * delta_min
    {
        CC_DefaultControls(c, q, delta, order, controls);
        return;
    }
}

/**
 * @brief Appends controls with a default cc-turn consisting of two clothoids and a circular arc
 * @param c :the circle of cc-turn
 * @param q :the configuration q
 * @param delta :the angle betwen start configuration of circle and configuration q
 * @param order :the order of control[true-> positive oder, false->negative oder]
 * @param controls :output the control commond
 */
void CC_DefaultControls(HC_CC_Circle &c, const Configuration &q, double delta, bool order, vector<Control> &controls)
{
    double length_min = fabs(c.getKappa() / c.getSigma());
    double length_arc = fabs(c.getKappaInv()) * c.cc_circular_deflection(delta);
    double d = Direction(c.getForward(), order);

    Control control_clothoid1,control_arc,control_clothoid2;
    // first clothoid path
    control_clothoid1.delta_s = d * length_min;
    control_clothoid1.kappa = 0.0;
    control_clothoid1.sigma = c.getSigma();
    controls.push_back(control_clothoid1);

    // circle arc path
    control_arc.delta_s = d * length_arc;
    control_arc.kappa = c.getKappa();
    control_arc.sigma = 0.0;
    controls.push_back(control_arc);

    // second clothoid path
    control_clothoid2.delta_s = d * length_min;
    control_clothoid2.kappa = c.getKappa();
    control_clothoid2.sigma = -c.getSigma();
    controls.push_back(control_clothoid2);
    return;
}

/**
 * @brief Appends controls with an elementary path if one exists
 * @param c :the circle of cc-turn
 * @param q :the configuration q
 * @param delta :delta :the angle betwen start configuration of circle and configuration q
 * @param order :the order of control[true-> positive oder, false->negative oder]
 * @param controls :output the control commond
 * @return if exist return true else return false
 */
bool CC_ElementaryControls(HC_CC_Circle &c, const Configuration &q, double delta, bool order, vector<Control> &controls)
{
    double sigma0;
    if(c.cc_elementary_path_sharpness(q, delta, sigma0))
    {
        double length = sqrt(delta / fabs(sigma0));
        double d = Direction(c.getForward(), order);

        Control control_clothoid1,control_clothoid2;

        control_clothoid1.delta_s = d * length;
        control_clothoid1.kappa = 0.0;
        control_clothoid1.sigma = sigma0;
        controls.push_back(control_clothoid1);

        control_clothoid2.delta_s = d * length;
        control_clothoid2.kappa = sigma0 * length;
        control_clothoid2.sigma = -sigma0;
        controls.push_back(control_clothoid2);
        return true;
    }
    else
    {
        return false;
    }
}

}// end of name space
