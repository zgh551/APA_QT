#ifndef STEERINGCOMMON_H
#define STEERINGCOMMON_H

#include <vector>
#include <cassert>
#include <limits>

#include "Common/Math/math_utils.h"
#include "Planning/Common/configuration.h"
#include "Planning/Path/hc_cc_circle.h"

using namespace std;
using namespace math;

namespace steering {

/**
 * @brief The State struct: Description of a kinematic car's state
 */
struct State
{
    /**
     * @brief x :Position in x of the robot
     */
    double x;

    /**
     * @brief y :Position in y of the robot
     */
    double y;

    /**
     * @brief theta :Orientation of the robot
     */
    double psi;

    /**
     * @brief kappa :Curvature at position (x,y)
     */
    double kappa;

    /**
     * @brief d :Driving direction {-1,0,1}
     */
    double d;
};

/**
 * @brief Checks whether two states are equal
 * @param state1 :one state
 * @param state2 :the another state
 * @return return the result
 */
bool StateEqual(const State &state1, const State &state2);

/**
 * @brief The Control struct :Description of a path segment with its corresponding control inputs
 */
struct Control
{
    /**
     * @brief delta_s :Signed arc length of a segment
     */
    double delta_s;

    /**
     * @brief kappa :Curvature at the beginning of a segment
     */
    double kappa;

    /**
     * @brief sigma :Sharpness (derivative of curvature with respect to arc length) of a segment
     */
    double sigma;
};

/**
 * @brief Reverses a control
 * @param the reversed control
 */
void ReverseControl(Control &control);

/**
 * @brief Subtracts control2 from control1
 * @param control1
 * @param control2
 * @return return the control1 - control2
 */
Control SubtractControl(const Control &control1, const Control &control2);

/**
 * @brief Appends controls with 0 input
 * @param controls :output the control commond
 */
void EmptyControls( vector<Control> &controls );

/**
 * @brief Appends controls with a straight line
 * @param q1 :the start configuration point of the straight line
 * @param q2 :the end configuration point of the straight line
 * @param controls :output the result to the controls
 */
void StraightControls(const Configuration &q1, const Configuration &q2, vector<Control> &controls);

/**
 * @brief Appends controls with a rs-turn
 * @param c :the circle of rs-turn
 * @param q :the configuration q
 * @param order :the order of control[true-> positive oder, false->negative oder]
 * @param controls :output the control commond
 */
void RS_TurnControls(HC_CC_Circle &c, const Configuration &q, bool order, vector<Control> &controls);

/**
 * @brief Appends controls with a hc-turn
 * @param c :the circle of hc-turn
 * @param q :the configuration q
 * @param order :the order of control[true-> positive oder, false->negative oder]
 * @param controls :output the control commond
 */
void HC_TurnControls(HC_CC_Circle &c, const Configuration &q, bool order, vector<Control> &controls);

/**
 * @brief Appends controls with a cc-turn
 * @param c :the circle of cc-turn
 * @param q :the configuration q
 * @param order :the order of control[true-> positive oder, false->negative oder]
 * @param controls :output the control commond
 */
void CC_TurnControls(HC_CC_Circle &c, const Configuration &q, bool order, vector<Control> &controls);

/**
 * @brief Appends controls with a default cc-turn consisting of two clothoids and a circular arc
 * @param c :the circle of cc-turn
 * @param q :the configuration q
 * @param delta :the angle betwen start configuration of circle and configuration q
 * @param order :the order of control[true-> positive oder, false->negative oder]
 * @param controls :output the control commond
 */
void CC_DefaultControls(HC_CC_Circle &c, const Configuration &q, double delta, bool order, vector<Control> &controls);

/**
 * @brief Appends controls with an elementary path if one exists
 * @param c :the circle of cc-turn
 * @param q :the configuration q
 * @param delta :delta :the angle betwen start configuration of circle and configuration q
 * @param order :the order of control[true-> positive oder, false->negative oder]
 * @param controls :output the control commond
 * @return if exist return true else return false
 */
bool CC_ElementaryControls(HC_CC_Circle &c, const Configuration &q, double delta, bool order, vector<Control> &controls);

}

#endif // STEERINGCOMMON_H
