#ifndef HC_CC_STATESPACE_H
#define HC_CC_STATESPACE_H

#include <vector>
#include <cmath>

#include "Planning/Common/steering_common.h"
#include "Planning/Path/hc_cc_circle.h"
#include "Planning/Path/path_line.h"
//#include "Planning/Interface/sh2_state_space.h"
#include "Common/Math/fresnel.h"
#include "Common/Math/math_utils.h"

//#include "ompl/base/SpaceInformation.h"
//#include "ompl/base/StateValidityChecker.h"
//#include "ompl/base/spaces/SE2StateSpace.h"
//#include "ompl/base/MotionValidator.h"

using namespace std;
using namespace steering;
//namespace ob = ompl::base;

class HC_CC_StateSpace
{
public:
    /**
     * @brief HC_CC_StateSpace :Constructor
     * @param kappa : the curvature of the circle
     * @param sigma : the sharpness of the clothoid
     * @param discretization : the coefficient of discretization
     */
    HC_CC_StateSpace(double kappa, double sigma, double discretization);

    virtual ~HC_CC_StateSpace();

    /**
     * @brief getControls :Virtual function that return controls of the shortest path from stat1 to state2
     * @param state1 :the start state
     * @param state2 :the end state
     * @return the controls
     */
    virtual vector<Control> getControls(const State &state1, const State &state2) const = 0;

    /**
     * @brief Returns shortest path length from state1 to state2
     * @param state1
     * @param state2
     * @return the shortest path length
     */
    virtual double getDistance(const State &state1, const State &state2) const = 0;

    /**
     * @brief get the path from state1 to state2
     * @param state1 :start state
     * @param state2 :end state
     * @return return the path from state1 to state2
     */
    vector<State> getPath(const State &state1, const State &state2) const;

    /**
     * @brief Computation of the integrate states given a start state and controls
     * @param state : the start state
     * @param controls :controls commond
     * @return return integrated states given a start state and controls
     */
    vector<State> integrate(const State &state, const vector<Control> &controls) const;

    /**
     * @brief Computation the interpolate state at distance t in [0, 1](percentage if total path lenght)
     * @param state :the start state
     * @param controls :the control commond
     * @param t :the distance
     * @return return the interpolate state at distance t
     */
    State interpolate(const State &state, const vector<Control> &controls, double t) const;

    /**
     * @brief Computation of integrated state given a start state, a control and a integration step
     * @param state :start state
     * @param control :control commond
     * @param integration_step :integration step
     * @return return the integrated state
     */
    inline State integrate_ODE(const State &state, const Control &control, double integration_step) const;


//    double distance(const ob::State *state1, const ob::State *state2) const override;

//    void interpolate(const ob::State *from, const ob::State *to, double t, ob::State *state) const override;

protected:

    /**
     * @brief _kappa :the curvature of clothoid
     */
    double _kappa;

    /**
     * @brief _sigma :the sharpness of clothoid
     */
    double _sigma;

    /**
     * @brief _discretization :the coefficient of the discretization path
     */
    double _discretization;

    /**
     * @brief the parameters of a hc/cc circle
     */
    HC_CC_Circle_Param _hc_cc_circle_param;
};


/** \brief A Reeds-Shepp motion validator that only uses the state validity checker.
    Motions are checked for validity at a specified resolution.

    This motion validator is almost identical to the DiscreteMotionValidator
    except that it remembers the optimal ReedsSheppPath between different calls to
    interpolate. */
//class HC_CC_ReedsSheppMotionValidator : public ob::MotionValidator
//{
//public:
//    HC_CC_ReedsSheppMotionValidator(ob::SpaceInformation *si) : MotionValidator(si)
//    {
//        defaultSettings();
//    }
//    HC_CC_ReedsSheppMotionValidator(const ob::SpaceInformationPtr &si) : MotionValidator(si)
//    {
//        defaultSettings();
//    }
//    ~HC_CC_ReedsSheppMotionValidator() override = default;
//    bool checkMotion(const ob::State *s1, const ob::State *s2) const override;
//    bool checkMotion(const ob::State *s1, const ob::State *s2, std::pair<ob::State *, double> &lastValid) const override;

//private:
//    HC_CC_StateSpace *stateSpace_;
//    void defaultSettings();
//};
#endif // HC_CC_STATESPACE_H
