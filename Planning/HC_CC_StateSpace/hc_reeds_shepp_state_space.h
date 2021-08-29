#ifndef HC_REEDSSHEPPSTATESPACE_H
#define HC_REEDSSHEPPSTATESPACE_H

#include <algorithm>
#include <limits>
#include <memory>
#include <vector>

#include "Planning/Interface/hc_cc_state_space.h"
#include "Planning/Path/hc_cc_circle.h"
#include "Planning/Path/hc_cc_rs_path.h"
#include "Common/Configure/Configs/system_config.h"
#include "Planning/HC_CC_StateSpace/hc00_reeds_shepp_state_space.h"
#include "Planning/HC_CC_StateSpace/hc0pm_reeds_shepp_state_space.h"
#include "Planning/HC_CC_StateSpace/hcpm0_reeds_shepp_state_space.h"
#include "Planning/HC_CC_StateSpace/hcpmpm_reeds_shepp_state_space.h"

//#include "ompl/base/State.h"

using namespace std;
using namespace steering;
//namespace ob = ompl::base;

class HC_ReedsSheppStateSpace : public HC_CC_StateSpace
{
public:
    /**
     * @brief HC00_ReedsSheppStateSpace Constructor
     * @param kappa :the max curvature of the path
     * @param sigma :the max sharpness of the path
     * @param discretization :the discretization step
     */
    HC_ReedsSheppStateSpace(double kappa = 0.2, double sigma = 0.2, double discretization = 0.02);

    /**
     * @brief Destructor
     */
    virtual ~HC_ReedsSheppStateSpace() override;

    /**
     * @brief Predicts a state forwards and backwards to zero and max. curvature
     * @param state :the configuration state
     * @return the predicts state and control
     */
    vector<pair<State, Control>> PredictState(const State &state) const;

    /**
     * @brief Returns shortest path length from state1 to state2
     * @param state1
     * @param state2
     * @return the shortest path length
     */
    double getDistance(const State &state1, const State &state2) const override;

//    double distance(const ob::State *state1, const ob::State *state2) const;


    HC_CC_RS_Path* getCirclePath(const State &state1, const State &state2) const;

    /**
     * @brief Returns controls of the shortest path from state1 to state2
     * @param state1
     * @param state2
     * @return the controls
     */
    vector<Control> getControls(const State &state1, const State &state2) const override;
private:
    HC00_ReedsSheppStateSpace   _hc00_reeds_shepp_state_space;
    HC0PM_ReedsSheppStateSpace  _hc0pm_reeds_shepp_state_space;
    HCPM0_ReedsSheppStateSpace  _hcpm0_reeds_shepp_state_space;
    HCPMPM_ReedsSheppStateSpace _hcpmpm_reeds_shepp_state_space;
};

#endif // HC_REEDSSHEPPSTATESPACE_H
