#ifndef HCPMPM_REEDSSHEPPSTATESPACE_H
#define HCPMPM_REEDSSHEPPSTATESPACE_H

#include <limits>
#include <memory>
#include <vector>

#include "Planning/Interface/hc_cc_state_space.h"
#include "Planning/Path/hc_cc_circle.h"
#include "Planning/Path/hc_cc_rs_path.h"

#include "Common/Configure/Configs/system_config.h"

using namespace std;
using namespace steering;
using namespace hc_cc_rs;

#define HC_REGULAR false
#define CC_REGULAR false

class HCPMPM_ReedsSheppStateSpace : public HC_CC_StateSpace
{
public:
    /**
     * @brief HC00_ReedsSheppStateSpace Constructor
     * @param kappa :the max curvature of the path
     * @param sigma :the max sharpness of the path
     * @param discretization :the discretization step
     */
    HCPMPM_ReedsSheppStateSpace(double kappa, double sigma, double discretization = 0.1);

    /**
     * @brief Destructor
     */
    virtual ~HCPMPM_ReedsSheppStateSpace() override;

    /**
     * @brief Returns a sequence of turns and straight lines connecting the two circles c1 and c2
     * @param c1 :start circle
     * @param c2 :end circle
     * @return a sequence of turns and straight line
     */
    HC_CC_RS_Path* HCPMPM_CirclesReedsSheppPath(HC_CC_Circle &c1, HC_CC_Circle &c2) const;

    /**
     * @brief Returns a sequence of turns and straight lines connecting a start and an end configuration
     * @param state1 :the start state
     * @param state2 :the end state
     * @return a sequence of turns and straight lines
     */
    HC_CC_RS_Path* HCPMPM_ReedsSheppPath(const State &state1, const State &state2) const;

    /**
     * @brief Returns shortest path length from state1 to state2
     * @param state1 :the start state
     * @param state2 :the end state
     * @return
     */
    double getDistance(const State &state1, const State &state2) const;

    /**
     * @brief Returns controls of the shortest path from state1 to state2
     * @param state1 :the start state
     * @param state2 :the end state
     * @return
     */
    vector<Control> getControls(const State &state1, const State &state2) const override;
private:
    /**
     * @brief class that contains functions to compute the families
     */
    class HCPMPM_ReedsShepp;

    /**
     * @brief unique pointer on class with families
     */
    unique_ptr<HCPMPM_ReedsShepp> _hcpmpm_reeds_shepp;

    /**
     * @brief Param of a rs-circle
     */
    HC_CC_Circle_Param _rs_circle_param;

    /**
     * @brief Outer radius of a hc-/cc-circle
     */
    double _radius;

    /**
     * @brief Angle between a configuration on the hc-/cc-circle and the tangent to the circle at that position
     */
    double _mu;

    /**
     * @brief Sine and cosine of mu
     */
    double _sin_mu, _cos_mu;
};

#endif // HCPMPM_REEDSSHEPPSTATESPACE_H
