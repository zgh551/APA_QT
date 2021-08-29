#include "hc_reeds_shepp_state_space.h"

/**
 * @brief HC00_ReedsSheppStateSpace Constructor
 * @param kappa :the max curvature of the path
 * @param sigma :the max sharpness of the path
 * @param discretization :the discretization step
 */
HC_ReedsSheppStateSpace::HC_ReedsSheppStateSpace(double kappa, double sigma, double discretization)
    : HC_CC_StateSpace(kappa, sigma, discretization)
    , _hc00_reeds_shepp_state_space(kappa, sigma, discretization)
    , _hc0pm_reeds_shepp_state_space(kappa, sigma, discretization)
    , _hcpm0_reeds_shepp_state_space(kappa, sigma, discretization)
    , _hcpmpm_reeds_shepp_state_space(kappa, sigma, discretization)
{

}

HC_ReedsSheppStateSpace::~HC_ReedsSheppStateSpace() = default;

/**
 * @brief Predicts a state forwards and backwards to zero and max. curvature
 * @param state :the configuration state
 * @return the predicts state and control
 */
vector<pair<State, Control>> HC_ReedsSheppStateSpace::PredictState(const State &state) const
{
    vector<pair<State, Control>> states_controls;

    // no prediction required
    if( (fabs(state.kappa) < math::getEpsilon()) ||
        ((_kappa - fabs(state.kappa)) < math::getEpsilon()))
    {
        pair<State, Control> state_control;
        state_control.first = state;
        state_control.second.delta_s = 0.0;
        state_control.second.kappa = state.kappa;
        state_control.second.sigma = 0.0;
        states_controls.push_back(state_control);
        return states_controls;
    }

    states_controls.reserve(4);
    double sgn_kappa = math::sgn(state.kappa);
    pair<State, Control> state_control1, state_control2, state_control3, state_control4;

    // assign controls
    state_control1.second.delta_s = (_kappa - sgn_kappa * state.kappa) / _sigma;
    state_control1.second.kappa = state.kappa;
    state_control1.second.sigma = sgn_kappa * _sigma;
    states_controls.push_back(state_control1);

    state_control2.second.delta_s = -state_control1.second.delta_s;
    state_control2.second.kappa = state.kappa;
    state_control2.second.sigma = state_control1.second.sigma;
    states_controls.push_back(state_control2);

    state_control3.second.delta_s = sgn_kappa * state.kappa / _sigma;
    state_control3.second.kappa = state.kappa;
    state_control3.second.sigma = -sgn_kappa * _sigma;
    states_controls.push_back(state_control3);

    state_control4.second.delta_s = -state_control3.second.delta_s;
    state_control4.second.kappa = state.kappa;
    state_control4.second.sigma = state_control3.second.sigma;
    states_controls.push_back(state_control4);

    // predict states with controls
    for (auto &state_control : states_controls)
    {
        double d = math::sgn(state_control.second.delta_s);
        double abs_delta_s = fabs(state_control.second.delta_s);
        double sigma = state_control.second.sigma;
        math::clothoid_to_end(state.x, state.y, state.psi, state.kappa,
                              sigma, d, abs_delta_s,
                              &state_control.first.x, &state_control.first.y,
                              &state_control.first.psi, &state_control.first.kappa);
    }
    return states_controls;
}

/**
 * @brief Returns shortest path length from state1 to state2
 * @param state1
 * @param state2
 * @return the shortest path length
 */
double HC_ReedsSheppStateSpace::getDistance(const State &state1, const State &state2) const
{
    vector<pair<State, Control>> start_states_controls = this->PredictState(state1);
    vector<pair<State, Control>> end_states_controls = this->PredictState(state2);
    vector<double> distances;

    distances.reserve(16);

    // compute the path length for all predicted start and end states
    for (const auto &start_state_control : start_states_controls)
    {
        State start_state = start_state_control.first;
        Control start_control = start_state_control.second;
        for (const auto &end_state_control : end_states_controls)
        {
            State end_state = end_state_control.first;
            Control end_control = end_state_control.second;

            // check if start and goal state are equal
            if (steering::StateEqual(start_state, end_state))
            {
                Control control = steering::SubtractControl(start_control, end_control);
                distances.push_back(fabs(control.delta_s));
            }
            else // call appropriate state space
            {
                double distance = 0.0;
                if (fabs(start_state.kappa) < math::getEpsilon())
                {
                    if ( fabs(end_state.kappa) < math::getEpsilon() )
                    {
                        distance += _hc00_reeds_shepp_state_space.getDistance(start_state, end_state);
                    }
                    else
                    {
                        distance += _hc0pm_reeds_shepp_state_space.getDistance(start_state, end_state);
                    }
                }
                else
                {
                    if ( fabs(end_state.kappa) < math::getEpsilon() )
                    {
                        distance += _hcpm0_reeds_shepp_state_space.getDistance(start_state, end_state);
                    }
                    else
                    {
                        distance += _hcpmpm_reeds_shepp_state_space.getDistance(start_state, end_state);
                    }
                }
                //adjust controls by intial and final control
                if ( fabs(start_control.delta_s) > math::getEpsilon() )
                {
                    distance += fabs(start_control.delta_s);
                }
                if ( fabs(end_control.delta_s) > math::getEpsilon() )
                {
                    distance += fabs(end_control.delta_s);
                }
                distances.push_back(distance);
            } //end if steering::StateEqual(start_state, end_state)
        } // end for end_state_control
    } // end for start_state_control
    return *min_element(distances.begin(), distances.end());
}

HC_CC_RS_Path* HC_ReedsSheppStateSpace::getCirclePath(const State &state1, const State &state2) const
{

    vector<pair<State, Control>> start_states_controls = this->PredictState(state1);
    vector<pair<State, Control>> end_states_controls   = this->PredictState(state2);

    vector<pair<HC_CC_RS_Path*, double>> hc_rs_path_distance_pairs;
    HC_CC_RS_Path *circle_path = nullptr;
    hc_rs_path_distance_pairs.reserve(16);

    // compute the path for all predicted start and end states
    for (const auto &start_state_control : start_states_controls)
    {
        State start_state = start_state_control.first;
        Control start_control = start_state_control.second;
        for (const auto &end_state_control : end_states_controls)
        {
            State end_state = end_state_control.first;
            Control end_control = end_state_control.second;
            vector<Control> hc_rs_control;
            // check if start and goal state are equal
            if (steering::StateEqual(start_state, end_state))
            {
                Control control = steering::SubtractControl(start_control, end_control);
                hc_rs_control.push_back(control);
            }
            else// call the appropriate state space
            {
                if(fabs(start_state.kappa) < math::getEpsilon())
                {
                    if (fabs(end_state.kappa) < math::getEpsilon())
                    {
                        hc_rs_control = _hc00_reeds_shepp_state_space.getControls(start_state, end_state);
                        circle_path = _hc00_reeds_shepp_state_space.HC00_ReedsSheppPath(start_state, end_state);
                    }
                    else
                    {
                        hc_rs_control = _hc0pm_reeds_shepp_state_space.getControls(start_state, end_state);
                        circle_path = _hc0pm_reeds_shepp_state_space.HC0PM_ReedsSheppPath(start_state, end_state);
                    }
                }
                else
                {
                    if (fabs(end_state.kappa) < math::getEpsilon())
                    {
                        hc_rs_control = _hcpm0_reeds_shepp_state_space.getControls(start_state, end_state);
                        circle_path = _hcpm0_reeds_shepp_state_space.HCPM0_ReedsSheppPath(start_state, end_state);
                    }
                    else
                    {
                        hc_rs_control = _hcpmpm_reeds_shepp_state_space.getControls(start_state, end_state);
                        circle_path = _hcpmpm_reeds_shepp_state_space.HCPMPM_ReedsSheppPath(start_state, end_state);
                    }
                }
                // adjust controls by intial and final control
                if(fabs(start_control.delta_s) < math::getEpsilon())
                {
                    hc_rs_control.insert(hc_rs_control.begin(), start_control);
                }
                if(fabs(end_control.delta_s) > math::getEpsilon())
                {
                    steering::ReverseControl(end_control);
                    hc_rs_control.insert(hc_rs_control.end(), end_control);
                }
            }
            // compute the path length
            double distance = 0.0;
            for (const auto &control : hc_rs_control)
            {
                distance += fabs(control.delta_s);
            }

            // push back
            pair<HC_CC_RS_Path*, double> hc_rs_controls_distance_pair;
            hc_rs_controls_distance_pair.first = circle_path;
            hc_rs_controls_distance_pair.second = distance;
            hc_rs_path_distance_pairs.push_back(hc_rs_controls_distance_pair);
        }
    }

    // sort the controls with respect to path length
    sort(hc_rs_path_distance_pairs.begin(), hc_rs_path_distance_pairs.end(),
         [](const pair<HC_CC_RS_Path*, double> &i, const pair<HC_CC_RS_Path*, double> &j)
    {
        return i.second < j.second;
    });
    return hc_rs_path_distance_pairs[0].first;
}
/**
 * @brief Returns controls of the shortest path from state1 to state2
 * @param state1
 * @param state2
 * @return the controls
 */
vector<Control> HC_ReedsSheppStateSpace::getControls(const State &state1, const State &state2) const
{
    vector<pair<State, Control>> start_states_controls = this->PredictState(state1);
    vector<pair<State, Control>> end_states_controls   = this->PredictState(state2);
    vector<pair<vector<Control>, double>> hc_rs_controls_distance_pairs;
    hc_rs_controls_distance_pairs.reserve(16);

    // compute the path for all predicted start and end states
    for (const auto &start_state_control : start_states_controls)
    {
        State start_state = start_state_control.first;
        Control start_control = start_state_control.second;
        for (const auto &end_state_control : end_states_controls)
        {
            State end_state = end_state_control.first;
            Control end_control = end_state_control.second;
            vector<Control> hc_rs_control;
            // check if start and goal state are equal
            if (steering::StateEqual(start_state, end_state))
            {
                Control control = steering::SubtractControl(start_control, end_control);
                hc_rs_control.push_back(control);
            }
            else// call the appropriate state space
            {
                if(fabs(start_state.kappa) < math::getEpsilon())
                {
                    if (fabs(end_state.kappa) < math::getEpsilon())
                    {
                        hc_rs_control = _hc00_reeds_shepp_state_space.getControls(start_state, end_state);
                    }
                    else
                    {
                        hc_rs_control = _hc0pm_reeds_shepp_state_space.getControls(start_state, end_state);
                    }
                }
                else
                {
                    if (fabs(end_state.kappa) < math::getEpsilon())
                    {
                        hc_rs_control = _hcpm0_reeds_shepp_state_space.getControls(start_state, end_state);
                    }
                    else
                    {
                        hc_rs_control = _hcpmpm_reeds_shepp_state_space.getControls(start_state, end_state);
                    }
                }
                // adjust controls by intial and final control
                if(fabs(start_control.delta_s) > math::getEpsilon())
                {
                    hc_rs_control.insert(hc_rs_control.begin(), start_control);
                }
                if(fabs(end_control.delta_s) > math::getEpsilon())
                {
                    steering::ReverseControl(end_control);
                    hc_rs_control.insert(hc_rs_control.end(), end_control);
                }
            }
            // compute the path length
            double distance = 0.0;
            for (const auto &control : hc_rs_control)
            {
                distance += fabs(control.delta_s);
            }

            // push back
            pair<vector<Control>, double> hc_rs_controls_distance_pair;
            hc_rs_controls_distance_pair.first = hc_rs_control;
            hc_rs_controls_distance_pair.second = distance;
            hc_rs_controls_distance_pairs.push_back(hc_rs_controls_distance_pair);
        }
    }

    // sort the controls with respect to path length
    sort(hc_rs_controls_distance_pairs.begin(), hc_rs_controls_distance_pairs.end(),
         [](const pair<vector<Control>, double> &i, const pair<vector<Control>, double> &j)
    {
        return i.second < j.second;
    });
    return hc_rs_controls_distance_pairs[0].first;
}
