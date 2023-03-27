#ifndef MDP_HPP
#define MDP_HPP

#include "history.hpp"

#include <map>
#include <fstream>
#include <iostream>
#include <string>
#include <cassert>
#include <algorithm>
#include <unordered_map>
#include <cmath>
#include <chrono>
#include <numeric>

template < typename state_t, typename action_t >
struct MDP
{
    virtual state_t initial_state() = 0;
    virtual std::vector<action_t> get_actions(state_t&) = 0;

    // possible outcome states, probability vector
    virtual std::map<state_t, double> state_action(state_t&, action_t&) = 0;

    // treasure reward for action from treasure state // action ommited since reward doesnt depend on it
    virtual int reward(history<state_t, action_t>&, state_t&) = 0;
    virtual int max_reward() = 0;
    virtual bool is_fail_state(state_t&) = 0;

    // take treasure in real step
    virtual void take_gold(state_t&) = 0;

    // write history for evaluation
    virtual void write_history(std::ofstream&, history<state_t, action_t>&) = 0;

    virtual ~MDP() = 0;
};

#endif