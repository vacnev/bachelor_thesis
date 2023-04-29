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
#include <random>
#include <iterator>

template < typename state_t, typename action_t >
struct MDP
{
    virtual state_t initial_state() = 0;
    virtual std::vector<action_t> get_actions(const state_t&) = 0;

    // possible outcome states, probability vector
    virtual std::map<state_t, double> state_action(const state_t&, const action_t&) = 0;

    // treasure reward for action from treasure state // action ommited since reward doesnt depend on it
    virtual int reward(history<state_t, action_t>&, const state_t&, const action_t&) = 0;
    virtual int max_payoff() = 0;
    virtual bool is_fail_state(const state_t&) = 0;

    // take treasure in real step
    virtual bool take_gold(const state_t&) = 0;
    virtual int gold_rem() = 0;
    virtual int gold_reward() = 0;

    // write history for evaluation
    virtual void write_history(std::ofstream&, history<state_t, action_t>&) = 0;

    // init plan for next episode
    virtual void init_plan() = 0;

    virtual ~MDP() = default;
};

#endif