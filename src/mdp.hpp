#include <vector>
#include <unordered_map>
#include "history.h"

template < typename action_t, typename state_t >
struct MDP
{
    virtual state_t initial_state() = 0;
    virtual std::vector<action_t> get_actions(state_t) = 0;

    // possible outcome states, probability vector
    virtual std::unordered_map<state_t, double> state_action(state_t, action_t) = 0;

    // treasure reward for action from treasure state
    virtual int reward(history<state_t, action_t>&, state_t, action_t) = 0;
    virtual int max_reward() = 0;
    virtual bool is_fail_state(state_t) = 0;

    // returns a reward if we are on the treasure
    virtual int is_treasure(state_t) = 0;
    virtual void take_treasure(state_t) = 0;

    virtual ~MDP() = 0;
};