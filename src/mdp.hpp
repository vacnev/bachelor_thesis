#include <vector>

template < typename action_t, typename state_t >
struct MDP
{
    virtual state_t initial_state() = 0;
    virtual std::vector<action_t> get_actions(state_t) = 0;

    // possible outcome states, probability vector
    virtual std::tuple<std::vector<state_t>, std::vector<double>> state_action(state_t, action_t) = 0;

    virtual int reward(state_t, action_t) = 0;
    virtual bool is_fail_state(state_t) = 0;
    virtual ~MDP() = 0;
};