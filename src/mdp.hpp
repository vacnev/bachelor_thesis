#include<vector>

template < typename action_t, typename state_t >
struct MDP
{
    virtual state_t initial_state() = 0;
    virtual std::vector<action_t> get_actions(state_t) = 0;
    virtual std::tuple<std::vector<state_t>, std::vector<double>, float> state_action(state_t, action_t) = 0;
    virtual ~MDP() = 0;
};