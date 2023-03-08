#include<vector>

template < typename action_t, typename state_t >
struct MDP
{
    virtual state_t initial_state();
    virtual std::vector<action_t> get_actions(state_t);
    virtual std::tuple<std::vector<state_t>, std::vector<double>, float> state_action(state_t, action_t);
    virtual ~MDP() = 0;
};