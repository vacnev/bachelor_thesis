#include<vector>

template< typename state_t, typename action_t >
struct history
{
    std::vector<state_t> states;
    std::vector<action_t> actions;

    history(state_t s) {
        states.emplace_back(std::move(s));
    }

    state_t& last() {
        return states.back();
    }
};

