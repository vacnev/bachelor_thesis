#include <vector>
#include <memory>

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

    action_t& last_action() {
        return actions.back();
    }

    int depth() {
        return actions.size();
    }

    void add(action_t a, state_t s) {
        states.push_back(s);
        actions.push_back(a);
    }
};


