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

    int depth() {
        return actions.size()
    }
};

template< typename state_t, typename action_t >
struct node
{
    std::unique_ptr<history> his;

};
