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
    history his;
    double L_value; //lower bound on value function = L0
    double U_value; //upper bound U
    double l_rwdu; // lower bound on rwdu
    double u_rwdu; // upper bound on rwdu
    std::vector<std::unique_ptr<node>> children;
    std::vector<double> scenarios;

    /*node(state_t s, double L) : his(std::make_unique<history>(s)), L_value(L) {
        initialize_values();
    }*/
    
    node(std::unique_ptr<history>&& h, double L) : his(std::move(h)), L_value(L) {
        initialize_values();
    }

    // init U, l, u
    void initialize_values() {
        // TODO
    }
};
