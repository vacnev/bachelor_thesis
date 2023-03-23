#include "history.hpp"
#include "mdp.hpp"
#include <unordered_map>
#include <memory>
#include <vector>
#include <cmath>


template< typename state_t, typename action_t >
struct uct_tree
{
    MDP mdp;

    double c; //exploration constant

    struct node
    {
        uct_tree& tree;

        history<state_t, action_t> his;

        size_t N = 1; //number of visits

        double r; // risk estimate

        double v; // payoff estimate

        std::unordered_map<action_t, size_t> action_visits;

        std::unordered_map<action_t, double> action_payoff;

        node* parent;

        std::unordered_map<action_t, std::vector<std::unique_ptr<node>>> children;

        node(uct_tree& t, history<state_t, action_t> h, node* p) : tree(t), his(h), parent(p) {

            default_policy(); // sets r, v
        }

        state_t& state() {
            return his.last();
        }

        double uct(action_t action) {
            double Vmax = std::max_element(action_payoff.begin(), action_payoff.end(),
                            [](auto l, auto r){ return l.second < r.second; })->second;
            double Vmin = std::min_element(action_payoff.begin(), action_payoff.end(),
                            [](auto l, auto r){ return l.second < r.second; })->second;
            
            return ((action_visits[action] - Vmin) / (Vmax - Vmin)) +
                     tree.c * std::sqrt(std::log(N) / (action_visits[action] + 1));
        }
    };
};