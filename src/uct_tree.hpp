#include "mdp.hpp"

template< typename state_t, typename action_t >
struct uct_tree
{
    MDP<state_t, action_t>* mdp;

    double c = 0.1; //exploration constant

    std::mt19937 generator{std::random_device{}()};

    // depth of random playouts
    size_t steps_default = 10;

    // discount factor
    double gamma = 0.9;

    struct node
    {
        uct_tree& tree;

        history<state_t, action_t> his;

        size_t N = 0; //number of visits

        double r; // risk estimate

        double v; // payoff estimate

        int payoff; // cummulative payoff

        std::unordered_map<action_t, size_t> action_visits;

        std::unordered_map<action_t, double> action_payoff;

        node* parent;

        std::unordered_map<action_t, std::vector<std::unique_ptr<node>>> children;
        // could be changed for pair action, state (better performance of simulate)

        node(uct_tree& t, history<state_t, action_t> h, node* p, int payoff = 0)
             : tree(t), his(h), parent(p), payoff(payoff) {

            default_policy(); // sets r, v
        }

        // fail state nodes
        node(uct_tree& t, history<state_t, action_t> h, node* p, double risk, int payoff = 0)
         : tree(t), his(h), parent(p), r(risk), v(0), payoff(payoff) {}

        state_t& state() {
            return his.last();
        }

        bool leaf() {
            return children.empty();
        }

        double uct(action_t action) {
            double Vmax = std::max_element(action_payoff.begin(), action_payoff.end(),
                            [](auto& l, auto& r){ return l.second < r.second; })->second;
            double Vmin = std::min_element(action_payoff.begin(), action_payoff.end(),
                            [](auto& l, auto& r){ return l.second < r.second; })->second;
            
            return ((action_payoff[action] - Vmin) / (Vmax - Vmin)) +
                     tree.c * std::sqrt(std::log(N) / (action_visits[action] + 1));
        }

        // random playouts, sets r and v
        void default_policy() {

            auto [v_est, r_est] = default_rec(state(), 0);

            v = v_est;
            r = r_est;
        }

        // payoff, risk
        std::pair<double, double> default_rec(state_t curr, size_t step) {

            if (tree.mdp.is_fail_state(curr))
                return {0, 1};

            if (step == tree.steps_default)
                return {0, 0};

            // sample action
            std::vector<action_t> actions = tree.mdp.get_actions(curr);
            std::uniform_int_distribution<std::size_t> distr(0, actions.size() - 1);
            size_t a = distr(tree.generator);
            action_t action = actions[a];
            int rew = tree.mdp.reward(his, curr, action);

            // sample state
            auto state_distr = tree.mdp.state_action(curr, action);
            std::uniform_real_distribution d;
            double num = d(tree.generator);

            state_t state;
            for (auto [s, w] : state_distr) {

                if (num <= w) {
                    state = s;
                    break;
                }

                num -= w;
            }

            auto [value, risk] = default_rec(state, ++step);

            return {rew + tree.gamma * value, risk};
        }
    };

    std::unique_ptr<node> root;

    uct_tree(MDP<state_t, action_t>* mdp) : mdp(mdp), root(new node(*this, {mdp.initial_state()}), NULL) {}
    
    // one mcts iteration
    void simulate(size_t steps) {

        size_t depth = 0;
        node* curr = root.get();

        // find leaf
        while (!curr->leaf()) {

            // pick action according their uct values
            std::unordered_map<action_t, double> uct_act;

            for (auto [action, _] : curr->children) {

                uct_act[action] = curr->uct(action);
            }

            action_t a_star = std::max_element(uct_act.begin(), uct_act.end(),
                              [](auto& l, auto& r) { return l.second < r.second; })->first;

            // sample state   // duplicate
            auto state_distr = mdp.state_action(curr->state(), a_star);
            std::uniform_real_distribution d;
            double num = d(generator);

            state_t s_star;
            for (auto [s, w] : state_distr) {

                if (num <= w) {
                    s_star = s;
                    break;
                }

                num -= w;
            }

            for (auto it = curr->children[a_star].begin(); it != curr->children[a_star].end(); ++it) {

                if (*it->state() == s_star) {
                    curr = it->get();
                    break;
                }
            }

            ++depth;
        }

        if (depth < steps && !mdp.is_fail_state(curr->state())) {

            for (auto action : mdp.get_actions(curr->state())) {
                int rew = mdp.reward(curr->his, curr->state(), action);
                int payoff = curr->payoff + rew;

                for (auto& state_distr : mdp.state_action(curr->state(), action)) {
                    for (auto [state, _] : state_distr) {

                        history<state_t, action_t> new_h = curr->his;
                        new_h.add(action, state);

                        if (mdp.is_fail_state(state))
                            curr->children[action].emplace_back(
                                std::make_unique<node>(*this, std::move(new_h), curr, 1, payoff));
                        else
                            curr->children[action].emplace_back(
                                std::make_unique<node>(*this, std::move(new_h), curr, payoff));
                    }
                }
            }
        }

        // backpropagation
        double val = curr->v;
        curr->N++;
        node* parent;

        while ((parent = curr->parent)) {

            action_t& action = curr->his.last_action();
            parent->N++;
            parent->action_visits[action]++;
            val = mdp.reward(parent->his, parent->state(), action) + gamma * val;
            parent->action_payoff[action] += (val - parent->action_payoff[action]) / parent->action_visits[action];

            curr = parent;
        }
    }
};