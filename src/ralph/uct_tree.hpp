#include "../mdp.hpp"

template< typename state_t, typename action_t >
struct uct_tree
{
    MDP<state_t, action_t>* mdp;

    double c = 0.1; //exploration constant

    std::mt19937 generator{std::random_device{}()};

    // depth of random playouts
    size_t steps_default = 10;

    // number of ranfom playouts
    size_t default_playouts = 10;

    // discount factor
    double gamma = 0.95;

    struct node
    {
        uct_tree& tree;

        history<state_t, action_t> his;

        size_t N = 0; //number of visits

        double payoff; // cummulative payoff

        std::unordered_map<action_t, size_t> action_visits;

        std::unordered_map<action_t, double> action_payoff;

        node* parent;

        double r; // risk estimate

        double v; // payoff estimate

        int gold_count; // remaining gold

        std::unordered_map<action_t, std::vector<std::unique_ptr<node>>> children;
        // could be changed for pair action, state (better performance of simulate)

        node(uct_tree& t, history<state_t, action_t> h, node* p, double payoff, int g_c)
             : tree(t), his(h), payoff(payoff), parent(p), gold_count(g_c) {

            default_policy(); // sets r, v
        }

        // fail state nodes
        node(uct_tree& t, history<state_t, action_t> h, node* p, double payoff, double risk)
         : tree(t), his(h), payoff(payoff), parent(p), r(risk), v(0), gold_count(0) {}

        state_t& state() {
            return his.last();
        }

        bool leaf() {
            return children.empty();
        }

        double uct(action_t action) {

            /*std::cout << "print action payoff\n";
            for (auto [a, d] : action_payoff) {
                std::cout << "action: " << a << " payoff: " << d << '\n';
            }*/

            double Vmax = 0, Vmin = 0;
            if (!action_payoff.empty()) {
                Vmax = std::max_element(action_payoff.begin(), action_payoff.end(),
                            [](auto& l, auto& r){ return l.second < r.second; })->second;
                Vmin = std::min_element(action_payoff.begin(), action_payoff.end(),
                            [](auto& l, auto& r){ return l.second < r.second; })->second;
            }

            //std::cout << "uct after maxm in\n";
            
            return ((action_payoff[action] - Vmin) / (Vmax - Vmin)) +
                     tree.c * std::sqrt(std::log(N) / (action_visits[action] + 1));
        }

        // random playouts, sets r and v
        void default_policy() {

            v = 0;
            r = 0;

            for (size_t i = 0; i < tree.default_playouts; ++i) {
                history<state_t, action_t> h = his;
                auto [v_est, r_est] = default_rec(h, state(), 0);
                v += v_est;
                r += r_est;
            }

            v /= tree.default_playouts;
            r /= tree.default_playouts;
        }

        // payoff, risk
        std::pair<double, double> default_rec(history<state_t, action_t>& h, state_t curr, size_t step) {

            if (tree.mdp->is_fail_state(curr))
                return {0, 1};

            if (step == tree.steps_default)
                return {0, 0};

            // sample action
            std::vector<action_t> actions = tree.mdp->get_actions(curr);
            std::uniform_int_distribution<std::size_t> distr(0, actions.size() - 1);
            size_t a = distr(tree.generator);
            action_t action = actions[a];
            int rew = tree.mdp->reward(h, curr, action);

            // sample state
            auto state_distr = tree.mdp->state_action(curr, action);
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

            h.add(action, state);
            auto [value, risk] = default_rec(h, state, ++step);

            return {rew + tree.gamma * value, risk};
        }
    };

    std::unique_ptr<node> root;

    uct_tree(MDP<state_t, action_t>* mdp) : mdp(mdp) {

        root = std::make_unique<node>(*this, history<state_t, action_t>(mdp->initial_state()), nullptr, 0, mdp->gold_rem());
    }
    
    // one mcts iteration
    void simulate(size_t steps) {

        size_t depth = 0;
        node* curr = root.get();

        // find leaf, node selection
        while (!curr->leaf() && depth <= steps) {

            /*std::cout << "uct selection:";
            state_t tmp = curr->state();
            std::cout << '(' << tmp.first.first << ", " << tmp.first.second << ") " << tmp.second << '\n';*/

            // pick action according their uct values
            std::unordered_map<action_t, double> uct_act;
            
            for (auto it = curr->children.begin(); it != curr->children.end(); ++it) {
                //std::cout << "uct action: " << curr->uct(it->first) << '\n';
                uct_act[it->first] = curr->uct(it->first);
            }

            //std::cout << "after uct\n";

            action_t a_star = std::max_element(uct_act.begin(), uct_act.end(),
                              [](auto& l, auto& r) { return l.second < r.second; })->first;

            //std::cout << "a star selected\n";

            // sample state   // duplicate
            auto state_distr = mdp->state_action(curr->state(), a_star);
            std::uniform_real_distribution d;
            double num = d(generator);

            state_t s_star;
            for (auto [s, w] : state_distr) {

                //std::cout << "state select; num: " << num << " weight: " << w << " " << s_star.first.first << ", " << s_star.first.second << " " << s_star.second << '\n';

                if (num <= w) {
                    s_star = s;
                    break;
                }

                num -= w;
            }

            //std::cout << "s star selected \n";

            /*std::cout << "action: " << a_star << " state: " << s_star.first.first << ", " << s_star.first.second << " " << s_star.second << '\n';
            std::cout << "children " << curr->children.empty() << '\n';*/

            for (auto it = curr->children[a_star].begin(); it != curr->children[a_star].end(); ++it) {

                //std::cout << "finding state: " << (*it)->state().first.first << ", " << (*it)->state().first.second << " " << (*it)->state().second << '\n';

                if ((*it)->state() == s_star) {
                    curr = it->get();
                    break;
                }
            }

            ++depth;

            //std::cout << "node selection step\n";
        }

        //std::cout << "node selection done\n";

        // expansion
        if (depth < steps && !mdp->is_fail_state(curr->state()) && curr->gold_count > 0) {

            for (auto action : mdp->get_actions(curr->state())) {

                int child_gold_count = curr->gold_count;
                int rew = mdp->reward(curr->his, curr->state(), action);
                if (rew == mdp->gold_reward())
                    child_gold_count--;
                double payoff = curr->payoff + std::pow(gamma, curr->his.actions.size()) * rew;

                auto state_distr = mdp->state_action(curr->state(), action);
                for (auto state_prob : state_distr) {

                    history<state_t, action_t> new_h = curr->his;
                    new_h.add(action, state_prob.first);

                    if (mdp->is_fail_state(state_prob.first)) {
                        //std::cout << "expand failt state\n";
                        curr->children[action].emplace_back(
                            std::make_unique<node>(*this, std::move(new_h), curr, payoff, 1));
                    } else
                        curr->children[action].emplace_back(
                            std::make_unique<node>(*this, std::move(new_h), curr, payoff, child_gold_count));
                }
            }
        }

        //std::cout << "after expansion\n";

        // backpropagation
        double val = curr->v;
        curr->N++;
        node* parent = curr->parent;

        while (curr != root.get()) {

            //std::cout << "curr state: (" << curr->state().first.first << ", " << curr->state().first.second << ") " << curr->state().second << '\n';

            action_t& action = curr->his.last_action();
            //std::cout << "flag1\n";
            parent->N++;
            //std::cout << "flag2\n";
            parent->action_visits[action] += 1;
            //std::cout << parent->action_visits[action] << "flag3\n";
            val = mdp->reward(parent->his, parent->state(), action) + gamma * val;
            //std::cout << "flag4\n";
            parent->action_payoff[action] += (val - parent->action_payoff[action]) / parent->action_visits[action];
            //std::cout << parent->action_payoff[action] << "flag5\n";

            curr = parent;
            parent = curr->parent;
        }

        //std::cout << "simulate step\n";
    }
};