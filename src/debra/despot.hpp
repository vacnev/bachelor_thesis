#include "../mdp.hpp"

// if fail state kills us with prob 1, then there is no point in exploring such states
// if not, despot can be modified to continue exploration (default_policy, explore)

using scenario = std::vector<double>;

template< typename state_t, typename action_t >
struct despot
{
    // TODO: set intial params

    MDP<state_t, action_t>* mdp;

    // target gap
    double eta0 = 0.1;

    // target rate
    double eta_rate = 0.95;

    // number of sampled scenarios
    size_t K = 500;

    // max depth
    size_t D;

    // regularization const
    double lambda = 0.1;

    // steps of default policy
    size_t D_default = 10;

    // discount parameter
    double gamma = 0.95;

    // maximum planning time per step
    int T_max = 3;

    std::mt19937 generator{std::random_device{}()};

    struct node
    {
        despot& tree;

        history<state_t, action_t> his;
        double L_value; //lower bound on value function = L0
        double U_value; //upper bound U
        double l_rwdu; // lower bound on rwdu
        double u_rwdu; // upper bound on rwdu
        std::unordered_map<action_t, std::vector<std::unique_ptr<node>>> children;
        std::vector<scenario> scenarios;
        size_t depth;
        double risk; // risk estimate
        node* parent;
        double payoff; // cumulative payoff
        
        node(despot& tree, history<state_t, action_t> h, std::vector<std::vector<double>>&& scenarios,
             node* parent = nullptr, size_t depth = 0, double payoff = 0)
             : tree(tree), his(h), scenarios(std::move(scenarios)), parent(parent), depth(depth), payoff(payoff) {
            initialize_values();
        }

        // fail state node
        node(despot& tree, history<state_t, action_t> h, node* parent, size_t depth, double payoff)
             : tree(tree), his(h), parent(parent), depth(depth), payoff(payoff),
               risk(1), U_value(0), L_value(0), l_rwdu(0), u_rwdu(0) {}

        state_t& state() {
            return his.last();
        }

        // init U, l, u
        void initialize_values() {
            default_policy();

            U_value = tree.mdp->max_reward() / (1 - tree.gamma);
            l_rwdu = (scenarios.size() / (double) tree.K) * std::pow(tree.gamma, depth) * L_value;
            u_rwdu = (scenarios.size() / (double) tree.K) * std::pow(tree.gamma, depth) * U_value - tree.lambda;

            if (u_rwdu < l_rwdu)
                u_rwdu = l_rwdu;
        }

        // random playouts, sets L0 and risk
        void default_policy() {
            double L = 0;
            double r = 0;

            for (size_t i = 0; i < scenarios.size(); i++) {
                history<state_t, action_t> h = his;
                auto result = default_policy_rec(0, h, scenarios[i]);
                L += result.first;
                r += result.second;
            }
            
            L_value = L / scenarios.size();
            risk = r / scenarios.size();
        }

        // payoff, risk
        std::pair<double, double> default_policy_rec(size_t step, history<state_t, action_t>& h, std::vector<double>& scenar) {
            
            if (tree.mdp->is_fail_state(h.last()))
                return {0, 1};

            if (step == tree.D_default)
                return {0, 0};

            std::vector<action_t> actions = tree.mdp->get_actions(h.last());
            std::uniform_int_distribution<std::size_t> distr(0, actions.size() - 1);
            size_t a = distr(tree.generator);
            action_t action = actions[a];
            
            // state distr
            auto state_distr = tree.mdp->state_action(h.last(), action);
            state_t next = tree.sample_state(state_distr, scenar[tree.D + step]);
            int rew = tree.mdp->reward(h, h.last(), action);
            h.add(action, next);

            auto child = default_policy_rec(step + 1, h, scenar);

            return {rew + tree.gamma * child.first, child.second};
        }

        double eta() {
            return u_rwdu - l_rwdu;
        }

        void make_default() {
            U_value = L_value;
            l_rwdu = (scenarios.size() / (double) tree.K) * std::pow(tree.gamma, depth) * L_value;
            u_rwdu = l_rwdu;
        }

        bool leaf() {
            return children.empty();
        }
    };

    // init node
    std::unique_ptr<node> n0;

    despot(MDP<state_t, action_t>* mdp, size_t depth, history<state_t, action_t> h0) : mdp(mdp), D(depth) {
        
        std::vector<scenario> scenarios = sample_scenarios();
        n0 = std::make_unique<node>(*this, h0, std::move(scenarios));
        build_despot();
    }

    void build_despot() {

        // start time
        auto start = std::chrono::steady_clock::now();

        // until we reach desired gap or time
        while (n0->eta() > eta0 && 
               std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < T_max) {

            node* n = explore(n0.get());
            backup(n);
        }
    }

    node* explore(node* n) {

        while (n->depth <= D && excess_uncertainty(n) > 0 && !prune(n)) {

            // leaf node - expansion
            if (n->leaf()) {
                
                std::vector<action_t> actions = mdp->get_actions(n->state());

                for (size_t i = 0; i < actions.size(); i++) {

                    // distribute scenarios
                    std::map<state_t, std::vector<scenario>> distr;

                    action_t action = actions[i];
                    auto states_distr = mdp->state_action(n->state(), action);
                    double payoff = n->payoff + std::pow(gamma, n->depth) * mdp->reward(n->his, n->state(), action);

                    for (size_t j = 0; j < n->scenarios.size(); j++) {
                        double s = n->scenarios[j][n->depth];

                        state_t state = sample_state(states_distr, s);

                        distr[state].push_back(n->scenarios[j]);
                    }

                    for (auto it = distr.begin(); it != distr.end(); it++) {

                        history<state_t, action_t> new_h = n->his;
                        new_h.add(action, it->first);

                        std::unique_ptr<node> new_node;

                        if (mdp->is_fail_state(it->first)) {
                            new_node = std::make_unique<node>(*this, new_h, n, n->depth + 1, payoff);
                        } else {
                            new_node = std::make_unique<node>(*this, new_h, std::move(it->second), n, n->depth + 1, payoff);
                        }

                        n->children[action].push_back(std::move(new_node));
                    }
                }
            }

            // a*

            auto comp = [&](auto& a, auto& b) {
                double coef = (n->scenarios.size() / (double) K) * std::pow(gamma, n->depth);

                double left = std::accumulate(a.second.begin(), a.second.end(), 0, [](auto& c, auto&r) {return c + r->u_rwdu;});
                left += coef * mdp->reward(n->his, n->state(), a.first) - lambda;

                double right = std::accumulate(b.second.begin(), b.second.end(), 0, [](auto& c, auto&r) {return c + r->u_rwdu;});
                right += coef * mdp->reward(n->his, n->state(), b.first) - lambda;

                return left < right;
            };

            action_t a_star = std::max_element(n->children.begin(), n->children.end(), comp)->first;

            // s* -> next node

            n = std::max_element(n->children[a_star].begin(), n->children[a_star].end(),
                [this](auto& a, auto&b) {return excess_uncertainty(a.get()) < excess_uncertainty(b.get());})->get();

            
            if (n->depth > D)
                n->make_default();
        }

        return n;
    }

    double excess_uncertainty(node* n) {
        return n->eta() + (n->scenarios.size() / (double) K) * eta_rate * n0->eta();
    }

    bool prune(node* n) {
        bool blocked = false;

        while (n->parent) {

            bool pruned = false;
            node* parent = n->parent;

            while (parent) {
                
                if ((parent->scenarios.size() / (double) K) * std::pow(gamma, parent->depth) * (parent->U_value - parent->L_value)
                    <= lambda * (n->depth - parent->depth + 1)) {
                        n->make_default();
                        backup(n);
                        blocked = true;
                        pruned = true;
                        break;
                    }

                parent = parent->parent;

            }

            if (!pruned)
                break;

            n = n->parent;
        }

        return blocked;
    }

    void backup(node* n) {

        n = n->parent;

        while (n) {

            std::vector<double> U_argmax;
            std::vector<double> u_argmax;
            std::vector<double> l_argmax;

            for (auto it = n->children.begin(); it != n->children.end(); it++) {

                auto& [action, children] = *it;

                double sum_U = 0, sum_u = 0, sum_l = 0;

                for (size_t j = 0; j < children.size(); j++) {
                    node* child = children[j].get();

                    sum_U += ((double) child->scenarios.size() / n->scenarios.size()) * child->U_value;
                    sum_u += child->u_rwdu;
                    sum_l += child->l_rwdu;
                }

                int rew = mdp->reward(n->his, n->state(), action);
                U_argmax.emplace_back(rew + gamma * sum_U);

                double w_rew = (n->scenarios.size() / (double) K) * std::pow(gamma, n->depth) * rew - lambda;
                u_argmax.emplace_back(w_rew + sum_u);
                l_argmax.emplace_back(w_rew + sum_l);
            }

            double l0 = (n->scenarios.size() / (double) K) * std::pow(gamma, n->depth) * n->L_value;
            u_argmax.push_back(l0);
            l_argmax.push_back(l0);

            n->U_value = *std::max_element(U_argmax.begin(), U_argmax.end());
            n->u_rwdu = *std::max_element(u_argmax.begin(), u_argmax.end());
            n->l_rwdu = *std::max_element(l_argmax.begin(), l_argmax.end());

            n = n->parent; // move up
        }
    }

    std::vector<std::vector<double>> sample_scenarios() {
        std::vector<scenario> scenarios;
        std::uniform_real_distribution d;
        //std::srand( (unsigned) time(NULL) );

        for(size_t i = 0; i < K; i++) {
            scenario scenar;

            for (size_t j = 0; j < D + D_default; j++) {
                //scenar.emplace_back((double) std::rand() / RAND_MAX);
                scenar.emplace_back(d(generator));
            }
            
            scenarios.push_back(std::move(scenar));
        }

        return scenarios;
    }

    // sample state using a scenar
    state_t sample_state(std::map<state_t, double>& states_distr, double scenar) {

        for (auto it = states_distr.begin(); it != states_distr.end(); ++it) {
            if (scenar <= it->second)
                return it->first;

            scenar -= it->second;
        }

        assert(false);

        return {};
    }


};