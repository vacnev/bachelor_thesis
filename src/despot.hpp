#include <mdp.hpp>
#include <history.hpp>
#include <time.h>
#include <cassert>
#include <cmath>
#include <random>
#include <algorithm>
#include <unordered_map>

// if fail state kills us with prob 1, then there is no point in exploring such states
// if not, despot can be modified to continue exploration (default_policy, explore)


template< typename action_t, typename state_t >
struct despot
{
    // TODO: set intial params

    MDP mdp;

    // init node
    std::unique_ptr<node> n0;

    // target gap
    double eta0 = 0.01;

    // target rate
    double eta_rate = 0.95;

    // number of sampled scenarios
    size_t K = 500;

    // max depth
    size_t D = 100;

    // regularization const
    double lambda = 1;

    // steps of default policy
    size_t D_default = 10;

    // discount parameter
    double gamma = 0.95;

    // maximum planning time per step
    // T_max TODO

    //template< typename state_t, typename action_t >
    struct node
    {
        history his;
        double L_value; //lower bound on value function = L0
        double U_value; //upper bound U
        double l_rwdu; // lower bound on rwdu
        double u_rwdu; // upper bound on rwdu
        std::unordered_map<action_t, std::vector<std::unique_ptr<node>>> children;
        std::vector<std::vector<double>> scenarios;
        size_t depth;
        double risk; // risk estimate
        node* parent;
        
        node(history h, std::vector<std::vector<double>>&& scenarios, node* parent = NULL, size_t depth = 0)
             : his(h), scenarios(std::move(scenarios)), parent(parent), depth(depth) {
            initialize_values();
        }

        state_t& state() {
            return his.last();
        }

        // init U, l, u
        void initialize_values() {
            default_policy();

            U_value = mdp.max_reward() / (1 - gamma);
            l_rwdu = (scenarios.size() / (double) K) * std::pow(gamma, depth) * L_value;
            u_rwdu = (scenarios.size() / (double) K) * std::pow(gamma, depth) * U_value - lambda;

            if (u_rwdu < l_rwdu)
                u_rwdu = l_rwdu;
        }

        // random playouts, sets L0 and risk
        void default_policy() {
            std::mt19937 generator(std::random_device{}());
            double L = 0;
            double r = 0;
            state_t curr = his.last();

            for (size_t i = 0; i < scenarios.size(); i++) {
                auto result = default_policy_rec(0, curr, generator, scenarios[i]);
                L += result.first;
                r += result.second;
            }
            
            L_value = L / scenarios.size();
            risk = r / scenarios.size();
        }

        // payoff, risk
        std::pair<double, double> default_policy_rec(size_t step, state_t curr, std::mt19937& generator, std::vector<double>& scenario) {
            
            if (is_fail_state(curr))
                return {0, 1};

            if (step == D_default)
                return {0, 0};

            std::vector<action_t> actions = mdp.get_actions(curr);
            std::uniform_int_distribution<std::size_t> distr(0, actions.size() - 1);
            size_t a = distr(generator);
            action_t action = actions[a];
            int rew = mdp.reward(curr, action);
            
            // state distr
            auto options = mdp.state_action(curr, action);
            state_t next = sample_state(std::get<0>(options), std::get<1>(options), scenario[D + step]);

            auto child = default_policy_rec(step + 1, next, generator, scenario);

            return {rew + gamma * child.first, child.second};
        }

        double eta() {
            return u_rwdu - l_rwdu;
        }

        void make_default() {
            U_value = L_value;
            l_rwdu = (scenarios.size() / (double) K) * std::pow(gamma, depth) * L_value;
            u_rwdu = l_rwdu;
        }
    };

    despot(MDP mdp, history h0) : mdp(mdp) {
        
        std::vector<std::vector<double>> scenarios = sample_scenarios();
        n0 = std::make_unique<node>(h0, std::move(scenarios));
    }

    void build_despot() {

        // && total running time is less than Tmax
        while (n0->eta() > eta0) {

            node* n = explore(n0.get());
            backup(n);
        }
    }

    node* explore(node* n) {

        while (n->depth <= D && excess_uncertainty(n) > 0 && !prune(n)) {

        }
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

            n = parent;
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

                int rew = mdp.reward(n->state(), action);
                U_argmax.emplace_back(rew + gamma * sum);

                double w_rew = (n->scenarios.size() / (double) K) * std::pow(gamma, n->depth) * rew - lambda;
                u_argmax.emplace_back(w_rew + sum_u);
                l_argmax.emplace_back(w_rew + sum_l);
            }

            double l0 = (n->scenarios.size() / (double) K) * std::pow(gamma, n->depth) * n->L_value;
            u_argmax.push_back(l0);
            l_argmax.push_back(l0);

            n->U_value = *std::max_element(U_argmax.begin(), U_argmax.end());
            n->u_value = *std::max_element(u_argmax.begin(), u_argmax.end());
            n->l_value = *std::max_element(l_argmax.begin(), l_argmax.end());

            n = n->parent; // move up
        }
    }

    std::vector<std::vector<double>> sample_scenarios() {
        std::vector<std::vector<double>> scenarios;
        std::srand( (unsigned) time(NULL) );

        for(size_t i = 0; i < K; i++) {
            std::vector<double> scenario;

            for (size_t j = 0; j < D + D_default; j++) {
                scenario.emplace_back((double) std::rand() / RAND_MAX);
            }
            
            scenarios.push_back(std::move(scenario));
        }

        return scenarios;
    }

    // sample state using a scenario
    state_t sample_state(std::vector<state_t>& states, std::vector<double>& dist, double scenario) {

        for( size_t i = 0; i < states.size(); i++) {
            if (scenario <= dist[i])
                return states[i];

            scenario -= dist[i];
        }

        assert(false);
    }


};