#include <mdp.hpp>
#include <history.hpp>
#include <time.h>
#include <cassert>
#include <cmath>


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

    template< typename state_t, typename action_t >
    struct node
    {
        history his;
        double L_value; //lower bound on value function = L0
        double U_value; //upper bound U
        double l_rwdu; // lower bound on rwdu
        double u_rwdu; // upper bound on rwdu
        std::vector<std::unique_ptr<node>> children;
        std::vector<std::vector<double>> scenarios;
        
        node(history h, std::vector<std::vector<double>>&& scenarios) : his(h), scenarios(std::move(scenarios)) {
            initialize_values();
        }

        // init U, l, u
        void initialize_values() {
            default_policy();

            U_value = mdp.max_reward() / (1 - gamma);
            l_rwdu = (scenarios.size() / (double) K) * std::pow(gamma, his.depth()) * L_value;
            u_rwdu = (scenarios.size() / (double) K) * std::pow(gamma, his.depth()) * U_value - lambda;

            if (u_rwdu < l_rwdu)
                u_rwdu = l_rwdu;
        }

        // random playouts, sets L0
        double default_policy() {
            // vyresit depth, delka history != hloubka ve stromu
        }
    };

    despot(MDP mdp, history h0) : mdp(mdp) {
        
        std::vector<std::vector<double>> scenarios = sample_scenarios();
        n0 = std::make_unique<node>(h0, std::move(scenarios));
    }



    std::vector<std::vector<double>> sample_scenarios();

    // sample state using a scenario
    state_t sample_state(std::vector<state_t> states, std::vector<double> dist, double scenario) {

        for( size_t i = 0; i < states.size(); i++) {
            if (scenario <= dist[i])
                return states[i];

            scenario -= dist[i];
        }

        assert(false);
    }


};