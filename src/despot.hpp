#include <mdp.hpp>
#include <history.hpp>
#include <time.h>
#include <cassert>
#include <cmath>
#include <random>


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
        size_t depth;
        
        node(history h, std::vector<std::vector<double>>&& scenarios, size_t depth = 0)
             : his(h), scenarios(std::move(scenarios)), depth(depth) {
            initialize_values();
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

        // random playouts, sets L0
        void default_policy() {
            std::mt19937 generator(std::random_device{}());
            double L = 0;
            state_t curr = his.last();

            for (size_t i = 0; i < scenarios.size(); i++) {
                L += default_policy_rec(1, curr, generator, scenarios[i]);
            }
            
            L_value = L / scenarios.size();
        }

        double default_policy_rec(size_t step, state_t curr, std::mt19937& generator, std::vector<double>& scenario) {
            std::vector<action_t> actions = mdp.get_actions(curr);
            std::uniform_int_distribution<std::size_t> distr(0, actions.size() - 1);
            size_t a = distr(generator);
            action_t action = actions[a];
            int rew = mdp.reward(curr, action);

            if (step == D_default)
                return rew;
            
            // state distr
            auto options = mdp.state_action(curr, action);
            state_t next = sample_state(std::get<0>(options), std::get<1>(options), scenario[D + step]);

            return rew + gamma * default_policy_rec(step + 1, next, generator, scenario);
        }
    };

    despot(MDP mdp, history h0) : mdp(mdp) {
        
        std::vector<std::vector<double>> scenarios = sample_scenarios();
        n0 = std::make_unique<node>(h0, std::move(scenarios));
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