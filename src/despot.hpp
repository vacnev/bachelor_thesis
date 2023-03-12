#include <mdp.hpp>
#include <node.hpp>
#include <time.h>


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

    // maximum planning time per step
    // T_max TODO

    despot(MDP mdp, history h0) : mdp(mdp) {
        double L = default_policy(h0);
        n0 = std::make_unique<node>(h0, L);
        std::vector<double> scenarios = sample_scenarios();
        n0->scenarios = std::move(scenarios);
    }

    // random playouts
    double default_policy(history h); //TODO

    std::vector<double> sample_scenarios() {
        std::vector<double> scenarios;
        srand( (unsigned) time(NULL) );
        for(size_t i = 0; i < K; i++) {
            scenarios.emplace_back((double) rand() / RAND_MAX);
        }

        return scenarios;
    }


};