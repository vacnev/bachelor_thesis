#include <despot.hpp>

std::vector<std::vector<double>> despot::sample_scenarios() {
    std::vector<std::vector<double>> scenarios;
    srand( (unsigned) time(NULL) );

    for(size_t i = 0; i < K; i++) {
        std::vector<double> scenario;

        for (size_t j = 0; j < D + D_default; j++) {
            scenario.emplace_back((double) rand() / RAND_MAX);
        }
        
        scenarios.push_back(std::move(scenario));
    }

    return scenarios;
}