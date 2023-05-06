#include "ralph.hpp"
#include "../hallway/hallway.hpp"

int main()
{
    // filename, shift prob, trap prob, gold count
    auto filename = "../../hallway/hallway6.txt";
    double p_s = 0.05;
    double p_t = 1;
    int gold_count = 60;
    std::unique_ptr<MDP<state_t, action_t>> mdp = std::make_unique<hallway>(filename, p_s, p_t, gold_count);

    // max depth, allowed risk
    size_t max_depth = 500;
    double delta_threshold = 0;
    ralph<state_t, action_t> r(mdp.get(), max_depth, delta_threshold);

    for (size_t i = 0; i < 5; i++) {
        r.episode();
    }

    return 0;
}