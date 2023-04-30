#include "debra.hpp"
#include "../hallway/hallway.hpp"

int main()
{
    // filename, shift prob, trap prob, gold count
    auto filename = "../../hallway/hallway2.txt";
    double p_s = 0.1;
    double p_t = 1;
    int gold_count = 3;
    std::unique_ptr<MDP<state_t, action_t>> mdp = std::make_unique<hallway>(filename, p_s, p_t, gold_count);

    // max depth, allowed risk
    size_t max_depth = 60;
    double delta_threshold = 0;
    debra<state_t, action_t> d(mdp.get(), max_depth, delta_threshold);

    for (size_t i = 0; i < 100; i++) {
        d.episode();
    }
    

    return 0;
}