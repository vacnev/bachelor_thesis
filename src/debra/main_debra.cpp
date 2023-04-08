#include "debra.hpp"
#include "../hallway/hallway.hpp"

int main()
{
    // filename, shift prob, trap prob, gold count
    std::unique_ptr<MDP<state_t, action_t>> mdp = std::make_unique<hallway>("../../hallway/hallway2.txt", 0.1, 1, 3);

    // max depth, allowed risk
    debra<state_t, action_t> d(mdp.get(), 30, 0);
    d.episode();

    return 0;
}