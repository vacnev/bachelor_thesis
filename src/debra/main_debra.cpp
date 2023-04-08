#include "debra.hpp"
#include "../hallway/hallway.hpp"

int main()
{
    // filename, shift prob, trap prob, gold count
    std::unique_ptr<MDP<state_t, action_t>> mdp = std::make_unique<hallway>("../../hallway/hallway3.txt", 0, 0.05, 1);

    // max depth, allowed risk
    debra<state_t, action_t> d(mdp.get(), 20, 0.05);
    d.episode();

    return 0;
}