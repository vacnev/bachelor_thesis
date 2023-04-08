#include "debra.hpp"
#include "../hallway/hallway.hpp"

int main()
{
    std::unique_ptr<MDP<state_t, action_t>> mdp = std::make_unique<hallway>("../../hallway/hallway1.txt", 0, 0.2, 1);
    debra<state_t, action_t> d(mdp.get(), 20, 0);
    d.episode();

    return 0;
}