#include "ralph.hpp"
#include "../hallway/hallway.hpp"

int main()
{
    std::unique_ptr<MDP<state_t, action_t>> mdp = std::make_unique<hallway>("../../hallway/hallway1.txt", 0, 0.2, 1);
    ralph<state_t, action_t> r(mdp.get(), 20, 0);
    r.episode();

    return 0;
}