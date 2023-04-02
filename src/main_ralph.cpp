#include "ralph.hpp"
#include "hallway.hpp"

int main()
{
    std::unique_ptr<MDP<state_t, action_t>> mdp = std::make_unique<hallway>("../hallway1.txt");
    ralph<state_t, action_t> r(mdp.get(), 9, 0);
    r.episode();

    return 0;
}