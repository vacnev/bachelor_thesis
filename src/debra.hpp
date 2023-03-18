#include "despot.hpp"

template< typename state_t, typename action_t >
struct debra
{
    MDP mdp;

    // maximum depth
    size_t depth;

    // max risk bound
    double risk_delta;

    debra(MDP mdp, size_t depth, double risk_delta)
         : mdp(mdp), depth(depth), risk_delta(risk_delta) {}

    
};