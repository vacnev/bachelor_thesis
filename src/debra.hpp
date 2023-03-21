#include "despot.hpp"
#include "include/ortools/linear_solver/linear_solver.h"

template< typename state_t, typename action_t >
struct debra
{
    MDP mdp;

    // maximum depth
    size_t depth;

    // max risk bound
    double risk_delta;

    // Create the linear solvers with the GLOP backend.
    std::unique_ptr<MPSolver> solver_policy(MPSolver::CreateSolver("GLOP"));
    std::unique_ptr<MPSolver> solver_risk(MPSolver::CreateSolver("GLOP"));

    debra(MDP mdp, size_t depth, double risk_delta)
         : mdp(mdp), depth(depth), risk_delta(risk_delta) {}

    // sample episode, write (to file) trajectory (policy observation), cummulative reward
    void episode() {

        history h{mdp.initial_state()};

        for (size_t i = 0; i < depth; i++) {

            auto tree = std::make_unique<despot<state_t, action_t>>(mdp, depth - i, h);

            solver_policy->Clear();
            solver_risk->Clear();

            define_LP_risk(tree.get());
            //solve

            std::unordered_map<action_t, MPVariable* const> policy = define_LP_policy(tree.get(), risk_delta);
            //solve if infeasible define again with risk from LPrisk
        }
    }

    std::unordered_map<action_t, MPVariable*> define_LP_policy(despot<state_t, action_t>* tree, double risk) {

        std::unordered_map<action_t, MPVariable* const> policy;

        auto root = tree->n0.get();

        assert(!root->leaf());

        size_t ctr = 0; //counter

        MPObjective* const objective = solver_policy->MutableObjective();

        MPConstraint* const risk_cons = solver_policy->MakeRowConstraint(0.0, risk);

        MPVariable* const r = solver_policy->MakeIntVar(1, 1, "r") // (1)

        MPConstraint* const action_sum = solver_policy->MakeRowConstraint(0, 0); // setting 0 0 for equality could cause rounding problems
        action_sum->SetCoefficient(r, -1); // sum of action prob == prob of parent (2)

        auto actions = mdp.get_actions(root->state());
        for (auto ac_it = actions.begin(); ac_it != actions.end(); ++ac_it) {

            MPVariable* const ac = solver_policy->MakeVar(0.0, 1.0, std::to_string(ctr++));
            action_sum->SetCoefficient(ac, 1);
            policy[*ac_it] = ac; // add to policy to access solution

            auto states_distr = mdp.state_action(root->state(), *ac_it);

            for (auto it = root->children[*ac_it].begin(); it != root->children[*ac_it].end(); ++it) {

                MPVariable* const st = solver_policy->MakeVar(0.0, 1.0, std::to_string(ctr++));

                // st = ac * delta
                MPConstraint* const ac_st = solver_policy->MakeRowConstraint(0, 0);
                ac_st->SetCoefficient(st, -1);
                double delta = states_distr[it->state()];
                ac_st->SetCoefficient(ac, delta);

                LP_policy_rec(tree, &(*it), st, ctr, risk_cons, objective)
            }
        }


        objective->SetMaximization();
    }

    void LP_policy_rec(despot<state_t, action_t>* tree, despot::node<state_t,
                       action_t>* node, MPVariable* const var, size_t& ctr,
                       MPConstraint* const risk_cons, MPObjective* const objective) {
        // check leaf - objective, risk

        if (node->leaf()) {

            // objective
            double coef = node->payoff + std::pow(tree->gamma, node->depth) * node->L_value;
            objective->SetCoefficient(var, coef);

            // risk
            risk_cons->SetCoefficient(var, node->risk);
            return;
        }

        MPConstraint* const action_sum = solver_policy->MakeRowConstraint(0, 0); // setting 0 0 for equality could cause rounding problems
        action_sum->SetCoefficient(var, -1); // sum of action prob == prob of parent (2)

        auto actions = mdp.get_actions(node->state());
        for (auto ac_it = actions.begin(); ac_it != actions.end(); ++ac_it) {

            MPVariable* const ac = solver_policy->MakeVar(0.0, 1.0, std::to_string(ctr++)); // x_h,a
            action_sum->SetCoefficient(ac, 1);

            auto states_distr = mdp.state_action(node->state(), *ac_it);

            for (auto it = node->children[*ac_it].begin(); it != node->children[*ac_it].end(); ++it) {

                MPVariable* const st = solver_policy->MakeVar(0.0, 1.0, std::to_string(ctr++));

                // st = ac * delta
                MPConstraint* const ac_st = solver_policy->MakeRowConstraint(0, 0);
                ac_st->SetCoefficient(st, -1);
                double delta = states_distr[it->state()];
                ac_st->SetCoefficient(ac, delta);

                LP_policy_rec(tree, &(*it), st, ctr, risk_cons, objective)
            }
        }
    }

    void define_LP_risk(despot<state_t, action_t>* tree);

    void LP_risk_rec(despot<state_t, action_t>* tree, despot::node<state_t,
                       action_t>* node, MPVariable* const var, size_t& ctr,
                       MPObjective* const objective);
};