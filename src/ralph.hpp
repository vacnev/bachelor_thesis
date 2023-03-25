#include "uct_tree.hpp"
#include "include/ortools/linear_solver/linear_solver.h"
#include <chrono>
#include <cassert>


template< typename state_t, typename action_t >
struct ralph
{
    MDP mdp;

    // max risk bound
    double risk_delta;

    // Create the linear solvers with the GLOP backend.
    std::unique_ptr<MPSolver> solver_policy(MPSolver::CreateSolver("GLOP"));
    std::unique_ptr<MPSolver> solver_risk(MPSolver::CreateSolver("GLOP"));

    std::mt19937 generator(std::random_device{}());

    // max depth
    size_t depth;

    // maximum planning time per step
    int T_max = 60;

    ralph(MDP mdp, size_t H, double risk) : mdp(mdp), depth(H), risk_delta(risk) {}

    // sample episode, write (to file) trajectory (policy observation), cummulative reward
    void episode() {

        std::unique_ptr<uct_tree<state_t, action_t>> tree(mdp);

        int cum_payoff = 0;

        double delta = risk_delta;

        for (size_t i = 0; i < depth; i++) {

            // start time
            auto start = std::chrono::steady_clock()::now();

            // simulate until timeout
            while (std::chrono::duration<double>(std::chrono::steady_clock()::now() - start) < T_max) {

                tree->simulate(depth - i);
            }

            solver_policy->Clear();
            solver_risk->Clear();

            // rick contrib estimates and minimal possible risk (objective)
            auto [tau, risk_accept] = define_LP_risk(tree.get());
            const MPSolver::ResultStatus result_status = solver_risk->Solve();

            assert(result_status == MPSolver::OPTIMAL);
            
            // policy
            std::unordered_map<action_t, MPVariable* const> policy = define_LP_policy(tree.get(), delta);
            const MPSolver::ResultStatus result_status = solver_policy->Solve();

            if (result_status != MPSolver::OPTIMAL) {

                // if risk_delta is infeasable we relax the bound
                delta = risk_accept->Value();
                auto policy = define_LP_policy(tree.get(), delta);
                const MPSolver::ResultStatus result_status = solver_policy->Solve();

                assert(result_status == MPSolver::OPTIMAL);
            }


            // sample action
            std::vector<double> policy_distr;
            for (auto it = policy.begin(); it != policy.end(); it++) {
                policy_distr.emplace_back(it->second->solution_value());
            }
           
            std::discrete_distribution<> ad(policy_distr.begin(), policy_distr.end());
            int sample = ad(generator);

            action_t a_star = std::advance(policy.begin(), sample)->first;

            cum_payoff += mdp.reward(tree->root->his.last(), a_star);

            // sample state
            auto state_dist = mdp.state_action(tree->root->his.last(), a_star);
            std::vector<double> weights;
            for (auto [_, w] : state_dist) {
                weights.emplace_back(w);
            }

            std::discrete_distribution<> sd(weights.begin(), weights.end());
            sample = sd(generator);

            state_t s_star = std::advance(state_dist, sample)->first;

            // step
            auto& children = tree->root->children[a_star];
            for (auto it = children.begin(); it != children.end(); ++it) {

                if (*it->state() == s_star) {
                    tree->root = std::move(*it);
                    break;
                }
            }

            //altrisk
            double altrisk = 0;
            std::pair<action_t, state_t> step(a_star, s_star);
            for (auto& [action_state, var] : tau) {

                if (action_state != step)
                    altrisk += var->solution_value();
            }

            // adjust risk delta
            delta = (delta - altrisk) / tau[step]->solution_value();
        }

        // TODO
        // write path and payoff into a file (append)
        // printable state_t
    }

    std::unordered_map<action_t, MPVariable*> define_LP_policy(uct_tree<state_t, action_t>* tree, double risk) {

        std::unordered_map<action_t, MPVariable* const> policy;

        auto root = tree->root.get();

        assert(!root->leaf());

        size_t ctr = 0; //counter

        MPObjective* const objective = solver_policy->MutableObjective();

        MPConstraint* const risk_cons = solver_policy->MakeRowConstraint(0.0, risk); // risk

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

                // st = ac * delta (3)
                MPConstraint* const ac_st = solver_policy->MakeRowConstraint(0, 0);
                ac_st->SetCoefficient(st, -1);
                double delta = states_distr[*it->state()];
                ac_st->SetCoefficient(ac, delta);

                LP_policy_rec(tree, it->get(), st, ctr, risk_cons, objective, 1)
            }
        }


        objective->SetMaximization();

        return policy;
    }

    void LP_policy_rec(uct_tree<state_t, action_t>* tree, uct_tree::node<state_t,
                       action_t>* node, MPVariable* const var, size_t& ctr,
                       MPConstraint* const risk_cons, MPObjective* const objective, size_t node_depth) {

        if (node->leaf()) {

            // objective
            double coef = node->payoff + std::pow(tree->gamma, node_depth) * node->v;
            objective->SetCoefficient(var, coef);

            // risk
            risk_cons->SetCoefficient(var, node->r);
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
                double delta = states_distr[*it->state()];
                ac_st->SetCoefficient(ac, delta);

                LP_policy_rec(tree, it->get(), st, ctr, risk_cons, objective, node_depth + 1)
            }
        }
    }

    auto define_LP_risk(uct_tree<state_t, action_t>* tree) {

        std::unordered_map<std::pair<action_t, state_t>, MPVariable* const> tau; // risk contribution estimates

        auto root = tree->root.get();

        assert(!root->leaf());

        size_t ctr = 0; //counter

        MPObjective* const objective = solver_risk->MutableObjective();

        MPVariable* const r = solver_risk->MakeIntVar(1, 1, "r") // (1)

        MPConstraint* const action_sum = solver_risk->MakeRowConstraint(0, 0); // setting 0 0 for equality could cause rounding problems
        action_sum->SetCoefficient(r, -1); // sum of action prob == prob of parent (2)

        auto actions = mdp.get_actions(root->state());
        for (auto ac_it = actions.begin(); ac_it != actions.end(); ++ac_it) {

            MPVariable* const ac = solver_risk->MakeVar(0.0, 1.0, std::to_string(ctr++));
            action_sum->SetCoefficient(ac, 1);

            auto states_distr = mdp.state_action(root->state(), *ac_it);

            for (auto it = root->children[*ac_it].begin(); it != root->children[*ac_it].end(); ++it) {

                MPVariable* const st = solver_risk->MakeVar(0.0, 1.0, std::to_string(ctr++));

                // st = ac * delta (3)
                MPConstraint* const ac_st = solver_risk->MakeRowConstraint(0, 0);
                ac_st->SetCoefficient(st, -1);
                double delta = states_distr[*it->state()];
                ac_st->SetCoefficient(ac, delta);

                tau[{*ac_it, *it->state()}] = st;

                LP_risk_rec(tree, it->get(), st, ctr, objective)
            }
        }

        objective->SetMinimization();

        return std::pair<>(tau, objective);
    }

    void LP_risk_rec(uct_tree<state_t, action_t>* tree, uct_tree::node<state_t,
                       action_t>* node, MPVariable* const var, size_t& ctr,
                       MPObjective* const objective) {
        
        if (node->leaf()) {

            // objective
            objective->SetCoefficient(var, node->r);

            return;
        }                   

        MPConstraint* const action_sum = solver_risk->MakeRowConstraint(0, 0); // setting 0 0 for equality could cause rounding problems
        action_sum->SetCoefficient(var, -1); // sum of action prob == prob of parent (2)

        auto actions = mdp.get_actions(node->state());
        for (auto ac_it = actions.begin(); ac_it != actions.end(); ++ac_it) {

            MPVariable* const ac = solver_risk->MakeVar(0.0, 1.0, std::to_string(ctr++));
            action_sum->SetCoefficient(ac, 1);

            auto states_distr = mdp.state_action(node->state(), *ac_it);

            for (auto it = node->children[*ac_it].begin(); it != node->children[*ac_it].end(); ++it) {

                MPVariable* const st = solver_risk->MakeVar(0.0, 1.0, std::to_string(ctr++));

                // st = ac * delta (3)
                MPConstraint* const ac_st = solver_risk->MakeRowConstraint(0, 0);
                ac_st->SetCoefficient(st, -1);
                double delta = states_distr[*it->state()];
                ac_st->SetCoefficient(ac, delta);

                tau[{*ac_it, *it->state()}] = st;

                LP_risk_rec(tree, it->get(), st, ctr, objective)
            }
        } 
    }
};
