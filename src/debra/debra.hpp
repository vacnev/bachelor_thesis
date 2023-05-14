#include "despot.hpp"
#include "ortools/linear_solver/linear_solver.h"

using namespace operations_research;

template< typename state_t, typename action_t >
struct debra
{
    MDP<state_t, action_t>* mdp;

    // maximum depth
    size_t depth;

    // max risk bound
    double risk_delta;

    std::mt19937 generator{std::random_device{}()};

    debra(MDP<state_t, action_t>* mdp, size_t depth, double risk_delta)
         : mdp(mdp), depth(depth), risk_delta(risk_delta) {}

    // sample episode, write (to file) trajectory (policy observation), cummulative reward
    void episode() {

        mdp->init_plan();

        // Create the linear solvers with the GLOP backend.
        std::unique_ptr<MPSolver> solver_policy(MPSolver::CreateSolver("GLOP"));
        std::unique_ptr<MPSolver> solver_risk(MPSolver::CreateSolver("GLOP"));

        history<state_t, action_t> h{mdp->initial_state()};

        int cum_payoff = 0;

        double delta = risk_delta;

        size_t expanded_nodes = 0;

        for (size_t i = 0; i < depth; i++) {

            auto tree = std::make_unique<despot<state_t, action_t>>(mdp, depth - i, h);

            solver_policy->Clear();
            solver_risk->Clear();

            // rick contrib estimates and minimal possible risk (objective)
            std::map<std::pair<action_t, state_t>, double> tau;

            for (auto& [ac, vec] : tree->n0->children) {

                for (auto it = vec.begin(); it != vec.end(); ++it) {

                    if (mdp->is_fail_state((*it)->state())) {
                        tau.insert({{ac, (*it)->state()}, 1});
                        continue;
                    }

                    auto obj = define_LP_risk(it->get(), solver_risk.get());
                    MPSolver::ResultStatus result_status = solver_risk->Solve();
                    assert(result_status == MPSolver::OPTIMAL);
                    tau.insert({{ac, (*it)->state()}, obj->Value()});

                    solver_risk->Clear();
                }
            }
            
            // policy
            std::unordered_map<action_t, MPVariable* const> policy = define_LP_policy(tree.get(), delta, solver_policy.get());
            MPSolver::ResultStatus result_status = solver_policy->Solve();

            if (result_status != MPSolver::OPTIMAL) {

                // if risk_delta is infeasable we relax the bound
                auto risk_obj = define_LP_risk(tree->n0.get(), solver_risk.get());
                result_status = solver_risk->Solve();
                assert(result_status == MPSolver::OPTIMAL);

                double altdelta = risk_obj->Value();
                std::cout << "alter risk " << altdelta << '\n';
                solver_policy->Clear();
                policy = define_LP_policy(tree.get(), altdelta, solver_policy.get());
                result_status = solver_policy->Solve();

                assert(result_status == MPSolver::OPTIMAL);
            }

            // sample action
            std::vector<double> policy_distr;
            for (auto it = policy.begin(); it != policy.end(); it++) {
                policy_distr.emplace_back(it->second->solution_value());
                //std::cout << "action: " << it->first << " prob: " << it->second->solution_value() << '\n';
            }
           
            std::discrete_distribution<> ad(policy_distr.begin(), policy_distr.end());
            int sample = ad(generator);

            action_t a_star = std::next(std::begin(policy), sample)->first;

            cum_payoff += mdp->reward(h, h.last(), a_star);

            // all gold taken
            if (mdp->take_gold(h.last())) {
                std::cout << "all gold taken\n";
                break;
            }

            // sample state
            auto state_dist = mdp->state_action(h.last(), a_star);
            std::vector<double> weights;
            for (auto [_, w] : state_dist) {
                weights.emplace_back(w);
            }

            std::discrete_distribution<> sd(weights.begin(), weights.end());
            sample = sd(generator);

            state_t s_star = std::next(std::begin(state_dist), sample)->first;

            // step
            h.add(a_star, s_star);

            //std::cout << "real step: action: " << a_star << " state: (" << s_star.first.first << ", " << s_star.first.second << ") " << s_star.second << '\n';

            if (mdp->is_fail_state(s_star)) {
                std::cout << "killed\n";
                break;
            }

            //altrisk
            double altrisk = 0;
            std::pair<action_t, state_t> step(a_star, s_star);
            for (auto& [action_state, sol] : tau) {

                auto state_distr = mdp->state_action(h.last(), action_state.first);
                if (action_state != step)
                    //std::cout << "altrisk sol: " << var->solution_value() << " in state " << action_state.second.first.first << " " << action_state.second.first.second << '\n';
                    altrisk += sol * policy[action_state.first]->solution_value() * state_distr[action_state.second];
            }

            // adjust risk delta
            //std::cout << "altrisk step sol: " << tau[step] << " sum altrisk: " << altrisk << '\n';
            if (delta > altrisk) {

                auto state_distr = mdp->state_action(h.last(), a_star);
                delta = (delta - altrisk) / (policy[a_star]->solution_value() * state_dist[s_star]);
            }
                

            //std::cout << "root: (" << h.last().first.first << ", " << h.last().first.second << ") " << h.last().second << '\n';
            //std::cout << "real step: action: " << a_star << " state: (" << s_star.first.first << ", " << s_star.first.second << ") " << s_star.second << '\n';
            //std::cout << "cp: " << cum_payoff << '\n';

            expanded_nodes += tree->node_expanded;
        }

        // write result into a file
        std::ofstream file("debra_result.txt", std::ios::out | std::ios::app);
        file << cum_payoff << ";" << mdp->is_fail_state(h.last()) << ';' << expanded_nodes << "\n";
        //mdp->write_history(file, h);
        file.close();
    }

    std::unordered_map<action_t, MPVariable* const> define_LP_policy(despot<state_t, action_t>* tree,
                                                               double risk, MPSolver* solver_policy) {

        std::unordered_map<action_t, MPVariable* const> policy;

        auto root = tree->n0.get();

        assert(!root->leaf());

        size_t ctr = 0; //counter

        MPObjective* const objective = solver_policy->MutableObjective();

        MPConstraint* const risk_cons = solver_policy->MakeRowConstraint(0.0, risk); // risk

        MPVariable* const r = solver_policy->MakeIntVar(1, 1, "r"); // (1)

        MPConstraint* const action_sum = solver_policy->MakeRowConstraint(0, 0); // setting 0 0 for equality could cause rounding problems
        action_sum->SetCoefficient(r, -1); // sum of action prob == prob of parent (2)

        auto actions = mdp->get_actions(root->state());
        for (auto ac_it = actions.begin(); ac_it != actions.end(); ++ac_it) {

            MPVariable* const ac = solver_policy->MakeNumVar(0.0, 1.0, std::to_string(ctr++));
            action_sum->SetCoefficient(ac, 1);
            policy.insert({*ac_it, ac}); // add to policy to access solution

            auto states_distr = mdp->state_action(root->state(), *ac_it);

            for (auto it = root->children[*ac_it].begin(); it != root->children[*ac_it].end(); ++it) {

                MPVariable* const st = solver_policy->MakeNumVar(0.0, 1.0, std::to_string(ctr++));

                // st = ac * delta (3)
                MPConstraint* const ac_st = solver_policy->MakeRowConstraint(0, 0);
                ac_st->SetCoefficient(st, -1);
                double delta = states_distr[(*it)->state()];
                ac_st->SetCoefficient(ac, delta);

                LP_policy_rec(tree, it->get(), st, ctr, risk_cons, objective, solver_policy);
            }
        }


        objective->SetMaximization();

        return policy;
    }

    void LP_policy_rec(despot<state_t, action_t>* tree, typename despot<state_t, action_t>::node* node, MPVariable* const var, size_t& ctr,
                       MPConstraint* const risk_cons, MPObjective* const objective, MPSolver* solver_policy) {

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

        auto actions = mdp->get_actions(node->state());
        for (auto ac_it = actions.begin(); ac_it != actions.end(); ++ac_it) {

            MPVariable* const ac = solver_policy->MakeNumVar(0.0, 1.0, std::to_string(ctr++)); // x_h,a
            action_sum->SetCoefficient(ac, 1);

            auto states_distr = mdp->state_action(node->state(), *ac_it);

            for (auto it = node->children[*ac_it].begin(); it != node->children[*ac_it].end(); ++it) {

                MPVariable* const st = solver_policy->MakeNumVar(0.0, 1.0, std::to_string(ctr++));

                // st = ac * delta
                MPConstraint* const ac_st = solver_policy->MakeRowConstraint(0, 0);
                ac_st->SetCoefficient(st, -1);
                double delta = states_distr[(*it)->state()];
                ac_st->SetCoefficient(ac, delta);

                LP_policy_rec(tree, it->get(), st, ctr, risk_cons, objective, solver_policy);
            }
        }
    }

    auto define_LP_risk(typename despot<state_t, action_t>::node* root, MPSolver* solver_risk) {

        assert(!root->leaf());

        size_t ctr = 0; //counter

        MPObjective* const objective = solver_risk->MutableObjective();

        MPVariable* const r = solver_risk->MakeIntVar(1, 1, "r"); // (1)

        MPConstraint* const action_sum = solver_risk->MakeRowConstraint(0, 0); // setting 0 0 for equality could cause rounding problems
        action_sum->SetCoefficient(r, -1); // sum of action prob == prob of parent (2)

        auto actions = mdp->get_actions(root->state());
        for (auto ac_it = actions.begin(); ac_it != actions.end(); ++ac_it) {

            MPVariable* const ac = solver_risk->MakeNumVar(0.0, 1.0, std::to_string(ctr++));
            action_sum->SetCoefficient(ac, 1);

            auto states_distr = mdp->state_action(root->state(), *ac_it);

            for (auto it = root->children[*ac_it].begin(); it != root->children[*ac_it].end(); ++it) {

                MPVariable* const st = solver_risk->MakeNumVar(0.0, 1.0, std::to_string(ctr++));

                // st = ac * delta (3)
                MPConstraint* const ac_st = solver_risk->MakeRowConstraint(0, 0);
                ac_st->SetCoefficient(st, -1);
                double delta = states_distr[(*it)->state()];
                ac_st->SetCoefficient(ac, delta);

                LP_risk_rec(it->get(), st, ctr, objective, solver_risk);
            }
        }

        objective->SetMinimization();

        return objective;
    }

    void LP_risk_rec(typename despot<state_t, action_t>::node* node,
                     MPVariable* const var, size_t& ctr,
                     MPObjective* const objective, MPSolver* solver_risk) {
        
        if (node->leaf()) {

            // objective
            objective->SetCoefficient(var, node->risk);

            return;
        }                   

        MPConstraint* const action_sum = solver_risk->MakeRowConstraint(0, 0); // setting 0 0 for equality could cause rounding problems
        action_sum->SetCoefficient(var, -1); // sum of action prob == prob of parent (2)

        auto actions = mdp->get_actions(node->state());
        for (auto ac_it = actions.begin(); ac_it != actions.end(); ++ac_it) {

            MPVariable* const ac = solver_risk->MakeNumVar(0.0, 1.0, std::to_string(ctr++));
            action_sum->SetCoefficient(ac, 1);

            auto states_distr = mdp->state_action(node->state(), *ac_it);

            for (auto it = node->children[*ac_it].begin(); it != node->children[*ac_it].end(); ++it) {

                MPVariable* const st = solver_risk->MakeNumVar(0.0, 1.0, std::to_string(ctr++));

                // st = ac * delta (3)
                MPConstraint* const ac_st = solver_risk->MakeRowConstraint(0, 0);
                ac_st->SetCoefficient(st, -1);
                double delta = states_distr[(*it)->state()];
                ac_st->SetCoefficient(ac, delta);

                LP_risk_rec(it->get(), st, ctr, objective, solver_risk);
            }
        } 
    }
};