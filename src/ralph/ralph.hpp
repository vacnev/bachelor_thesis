#include "uct_tree.hpp"
#include "ortools/linear_solver/linear_solver.h"

using namespace operations_research;

template< typename state_t, typename action_t >
struct ralph
{
    MDP<state_t, action_t>* mdp;

    // max risk bound
    double risk_delta;

    std::mt19937 generator{std::random_device{}()};

    // max depth
    size_t depth;

    // maximum planning time per step
    double T_max = 3;

    ralph(MDP<state_t, action_t>* mdp, size_t H, double risk)
         : mdp(mdp), risk_delta(risk), depth(H) {}

    // sample episode, write (to file) trajectory (policy observation), cummulative reward
    void episode() {

        mdp->init_plan();

        // Create the linear solvers with the GLOP backend.
        std::unique_ptr<MPSolver> solver_policy(MPSolver::CreateSolver("GLOP"));
        std::unique_ptr<MPSolver> solver_risk(MPSolver::CreateSolver("GLOP"));

        auto tree = std::make_unique<uct_tree<state_t, action_t>>(mdp);

        int cum_payoff = 0;

        double delta = risk_delta;

        for (size_t i = 0; i < depth; i++) {

            // start time
            auto start = std::chrono::steady_clock::now();

            // simulate until timeout
            while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < T_max) {

                tree->simulate(depth - i);
            }

            //std::cout << "end simulation\n";

            //std::unique_ptr<MPSolver> solver_policy(MPSolver::CreateSolver("GLOP"));
            //std::unique_ptr<MPSolver> solver_risk(MPSolver::CreateSolver("GLOP"));

            solver_policy->Clear();
            solver_risk->Clear();

            // rick contrib estimates and minimal possible risk (objective)
            //auto [tau, risk_accept] = define_LP_risk(tree.get(), solver_risk.get());
            //MPSolver::ResultStatus result_status = solver_risk->Solve();

            //tau - risk contrib estimates and minimal possible risk (objective)
            std::map<std::pair<action_t, state_t>, double> tau;

            for (auto& [ac, vec] : tree->root->children) {

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

            //std::cout << "risk_solved\n" ; 
            
            // policy
            std::unordered_map<action_t, MPVariable* const> policy = define_LP_policy(tree.get(), delta, solver_policy.get());
            MPSolver::ResultStatus result_status = solver_policy->Solve();

            //std::cout << "policy solved\n";

            if (result_status != MPSolver::OPTIMAL) {

                // if risk_delta is infeasable we relax the bound
                std::cout << "alter risk\n";
                auto risk_obj = define_LP_risk(tree->root.get(), solver_risk.get());
                result_status = solver_risk->Solve();
                assert(result_status == MPSolver::OPTIMAL);

                double altdelta = risk_obj->Value();
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

            cum_payoff += mdp->reward(tree->root->his, tree->root->state(), a_star);

            if (mdp->take_gold(tree->root->state())) {
                std::cout << "all gold taken\n";
                break;
            }

            // sample state
            auto state_dist = mdp->state_action(tree->root->state(), a_star);
            std::vector<double> weights;
            for (auto [_, w] : state_dist) {
                weights.emplace_back(w);
            }

            std::discrete_distribution<> sd(weights.begin(), weights.end());
            sample = sd(generator);

            state_t s_star = std::next(std::begin(state_dist), sample)->first;

            // step
            auto& children = tree->root->children[a_star];
            //std::cout << "next root: " << children.empty() << '\n';
            for (auto it = children.begin(); it != children.end(); ++it) {

                if ((*it)->state() == s_star) {
                    tree->root = std::move(*it);
                    break;
                }
            }

            if (mdp->is_fail_state(s_star)) {
                std::cout << "killed\n";
                break;
            }

            //altrisk
            double altrisk = 0;
            std::pair<action_t, state_t> step(a_star, s_star);
            for (auto& [action_state, sol] : tau) {

                auto state_distr = mdp->state_action(tree->root->state(), action_state.first);
                if (action_state != step)
                    //std::cout << "altrisk sol: " << var->solution_value() << " in state " << action_state.second.first.first << " " << action_state.second.first.second << '\n';
                    altrisk += sol * policy[action_state.first]->solution_value() * state_distr[action_state.second];
            }

            // adjust risk delta
            //std::cout << "altrisk step sol: " << tau[step] << " sum altrisk: " << altrisk << '\n';
            if (delta > altrisk) {

                auto state_distr = mdp->state_action(tree->root->state(), a_star);
                delta = (delta - altrisk) / (policy[a_star]->solution_value() * state_dist[s_star]);
            }
                

            //std::cout << "root: (" << tree->root->state().first.first << ", " << tree->root->state().first.second << ") " << tree->root->state().second << '\n';
            std::cout << "real step: action: " << a_star << " state: (" << s_star.first.first << ", " << s_star.first.second << ") " << s_star.second << '\n';
            std::cout << "cp: " << cum_payoff << '\n';
        }

        //std::cout << "file write\n";
        std::ofstream file("ralph_result.txt", std::ios::out | std::ios::app);
        file << cum_payoff << ";" << mdp->is_fail_state(tree->root->state()) << ';' << tree->node_expanded << "\n";
        //mdp->write_history(file, tree->root->his);
        file.close();
    }

    std::unordered_map<action_t, MPVariable* const> define_LP_policy(uct_tree<state_t, action_t>* tree,
                                                     double risk, MPSolver* solver_policy) {

        std::unordered_map<action_t, MPVariable* const> policy;

        auto root = tree->root.get();

        assert(!root->leaf());

        size_t ctr = 0; //counter

        MPObjective* const objective = solver_policy->MutableObjective();

        MPConstraint* const risk_cons = solver_policy->MakeRowConstraint(0.0, risk); // risk

        //std::cout << "risk: " << risk << '\n';

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

                LP_policy_rec(tree, it->get(), st, ctr, risk_cons, objective, 1, solver_policy);
            }
        }


        objective->SetMaximization();

        return policy;
    }

    void LP_policy_rec(uct_tree<state_t, action_t>* tree, typename uct_tree<state_t, action_t>::node* node,
                       MPVariable* const var, size_t& ctr, MPConstraint* const risk_cons, MPObjective* const objective,
                       size_t node_depth, MPSolver* solver_policy) {

        if (node->leaf()) {

            // objective
            double coef = (node->payoff / std::pow(tree->gamma, tree->root->his.actions.size()))  + std::pow(tree->gamma, node_depth) * node->v;
            objective->SetCoefficient(var, coef);
            //std::cout << node->state().first.first << ", " << node->state().first.second << " leaf payoff: " << coef << " from first action: " << node->his.actions[0] << " with risk: " << node->r << '\n';

            // risk
            risk_cons->SetCoefficient(var, node->r);
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

                LP_policy_rec(tree, it->get(), st, ctr, risk_cons, objective, node_depth + 1, solver_policy);
            }
        }
    }

    auto define_LP_risk(typename uct_tree<state_t, action_t>::node* root, MPSolver* solver_risk) {

        //std::map<std::pair<action_t, state_t>, MPVariable* const> tau; // risk contribution estimates

        assert(!root.leaf());

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

                // tau[std::make_pair(*ac_it, (*it)->state())] = st;
                //state_t s = (*it)->state();
                //tau.insert({std::make_pair(*ac_it, (*it)->state()), st});

                LP_risk_rec(it->get(), st, ctr, objective, solver_risk);
            }
        }

        objective->SetMinimization();

        return objective;
    }

    void LP_risk_rec(typename uct_tree<state_t, action_t>::node* node,
                     MPVariable* const var, size_t& ctr,
                     MPObjective* const objective, MPSolver* solver_risk) {

        if (node->leaf()) {

            // objective
            objective->SetCoefficient(var, node->r);

            //std::cout << "leaf risk: " << node->r << '\n';

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
