#include "mdp.hpp"

enum Direction { LEFT, UP, DOWN, RIGHT };

// coordinates (row, collumn) from top left
using state_t = std::pair<std::pair<int, int>, Direction>;

enum action_t { TURN_RIGHT, TURN_LEFT, FORWARD };


// + spawn, x trap, g gold, 1 wall

struct hallway : public MDP<state_t, action_t>
{
    // probability we shift during forward
    double shift_p = 0.20;

    // probability trap sets
    double trap_p = 0.6;

    // fail state sink
    state_t f_state = {{0, 0}, UP};

    // move_rew (penalization)
    int move_rew = -1;

    // gold rew
    int gold_rew = 100;

    std::vector<std::string> plan;

    hallway(std::string filename) {
        std::string line;
        std::ifstream file(filename);

        while( std::getline(file, line) ) {
            plan.push_back(std::move(line));
        }

        file.close();
    }

    // always heading UP
    state_t initial_state() override {

        size_t j;
        for (size_t i = 0; i < plan.size(); i++) {

            if ((j = plan[i].find('+')) != std::string::npos) {
                return {{i, j}, UP};
            }
        }

        assert(false);
    }

    bool is_fail_state(const state_t& s) override {
        return s == f_state;
    }

    int max_reward() override {
        return gold_rew;
    }

    std::vector<action_t> get_actions(state_t& s) override {
        if (s == f_state)
            return {};

        return {TURN_RIGHT, TURN_LEFT, FORWARD};
    }

    // possible outcome states, probability vector
    std::map<state_t, double> state_action(state_t& s, action_t& a) override {

        std::map<state_t, double> s_a;

        switch (a) {

            case TURN_LEFT:
                switch (s.second) {

                    case UP: s_a[{s.first, LEFT}] = 1; break;

                    case LEFT: s_a[{s.first, DOWN}] = 1; break;

                    case DOWN: s_a[{s.first, RIGHT}] = 1; break;

                    case RIGHT: s_a[{s.first, UP}] = 1; break;
                }
                break;

            case TURN_RIGHT:
                switch (s.second) {

                    case UP: s_a[{s.first, RIGHT}] = 1; break;

                    case LEFT: s_a[{s.first, UP}] = 1; break;

                    case DOWN: s_a[{s.first, LEFT}] = 1; break;

                    case RIGHT: s_a[{s.first, DOWN}] = 1; break;
                }
                break;

            case FORWARD:
                
                switch (s.second) {

                    case UP:
                        forward_dir(s, s_a, 0, 1, 3);
                        break;

                    case LEFT:
                        forward_dir(s, s_a, 2, 0, 1);
                        break;
                    
                    case DOWN:
                        forward_dir(s, s_a, 3, 2, 0);
                        break;

                    case RIGHT:
                        forward_dir(s, s_a, 1, 3, 2);
                        break;
                }
        }

        return s_a;
    }

    // creates state_distr for forward action for one direction
    // last 3 arguments are keys to dirs (how to move)
    void forward_dir(state_t& s, std::map<state_t, double>& s_a, int left, int forward, int right) {

        // left, up, down, right
        std::array<std::pair<int, int>, 4> dirs = { {{0,-1}, {1,0}, {-1,0}, {0,1}} };
        double prob = 1;

        std::pair<int, int> coord = add(s.first, dirs[forward]);
        if (!is_wall(coord)) {

            //left
            std::pair<int, int> coord_shift = add(coord, dirs[left]);
            if (!is_wall(coord_shift)) {
                prob -= shift_p;

                if (is_trap(coord_shift)) {
                    s_a[f_state] += shift_p * trap_p;
                    s_a[{coord_shift, s.second}] = shift_p * (1 - trap_p);
                } else {
                    s_a[{coord_shift, s.second}] = shift_p;
                }
            }
            
            // right
            coord_shift = add(coord, dirs[right]);
            if (!is_wall(coord_shift)) {
                prob -= shift_p;

                if (is_trap(coord_shift)) {
                    s_a[f_state] += shift_p * trap_p;
                    s_a[{coord_shift, s.second}] = shift_p * (1 - trap_p);
                } else {
                    s_a[{coord_shift, s.second}] = shift_p;
                }
            }

            //forward
            if (is_trap(coord)) {
                s_a[f_state] += prob * trap_p;
                s_a[{coord, s.second}] = prob * (1 - trap_p);
            } else {
                s_a[{coord, s.second}] = prob;
            }
        }
    }

    std::pair<int, int> add(std::pair<int, int>& l, std::pair<int, int>& r) {
        return {l.first + r.first, l.second + r.second};
    }

    bool is_wall(std::pair<int, int>& coord) {
        return plan[coord.first][coord.second] == '1';
    }

    bool is_trap(std::pair<int, int>& coord) {
        return plan[coord.first][coord.second] == 'x';
    }


    // treasure reward for action from treasure state
    int reward(history<state_t, action_t>& his, state_t& s, action_t& a) override {

        action_t& tmp = a;

        // gold present and not already taken in history
        if (plan[s.first.first][s.first.second] == 'g' &&
            ++std::find(his.states.begin(), his.states.end(), s) == his.states.end()) {

            return gold_rew;
        }

        return move_rew;
    }

    // at real step
    void take_gold(state_t& s) override {

        if (plan[s.first.first][s.first.second] == 'g') {
            plan[s.first.first][s.first.second] = ' ';
        }
    }

    // write history for eval
    void write_history(std::ofstream& file, history<state_t, action_t>& his) override {
        
        for (size_t i = 0; i < his.actions.size(); ++i) {

            state_t& s = his.states[i];
            action_t& a = his.actions[i];
            file << '(' << s.first.first << ", " << s.first.second << ") " << s.second;
            file << " - " << a << " | ";
        }

        state_t& s = his.last();
        file << '(' << s.first.first << ", " << s.first.second << ") " << s.second << '\n';
    }

    ~hallway() override = default;
};