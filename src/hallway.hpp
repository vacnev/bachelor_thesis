#include "mdp.hpp"
#include <string>
#include <iostream>
#include <fstream>
#include <cassert>

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

    bool is_fail_state(state_t s) override {
        return s == f_state;
    }

    int max_reward() override {
        return gold_rew;
    }

    std::vector<action_t> get_actions(state_t s) {
        return {TURN_RIGHT, TURN_LEFT, FORWARD};
    }

    // possible outcome states, probability vector
    std::unordered_map<state_t, double> state_action(state_t& s, action_t& a) override {

        std::unordered_map<state_t, double> s_a;

        // left, up, down, right
        std::array<std::pair<int, int>, 4> dirs = { {{0,-1}, {1,0}, {-1,0}, {0,1}} };

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
                double prob = 1;
                
                switch (s.second) {

                    case UP:
                        std::pair<int, int> coord = add(s.first, dirs[2]);
                        if (!is_wall(coord)) {
                            
                            //left
                            std::pair<int, int> coord_shift = add(s.first, dirs[1])

                            //if wall
                        }
                            
                }
        }
    }

    std::pair<int, int> add(std::pair<int, int>& l, std::pair<int, int>& r) {
        return {l.first + r.first, l.second + r.second};
    }

    bool is_wall(std::pair<int, int>& coord) {
        return plan[coord.first][coord.second] == '1';
    }

    // treasure reward for action from treasure state
    int reward(history<state_t, action_t>&, state_t, action_t) = 0;

    ~hallway() = default;
};