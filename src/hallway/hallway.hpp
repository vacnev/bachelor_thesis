#include "../mdp.hpp"

enum Direction { LEFT, UP, DOWN, RIGHT };

// coordinates (row, collumn) from top left
using state_t = std::pair<std::pair<int, int>, Direction>;

enum action_t { TURN_RIGHT, TURN_LEFT, FORWARD };


// + spawn, x trap, g gold, 1 wall

struct hallway : public MDP<state_t, action_t>
{
    // probability we shift during forward
    double shift_p;

    // probability trap sets
    double trap_p;

    // fail state sink
    state_t f_state = {{0, 0}, UP};

    // move_rew (penalization)
    int move_rew = -1;

    // gold rew
    int gold_rew = 100;

    // number of gold
    int gold_count;

    std::vector<std::string> plan;

    // original gold count
    int init_gold_count;

    std::string filename;

    hallway(std::string filename, double s, double t, int g_c)
           : shift_p(s), trap_p(t), init_gold_count(g_c), filename(filename) {
        init_plan();
    }

    void init_plan() override {
        plan.clear();
        gold_count = init_gold_count;

        std::string line;
        std::ifstream file(filename, std::ios::in);

        if (!file.is_open())
            std::cout << "couldnt open file\n";

        while( std::getline(file, line) ) {
            std::cout << line << std::endl;
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

        return {};
    }

    int gold_rem() override {
        return gold_count;
    }

    int gold_reward() override {
        return gold_rew;
    }

    bool is_fail_state(const state_t& s) override {
        return s == f_state;
    }

    int max_payoff() override {
        return gold_rew * gold_count;
    }

    std::vector<action_t> get_actions(const state_t& s) override {
        if (s == f_state)
            return {};

        std::pair<int, int> dir;
        switch (s.second) {
            case UP: dir = {-1, 0}; break;
            case DOWN: dir = {1, 0}; break;
            case LEFT: dir = {0, -1}; break;
            case RIGHT: dir = {0, 1}; break;
        }

        // not bouncing into walls
        auto coord = add(s.first, dir);
        if (is_wall(coord))
            return {TURN_RIGHT, TURN_LEFT};
        
        return {TURN_RIGHT, TURN_LEFT, FORWARD};
    }

    // possible outcome states, probability vector
    std::map<state_t, double> state_action(const state_t& s, const action_t& a) override {

        std::map<state_t, double> s_a;

        /*std::cout << "state_action:";
        std::cout << '(' << s.first.first << ", " << s.first.second << ") " << s.second;
        std::cout << " - " << a << "\n";*/

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
    void forward_dir(const state_t& s, std::map<state_t, double>& s_a, int left, int forward, int right) {

        // left, up, down, right
        std::array<std::pair<int, int>, 4> dirs = { {{0,-1}, {-1,0}, {1,0}, {0,1}} };
        double prob = 1;

        std::pair<int, int> coord = add(s.first, dirs[forward]);
        if (!is_wall(coord)) {

            //left
            std::pair<int, int> coord_shift = add(coord, dirs[left]);
            if (!is_wall(coord_shift)) {
                prob -= shift_p;

                if (is_trap(coord_shift)) {
                    //std::cout << "TRAP\n";
                    s_a[f_state] += shift_p * trap_p;
                    s_a.insert({std::make_pair(coord_shift, s.second), shift_p * (1 - trap_p)});
                } else {
                    s_a.insert({std::make_pair(coord_shift, s.second), shift_p});
                }
            }
            
            // right
            coord_shift = add(coord, dirs[right]);
            if (!is_wall(coord_shift)) {
                prob -= shift_p;

                if (is_trap(coord_shift)) {
                    //std::cout << "TRAP\n";
                    s_a[f_state] += shift_p * trap_p;
                    s_a.insert({std::make_pair(coord_shift, s.second), shift_p * (1 - trap_p)});
                } else {
                    s_a.insert({std::make_pair(coord_shift, s.second), shift_p});
                }
            }

            //forward
            if (is_trap(coord)) {
                //std::cout << "TRAP\n";
                s_a[f_state] += prob * trap_p;
                s_a.insert({std::make_pair(coord, s.second), prob * (1 - trap_p)});
            } else {
                s_a.insert({std::make_pair(coord, s.second), prob});
            }
        }
    }

    std::pair<int, int> add(const std::pair<int, int>& l, const std::pair<int, int>& r) {
        return {l.first + r.first, l.second + r.second};
    }

    bool is_wall(std::pair<int, int>& coord) {
        //std::cout << "isWALL: " << coord.first << " " << coord.second << '\n';
        return plan[coord.first][coord.second] == '#';
    }

    bool is_trap(std::pair<int, int>& coord) {
        //std::cout << "isTRAP: " << coord.first << " " << coord.second << '\n';
        return plan[coord.first][coord.second] == 'x';
    }


    // treasure reward for action from treasure state
    int reward(history<state_t, action_t>& his, const state_t& s, const action_t& a) override {

        // gold present and not already taken in history
        if (plan[s.first.first][s.first.second] == 'g') {

            for (size_t i = 0; i < his.states.size() - 1; ++i) {

                if (his.states[i].first == s.first)
                    return move_rew;
            }

            return gold_rew;
        }

        return move_rew;
    }

    // at real step
    // return true if all golds have been taken
    bool take_gold(const state_t& s) override {

        if (plan[s.first.first][s.first.second] == 'g') {
            plan[s.first.first][s.first.second] = ' ';
            gold_count--;
        }

        return !gold_count;
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