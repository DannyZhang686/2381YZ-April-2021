#ifndef MASTER_CONTROLLER_HPP_
#define MASTER_CONTROLLER_HPP_
#include <functional>
#include <map>
#include <string>
#include "api.h"
// using namespace std;

class MasterController
{
public:
    void selector();
    void autonomous();
    void debug();
    void auton_editor();
    // static const int driver_mode = 0;
    // static const int auton_mode = 1; 
    // static const int debug_mode = 2;
    // static const int selector_mode = 3;
    enum State { driver_mode , auton_mode , debug_mode , selector_mode, auton_edit_mode};
    int _active_mode = driver_mode;

    double _autonomous_increment = 0;
    double _local_increment = 0;
    int _selected_module = 0;
    int _selected_mode = 0;
    static std::map<State,std::string> stateList;
    void run();

    std::vector<pros::Task> AutonTasks = {};
    std::vector<pros::Task> DriverTasks = {};


    void set_state(State state);

    static MasterController *instance();
};
#endif
