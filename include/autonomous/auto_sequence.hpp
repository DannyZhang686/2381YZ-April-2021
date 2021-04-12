
#include "autonomous/auto_task.hpp"
#include <vector>

#ifndef AUTO_SEQUENCE_HPP
#define AUTO_SEQUENCE_HPP


class AutoSequence : public AutoTask{
    public: 

    int tasksRemaining = 0;
    void run_sequence();
    void Reset();
    bool isSequenceFinished = false;
    
    std::vector<AutoTask> taskList;
    std::vector<AutoTask> resetTaskList;
    
    void add_tasks(std::vector<AutoTask> tasks);

    static AutoSequence* FromTasks(std::vector<AutoTask> tasks);
private:
    AutoSequence();
};

#endif