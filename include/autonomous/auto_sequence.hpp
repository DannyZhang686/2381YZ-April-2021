
#ifndef AUTO_SEQUENCE_HPP
#define AUTO_SEQUENCE_HPP

#include "autonomous/auto_task.hpp"
#include <vector>
#include <list>


class AutoSequence : public AutoTask {
    public:

    int tasksRemaining = 0;
    void run_sequence();
    void Reset();
    bool isSequenceFinished = false;

    std::list<AutoTask> taskList;
    std::list<AutoTask> resetTaskList;

    void add_tasks(std::list<AutoTask> tasks);

    AutoSequence(std::list<AutoTask> tasks);

};

#endif