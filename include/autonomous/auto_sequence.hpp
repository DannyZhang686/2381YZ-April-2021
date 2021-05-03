
#include "autonomous/auto_task.hpp"
#include <vector>
#include <list>

#ifndef AUTO_SEQUENCE_HPP
#define AUTO_SEQUENCE_HPP


class AutoSequence : public AutoTask {
    public:

    int tasksRemaining = 0;
    void run_sequence();
    void Reset();
    bool isSequenceFinished = false;

    std::list<AutoTask> taskList;
    std::list<AutoTask> resetTaskList;

    void add_tasks(std::list<AutoTask> tasks);

    static AutoSequence FromTasks(std::list<AutoTask> tasks);
    const AutoTaskVectorArgs SequenceConstructorArgs
    {
        runList: {[&](void) -> void {//run
            run_sequence();
        }},
        doneList : {[&](void)->bool {//done
            return isSequenceFinished;
        }}
    };
    AutoSequence(std::list<AutoTask> tasks);

    // private:
};

#endif