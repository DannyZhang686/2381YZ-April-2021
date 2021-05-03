#include "main.h"
#include "autonomous/auto_task.hpp"
#include "autonomous/auto_sequence.hpp"
#include <list>
#include <functional>
using namespace std;

AutoSequence AutoSequence::FromTasks(list<AutoTask> tasks) {
    AutoSequence instance = AutoSequence(tasks);

    return instance;
}

void AutoSequence::add_tasks(list<AutoTask> tasks) {
    taskList.insert(taskList.end(), tasks.begin(), tasks.end());
    resetTaskList.insert(resetTaskList.end(), tasks.begin(), tasks.end());
};

AutoSequence::AutoSequence(list<AutoTask> tasks) : AutoTask(
    AutoTaskVectorArgs({
        runList: {[&](void) -> void {//run
            run_sequence();
        }},
        doneList : {[&](void)->bool {//done
            return isSequenceFinished;
        }}})
    )
{
    add_tasks(tasks);
};

void AutoSequence::Reset()
{
    isSequenceFinished = false;
    taskList = list<AutoTask>(resetTaskList);
};

void AutoSequence::run_sequence() {
    // First check if the sequence is done.
    if (taskList.empty())
    {
        isSequenceFinished = true;
        return;
    }
    // Otherwise begin at the first task in the list.
    auto it = taskList.begin();

    while (it != taskList.end()) {

        // Iterate through the tasks in the list, beggining the event loop
        if (!it->_initialized)
        {
            // If the task is not initialized, it means that it is a new task - run the task's 
            // init function and then set the already initialized to true so init only runs once.
            it->initialize();
            it->_initialized = true;
        }

        // Run the main function of the task, contained within the run function - this performs the task's action
        it->run();

        // Done() returns a boolean - it will return true only if the task is considered "finished" - 
        // as defined in the definition of the task. Otherwise it will return false, as task is not "done".
        if (it->done())
        {
            // The Kill() function runs once at the end of the task, in order to perform
            // all of the functions that need to be done as the task is disposed of - stopping the drive for example.
            it->kill();
            // Erase the task from the task list.
            it = taskList.erase(it);
        }

        // If task is a synchronous task - stop the task loop here.
        // If the task is asynchronous, run this task, but continue to run the next tasks in the event loop
        // until a blocking task is reached.
        if (it->isSync)
        {
            break;
        }
        it++;
    }
};
