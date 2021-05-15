#include "main.h"
#include "autonomous/auto_task.hpp"

AutoTask AutoTask::AsyncTask(std::function<void(void)> task, std::function<bool(void)> done, std::function<void(void)> init, std::function<void(void)> kill)
{
    AutoTask asyncTask = AutoTask(task, done, false, init, kill);
    return asyncTask;
}
AutoTask AutoTask::SyncTask(std::function<void(void)> task, std::function<bool(void)> done, std::function<void(void)> init, std::function<void(void)> kill)
{
    AutoTask syncTask = AutoTask(task, done, true, init, kill);
    return syncTask;
}

AutoTask::AutoTask(std::function<void(void)> task, std::function<bool(void)> done, bool sync, std::function<void(void)> init, std::function<void(void)> kill)
    : isSync(sync), doneList({done}), runList({task}), initList({init}), killList({kill})
{
}

AutoTask::AutoTask(const AutoTaskVectorArgs args)
    : isSync(args.sync), doneList(args.doneList), runList(args.runList), initList(args.runList), killList(args.killList)
{

};

AutoTask& AutoTask::AddRun(std::function<void(void)> task)
{
    runList.push_back(task);
    return *this;
}

AutoTask& AutoTask::AddDone(std::function<bool(void)> done)
{
    doneList.push_back(done);
    return *this;
}

AutoTask& AutoTask::AddInit(std::function<void(void)> task)
{
    initList.push_back(task);
    return *this;
}


AutoTask& AutoTask::AddKill(std::function<void(void)> task)
{
    killList.push_back(task);
    return *this;
}


void AutoTask::run(void)
{
    for (const auto& value : this->runList)
    {
        value();
    }
}

void AutoTask::kill(void)
{
    for (const auto& value : this->killList)
    {
        value();
    }
}
void AutoTask::initialize(void)
{
    for (const auto& value : this->initList)
    {
        value();
    }
}
bool AutoTask::done(void)
{
    auto isDone = false;
    for (const auto& value : this->doneList)
    {
        isDone = isDone || value();
    }
    return isDone;
}