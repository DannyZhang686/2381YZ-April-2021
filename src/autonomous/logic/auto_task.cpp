#include "main.h"
#include "autonomous/auto_task.hpp"
#include "autonomous/auto_timer.hpp"

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
AutoTask AutoTask::AutoDelay(int time, bool sync)
{
    return AutoTimer({interval: time, sync : sync});
}

AutoTask::AutoTask(std::function<void(void)> task, std::function<bool(void)> done, bool sync, std::function<void(void)> init, std::function<void(void)> kill)
    : isSync(sync), doneList({done}), runList({task}), initList({init}), killList({kill})
{
}

AutoTask::AutoTask(const AutoTaskVectorArgs args) 
    : isSync(args.sync), doneList(args.doneList), runList(args.runList), initList(args.runList), killList(args.killList)
{

};

AutoTask AutoTask::AddRun(std::function<void(void)> task)
{
    auto runList2 = runList;
    runList2.push_back(task);
    return AutoTask({runList: runList2, doneList : doneList, initList : initList, killList : killList, sync : isSync});
}

AutoTask AutoTask::AddDone(std::function<bool(void)> done)
{
    auto doneList2 = doneList;
    doneList2.push_back(done);
    return AutoTask({runList: runList, doneList : doneList2, initList : initList, killList : killList, sync : isSync});
}

AutoTask AutoTask::AddInit(std::function<void(void)> task)
{
    auto initList2 = initList;
    initList2.push_back(task);
    return AutoTask({runList: runList, doneList : doneList, initList : initList2, killList : killList, sync : isSync});
}


AutoTask AutoTask::AddKill(std::function<void(void)> task)
{
    auto killList2 = killList;
    killList2.push_back(task);
    return AutoTask({runList: runList, doneList : doneList, initList : initList, killList : killList2, sync : isSync});
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
AutoTask AutoTask::TimeLimit(int time)
{

    AutoTask timedTask = AutoTimer(time);
    for (const auto& value : this->runList)
    {
        timedTask.AddRun(value);
    };
    for (const auto& value : this->killList)
    {
        timedTask.AddKill(value);
    }
    for (const auto& value : this->initList)
    {
        timedTask.AddInit(value);
    }
    for (const auto& value : this->doneList)
    {
        timedTask.AddDone(value);
    }
    return timedTask;
}
