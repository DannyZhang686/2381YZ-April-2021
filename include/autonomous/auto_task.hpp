#ifndef AUTO_TASK_HPP
#define AUTO_TASK_HPP

#include <functional>
#include <vector>

#define Delay AutoTask::AutoDelay
typedef std::function<void(void)> runFn_t;
typedef std::function<bool(void)> doneFn_t;

class AutoTask {

    public:

    struct AutoTaskArgs {
        std::function<void(void)> task;
        std::function<void(void)> done;

        bool sync = true;
        std::function<void(void)> init = [](void) -> void {};
        std::function<void(void)> kill = [](void) -> void {};
    };


    struct AutoTaskVectorArgs {
        std::vector<std::function<void(void)>> runList;
        std::vector<std::function<bool(void)>> doneList;
        std::vector<std::function<void(void)>> initList = {};
        std::vector<std::function<void(void)>> killList = {};
        bool sync = true;
    };

    bool done(void);
    void initialize(void);
    void run(void);
    void kill(void);

    const std::vector<std::function<void(void)>> runList;
    const std::vector<std::function<void(void)>> killList;
    const std::vector<std::function<void(void)>> initList;
    const std::vector<std::function<bool(void)>> doneList;

    static AutoTask AsyncTask(std::function<void(void)> task, std::function<bool(void)> done, std::function<void(void)> init = [](void) -> void {}, std::function<void(void)> kill = [](void) -> void {});
    static AutoTask SyncTask(std::function<void(void)> task, std::function<bool(void)> done, std::function<void(void)> init = [](void) -> void {}, std::function<void(void)> kill = [](void) -> void {});
    static AutoTask AutoDelay(int interval, bool sync = true);


    AutoTask TimeLimit(int time);

    bool isSync;
    bool _initialized = false;

    AutoTask(std::function<void(void)> task, std::function<bool(void)> done, bool sync = true, std::function<void(void)> init = [](void)-> void {}, std::function<void(void)> kill = [](void)->void {});
    AutoTask(const AutoTaskVectorArgs vectorArgs);

    AutoTask AddRun(std::function<void(void)> task);
    AutoTask AddKill(std::function<void(void)> kill);
    AutoTask AddInit(std::function<void(void)> init);
    AutoTask AddDone(std::function<bool(void)> done);

};

#endif