#ifndef AUTO_TIMER_HPP
#define AUTO_TIMER_HPP

#include <functional>
#include "auto_task.hpp"

struct AutoTimerArgs {
    int interval;
    bool sync = true;
    std::function<void(void)> task = [](void) -> void {};
    std::function<void(void)> init = [](void) -> void {};
    std::function<void(void)> kill = [](void) -> void {};
    std::function<void(void)> done = [](void) -> void {};
};
class AutoTimer : public AutoTask
{
public:
    int _time = 0;
    int _duration = 1;
    AutoTimer(AutoTimerArgs args);
    AutoTimer(int interval, bool isSync = true);

    std::function<bool(void)> _done_check;
    bool _timer_done();
    void _run_increment();
    std::function<void(void)> _run_action;

    void _run();
protected:
};

#endif
