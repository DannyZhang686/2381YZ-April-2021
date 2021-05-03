#include "main.h"
#include "autonomous/auto_timer.hpp"
#include "globals.hpp"

AutoTimer::AutoTimer(AutoTimerArgs args)
    : AutoTask{
          [&](void) -> void {
            _run_action();
            _run_increment();
          },
          [&](void) -> bool {

            return (_time >= _duration); }, args.sync, args.init, args.kill},
      _duration(args.interval), _run_action(args.task)
{
}
void AutoTimer::_run_increment()
{
  _time += DELAY_INTERVAL;
}
AutoTimer::AutoTimer(int duration, bool sync)
    : AutoTask{
          [&](void) -> void {
            _run_increment();
          },
          [&](void) -> bool { return (_timer_done()); }, sync},
      _duration(duration)
      
{
}
bool AutoTimer::_timer_done()
{
  return ((_time >= _duration));
}