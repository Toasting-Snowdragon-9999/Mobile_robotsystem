#include "communication_protocol/timer.h"

void Timer::start_timer()
{
    auto start = std::chrono::steady_clock::now(); // start timer

    while (1)
    {
        auto now = std::chrono::steady_clock::now(); // checks current time

        // if timer has run out set timeout to true
        if (now - start >= timeout_time)
        {
            _timeout.store(true);
            break;
        }
    }
}

bool Timer::get_timeout()
{
    return _timeout.load();
}