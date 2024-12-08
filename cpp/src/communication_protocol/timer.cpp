#include "communication_protocol/timer.h"

Timer::Timer() : _timeout(false) {}

Timer::~Timer(){}


// void Timer::start_timer()
// {
//     auto start = std::chrono::steady_clock::now(); // start timer

//     while (1)
//     {
//         auto now = std::chrono::steady_clock::now(); // checks current time

//         // if timer has run out set timeout to true
//         if (now - start >= timeout_time)
//         {
//             _timeout.store(true);
//             break;
//         }
//     }
//     std::this_thread::sleep_for(std::chrono::milliseconds(20));
// }

void Timer::start_timer(PhysicalLayer *data)
{
    // Reset the timeout flag before starting
    _timeout.store(false);

    // Launch a detached thread
    std::thread([this, data]()
                {
            auto start = std::chrono::steady_clock::now();

            // Sleep or busy-check until the timeout is reached
            while (true) {
                auto now = std::chrono::steady_clock::now();
                if (now - start >= timeout_time) {
                    _timeout.store(true); // Signal that the timeout occurred
                    data->stop_audio(true); // Signal the audio input to stop
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Prevent busy waiting
            } })
        .detach(); // Detach the thread
}

bool Timer::get_timeout()
{
    return _timeout.load();
}