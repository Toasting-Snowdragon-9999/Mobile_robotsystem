#include <chrono>
#include <atomic>
#include <thread>
#include "communication_protocol/physical_layer.h"

#define timeout_time std::chrono::seconds(5)

class Timer
{

private:
    std::atomic<bool> _timeout{false}; // Atomic bool to keep track of whether the timer has run out

public:
    Timer();

    ~Timer();

    /// @brief Timer that checks whether the timeout time has been reached
    /// @note Wrties true to private variable _timeout if timeout time has been reached
    void start_timer(PhysicalLayer *data);

    /// @brief Getter method for getting the timeout boolean
    /// @return Type - atomic bool
    bool get_timeout();

};