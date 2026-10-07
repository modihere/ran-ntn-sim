#include "simulator.h"

namespace ntn::common {

Time_ns Simulator::now() const {
    return current_time_;
}

void Simulator::schedule(Time_ns delay, std::function<void()> action) {
    Time_ns absolute_time = current_time_ + delay;
    events_.push(Event{absolute_time, ++next_event_id_, std::move(action)});
}

void Simulator::run(Time_ns duration) {
    is_running_ = true;
    Time_ns end_time = (duration > 0) ? (current_time_ + duration) : UINT64_MAX;

    while (is_running_ && !events_.empty()) {
        // Peek at the next event
        const Event& next_event = events_.top();

        // If the next event is strictly past our end_time, pause execution
        if (next_event.timestamp > end_time) {
            current_time_ = end_time;
            break;
        }

        // Advance the simulation clock to the event's timestamp
        current_time_ = next_event.timestamp;

        // Extract action and remove from queue
        // (Copying the function object because pop() destroys the top element)
        std::function<void()> action = next_event.action;
        events_.pop();

        // Execute the scheduled logic
        if (action) {
            action();
        }
    }

    if (events_.empty() && is_running_) {
        // If we drained the queue but had a specific duration, advance to the end_time
        if (duration > 0 && current_time_ < end_time) {
            current_time_ = end_time;
        }
        is_running_ = false;
    }
}

void Simulator::stop() {
    is_running_ = false;
}


Timer::Timer(Simulator& sim) : sim_(sim) {}

void Timer::start(Time_ns duration, std::function<void()> on_timeout) {
    is_running_ = true;
    
    // Incrementing the ID invalidates any previously scheduled timeout for this timer
    uint64_t timer_id = ++current_timer_id_;

    sim_.schedule(duration, [this, timer_id, on_timeout]() {
        // Only fire if the timer wasn't stopped or restarted in the meantime
        if (is_running_ && current_timer_id_ == timer_id) {
            is_running_ = false;
            if (on_timeout) {
                on_timeout();
            }
        }
    });
}

void Timer::stop() {
    is_running_ = false;
    // We increment the ID so that any pending lambda in the Simulator queue will see 
    // a mismatch and silently discard itself when it eventually gets popped.
    current_timer_id_++; 
}

bool Timer::is_running() const {
    return is_running_;
}

} // namespace ntn::common

