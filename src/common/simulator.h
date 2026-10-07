#pragma once

#include <cstdint>
#include <functional>
#include <queue>
#include <vector>

namespace ntn::common {

// Simulation time represented in integer nanoseconds
using Time_ns = uint64_t;

// A lightweight wrapper for an executable action
struct Event {
    Time_ns timestamp;
    uint64_t event_id; // Tie-breaker for stable ordering of simultaneous events
    std::function<void()> action;

    // Min-heap requires greater-than comparison for the smallest timestamp to be on top
    bool operator>(const Event& other) const {
        if (timestamp == other.timestamp) {
            return event_id > other.event_id;
        }
        return timestamp > other.timestamp;
    }
};

// Central discrete-event simulation engine
class Simulator {
public:
    Simulator() = default;
    ~Simulator() = default;

    // Disallow copy/move to maintain a stable orchestrator
    Simulator(const Simulator&) = delete;
    Simulator& operator=(const Simulator&) = delete;

    // Returns the current simulation time in nanoseconds
    Time_ns now() const;

    // Schedules an action to be executed after a specified delay
    void schedule(Time_ns delay, std::function<void()> action);

    // Runs the simulation either until the queue is empty or until a specific duration has passed.
    // If duration is 0, it runs until the queue is exhausted.
    void run(Time_ns duration = 0);

    // Stops the execution loop safely
    void stop();

private:
    Time_ns current_time_{0};
    uint64_t next_event_id_{0};
    bool is_running_{false};

    std::priority_queue<Event, std::vector<Event>, std::greater<Event>> events_;
};

// A cancelable timer abstraction built on top of the Simulator
class Timer {
public:
    explicit Timer(Simulator& sim);

    // Starts the timer. If it was already running, it restarts it.
    void start(Time_ns duration, std::function<void()> on_timeout);

    // Stops the timer to prevent the callback from firing
    void stop();

    // Returns true if the timer is actively counting down
    bool is_running() const;

private:
    Simulator& sim_;
    uint64_t current_timer_id_{0};
    bool is_running_{false};
};

} // namespace ntn::common
