#pragma once

#include "safety/SafetyCommand.hpp"

#include <cstddef>
#include <mutex>
#include <queue>

namespace tcas::orchestrator
{

class CommandQueue
{
public:
    void push(const safety::SafetyCommand& command)
    {
        std::lock_guard lock(mutex_);
        queue_.push(command);
    }

    [[nodiscard]]
    bool tryPop(safety::SafetyCommand* command)
    {
        if (command == nullptr)
        {
            return false;
        }

        std::lock_guard lock(mutex_);
        if (queue_.empty())
        {
            return false;
        }

        *command = queue_.front();
        queue_.pop();
        return true;
    }

    [[nodiscard]]
    std::size_t size() const
    {
        std::lock_guard lock(mutex_);
        return queue_.size();
    }

    void clear()
    {
        std::lock_guard lock(mutex_);
        queue_ = {};
    }

private:
    mutable std::mutex mutex_;
    std::queue<safety::SafetyCommand> queue_;
};

} // namespace tcas::orchestrator
