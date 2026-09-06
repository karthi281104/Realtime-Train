#include "safety/ConflictPriorityQueue.hpp"

namespace tcas::safety
{

void ConflictPriorityQueue::push(
    const conflict::Conflict& conflict,
    const RiskAssessment& risk)
{
    queue_.push(PrioritizedConflict{ conflict, risk });
}

bool ConflictPriorityQueue::empty() const noexcept
{
    return queue_.empty();
}

std::size_t ConflictPriorityQueue::size() const noexcept
{
    return queue_.size();
}

PrioritizedConflict ConflictPriorityQueue::top() const
{
    return queue_.top();
}

void ConflictPriorityQueue::pop()
{
    queue_.pop();
}

void ConflictPriorityQueue::clear()
{
    queue_ = {};
}

bool ConflictPriorityQueue::Comparator::operator()(
    const PrioritizedConflict& lhs,
    const PrioritizedConflict& rhs) const noexcept
{
    if (lhs.risk.score != rhs.risk.score)
    {
        return lhs.risk.score < rhs.risk.score;
    }

    if (lhs.conflict.firstConflictTime
        != rhs.conflict.firstConflictTime)
    {
        return lhs.conflict.firstConflictTime
            > rhs.conflict.firstConflictTime;
    }

    if (lhs.conflict.trainA != rhs.conflict.trainA)
    {
        return lhs.conflict.trainA > rhs.conflict.trainA;
    }

    return lhs.conflict.trainB > rhs.conflict.trainB;
}

} // namespace tcas::safety