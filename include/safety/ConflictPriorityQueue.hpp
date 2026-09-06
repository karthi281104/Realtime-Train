#pragma once

#include "conflict/Conflict.hpp"
#include "safety/RiskEngine.hpp"

#include <cstddef>
#include <queue>
#include <vector>

namespace tcas::safety
{

struct PrioritizedConflict
{
    conflict::Conflict conflict;
    RiskAssessment risk;
};

class ConflictPriorityQueue
{
public:
    void push(
        const conflict::Conflict& conflict,
        const RiskAssessment& risk
    );

    [[nodiscard]]
    bool empty() const noexcept;

    [[nodiscard]]
    std::size_t size() const noexcept;

    [[nodiscard]]
    PrioritizedConflict top() const;

    void pop();

    void clear();

private:
    struct Comparator
    {
        [[nodiscard]]
        bool operator()(
            const PrioritizedConflict& lhs,
            const PrioritizedConflict& rhs
        ) const noexcept;
    };

    std::priority_queue<
        PrioritizedConflict,
        std::vector<PrioritizedConflict>,
        Comparator
    > queue_;
};

} // namespace tcas::safety