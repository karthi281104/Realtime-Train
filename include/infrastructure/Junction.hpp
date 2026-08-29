#pragma once

#include "common/Types.hpp"

namespace tcas::infrastructure
{

class Junction
{
public:
    explicit Junction(JunctionId id);

    [[nodiscard]]
    JunctionId id() const noexcept;

private:
    JunctionId id_;
};

} // namespace tcas::infrastructure