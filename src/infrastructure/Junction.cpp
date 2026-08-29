#include "infrastructure/Junction.hpp"

namespace tcas::infrastructure
{

Junction::Junction(JunctionId id)
    : id_(id)
{
}

JunctionId Junction::id() const noexcept
{
    return id_;
}

} // namespace tcas::infrastructure