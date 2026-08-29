#pragma once

#include "common/Types.hpp"

#include <string>

namespace tcas::infrastructure
{

enum class NodeType
{
    Generic,
    Station,
    Junction,
    Platform
};

class Node
{
public:
    Node(
        NodeId id,
        std::string name,
        NodeType type
    );

    [[nodiscard]]
    NodeId id() const noexcept;

    [[nodiscard]]
    const std::string& name() const noexcept;

    [[nodiscard]]
    NodeType type() const noexcept;

private:
    NodeId id_;
    std::string name_;
    NodeType type_;
};

} // namespace tcas::infrastructure