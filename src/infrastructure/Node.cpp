#include "infrastructure/Node.hpp"

#include <utility>

namespace tcas::infrastructure {

Node::Node(NodeId id, std::string name, NodeType type)
    : id_(id), name_(std::move(name)), type_(type) {}

NodeId Node::id() const noexcept { return id_; }

const std::string& Node::name() const noexcept { return name_; }

NodeType Node::type() const noexcept { return type_; }

}  // namespace tcas::infrastructure