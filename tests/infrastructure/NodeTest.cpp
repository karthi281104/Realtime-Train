#include <gtest/gtest.h>

#include "infrastructure/Node.hpp"

using namespace tcas;
using namespace tcas::infrastructure;

TEST(NodeTest, StoresNodeInformation) {
  Node node(1, "Central", NodeType::Station);

  EXPECT_EQ(node.id(), 1);
  EXPECT_EQ(node.name(), "Central");
  EXPECT_EQ(node.type(), NodeType::Station);
}

TEST(NodeTest, SupportsJunctionType) {
  Node node(10, "J1", NodeType::Junction);

  EXPECT_EQ(node.type(), NodeType::Junction);
}

TEST(NodeTest, SupportsPlatformType) {
  Node node(20, "P1", NodeType::Platform);

  EXPECT_EQ(node.type(), NodeType::Platform);
}