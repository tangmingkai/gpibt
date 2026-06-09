#pragma once

#include <graph.hpp>
namespace gpibt {
struct Edge {
  Node* start;
  Node* end;
  bool operator==(const Edge& other) const {
    return start == other.start && end == other.end;
  }
};
};  // namespace gpibt
