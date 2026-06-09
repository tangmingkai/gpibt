#pragma once
#include "graph.hpp"
#include <vector>
#include <string>
#include "mapf-svg/algorithm/pibt_base.hpp"
namespace gpibt {
class GPIBT_B : public PIBT_Base {
 private:
  bool FuncGPIBT_B(Agent *ai, Agent *aj, bool is_back_flow,
                  bool is_limited_mode);
  int highest_priority_agent_id;
  std::vector<int> attempted_neighbour_num;
  struct Neighbor {
    int dist;
    Node *node;
    Neighbor(int _dist, Node *_node) : dist(_dist), node(_node) {}
    Neighbor() : dist(-1), node(nullptr) {}
  };
  using Neighbors = std::vector<Neighbor>;
  std::vector<Neighbors> agent_neighbors_;
  std::vector<int> chosen_neighbor_ids_;
  void IsSwapRequiredAndPossible(Agent *agent_i,
                                       const Agents &occupied_now,
                                       const Agents &occupied_next,
                                       const Neighbors &C,
                                       Agent **swap_agent);
  bool IsSwapRequired(Agent *agent_pusher, Agent *agent_puller,
                                 Node *v_pusher_origin, Node *v_puller_origin);
  bool IsSwapPossible(Node *v_pusher_origin, Node *v_puller_origin);
  std::mt19937 private_MT_;
  std::vector<bool> constant_agents_;
 public:
  static const std::string SOLVER_NAME;
  GPIBT_B(MAPF_Instance *_P, bool _use_guidance, bool _enable_swap_action,
         bool _set_initial_solution);
  virtual ~GPIBT_B() {}
  virtual bool ComputeOneStep();
};
}  // namespace gpibt

