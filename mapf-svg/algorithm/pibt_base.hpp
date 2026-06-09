#pragma once
#include <utility>
#include <vector>

#include "node.hpp"
#include "solver.hpp"

namespace gpibt {
class PIBT_Base : public MAPF_Solver {
 public:
  // virtual static const std::string SOLVER_NAME;
  PIBT_Base(MAPF_Instance* _P, bool _use_guidance, bool _enable_swap_action,
            bool _set_initial_solution);
  virtual ~PIBT_Base() {}

  struct Agent {
    int id;
    Node* v_now;        // current location
    Node* v_next;       // next location
    Node* g;            // goal
    int elapsed;        // eta
    int init_d;
    double tie_breaker;  // epsilon, tie-breaker
    Node *current_target;
    bool is_reached_goal;
  };
  using Agents = std::vector<Agent*>;
  virtual bool SetAndcomputeOneStep(Agents* A, int _time_step);
  int GetCurrentPotential(const Agents& A);
  int GetNextPotential(const Agents& A);
  int GetPotentialGain(const Agents& A);
  double GetObjectiveGain(const Agents& A,
                          const Agent* highest_priority_agent);
  double GetWeightedObjectiveGain(const Agents& A,
                          const Agent* highest_priority_agent);
  // double GetNextObjective(const Agents& A,
  //                 const Agent* highest_priority_agent);
  static bool CompareTwoAgents(const Agent *a, const Agent *b);
  void SortAgentsByPrioirty(Agents *A);
  void SortAgentsByID(Agents *A);
  Agent* HighestPriorityAgent(Agents* A);
  Agent* GetCurrentHighestPriorityAgent() { return highest_priority_agent_; }
  Graph* GetG() { return P_->GetG(); }
 protected:
  // PIBT agent

  Agents A_;
  Agent* highest_priority_agent_;
  // <node-id, agent>, whether the node is occupied or not
  // work as reservation table
  Agents occupied_now_;
  Agents occupied_next_;

  // result of priority inheritance: true -> valid, false -> invalid
  // bool funcPIBT(Agent* ai, Agent* aj = nullptr);
  virtual bool ComputeOneStep() = 0;
  // main
  void Run();

  int max_guidance_length_;
  std::vector<std::vector<std::pair<Node*, int>>> guidance_paths_;
  struct HeuristicTableElement {
    int dis2guidance;
    int guidance2goal;
    bool is_closed;
    HeuristicTableElement() = default;
    HeuristicTableElement(int _dis2guidance, int _guidance2goal,
                          bool _is_closed)
        : dis2guidance(_dis2guidance),
          guidance2goal(_guidance2goal),
          is_closed(_is_closed) {}
  };
  std::vector<std::vector<HeuristicTableElement>> heuristic_table_;
  std::vector<std::vector<Node*>> bfs_que_;
  std::vector<int> bfs_head_;
  std::vector<std::vector<int>> edge_occupy_;
  std::vector<int> node_occupy_;

  std::vector<bool> reached_goal_;
  int time_step_;

  const int infinity_ = 2147483647 / 2;
  void UpdateHeuristicTable(Agent* ai, Node* vj);
  void InitHeuristicTableSearch(
      Agent* ai, const std::vector<std::pair<Node*, int>>& guidance_path);
  void InitGuidance(const std::vector<Agent*>& agents);
  void RandomSelect(int num, std::vector<Agent*>* agents);
  void Refine(const std::vector<Agent*>& updated_agents);
  void Replan(const std::vector<Agent*>& updated_agents);
  void SearchGuidancePath(Agent* agent, const std::vector<int>& node_occupy,
                          const std::vector<std::vector<int>>& edge_occupy,
                          std::vector<std::pair<Node*, int>>* guidance_path);
  int GetPIBTHeuristic(Agent* ai, Node* vj);
  struct NodeState {
    Node* node;
    int g;
    int h;
    int f;
    std::pair<int, int> cost;
    double tie_breaker;
    NodeState* parent;
    int parent_neighbor_id;
    bool closed;

    NodeState(Node* n, int _g, int _h, const std::pair<int, int>& _cost,
              double tie_breaker)
        : node(n),
          g(_g),
          h(_h),
          f(_g + _h),
          cost(_cost),
          tie_breaker(tie_breaker),
          parent(nullptr),
          parent_neighbor_id(-1),
          closed(false) {}
  };

  struct CompareF {
    bool operator()(const NodeState* a, const NodeState* b) {
      if (a->f != b->f) return a->f > b->f;
      if (a->g != b->g) return a->g < b->g;
      return a->tie_breaker > b->tie_breaker;
    }
  };

  struct CompareJam {
    bool operator()(const NodeState* a, const NodeState* b) {
      if (a->cost != b->cost) return a->cost > b->cost;
      return a->tie_breaker > b->tie_breaker;
    }
  };

  bool CanReplace(const int new_g, const std::pair<int, int>& new_cost,
                  const double& new_tie_breaker, const NodeState* existing) {
    if (existing->closed) return false;
    if (new_g != existing->g) return new_g < existing->g;
    if (new_cost != existing->cost) return new_cost < existing->cost;
    return new_tie_breaker < existing->tie_breaker;
  }

};
};  // namespace gpibt

