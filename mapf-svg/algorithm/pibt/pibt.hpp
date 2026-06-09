#pragma once
#include "mapf-svg/algorithm/pibt_base.hpp"

namespace gpibt {
class PIBT : public PIBT_Base {
 private:
  virtual bool ComputeOneStep();
  bool FuncPIBT(Agent *ai, Agent *aj = nullptr);
  void IsSwapRequiredAndPossible(Agent *agent_i, const Agents &occupied_now,
                                 const Agents &occupied_next, const Nodes &C,
                                 Agent **swap_agent);
  bool IsSwapRequired(Agent *agent_pusher, Agent *agent_puller,
                      Node *v_pusher_origin, Node *v_puller_origin);
  bool IsSwapPossible(Node *v_pusher_origin, Node *v_puller_origin);
  std::mt19937 private_MT_;

 public:
  static const std::string SOLVER_NAME;
  PIBT(MAPF_Instance *_P, bool _use_guidance, bool _enable_swap_action,
       bool _set_initial_solution);
  virtual ~PIBT() {}
 

};
}  // namespace gpibt
