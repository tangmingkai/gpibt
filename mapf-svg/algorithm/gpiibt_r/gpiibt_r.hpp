#pragma once
#include <string>
#include <vector>
#include <memory>

#include "graph.hpp"
#include "mapf-svg/algorithm/gpiibt_b/gpiibt_b.hpp"
#include "mapf-svg/algorithm/pibt_base.hpp"
namespace gpibt {
class GPIBT_R : public PIBT_Base {
 private:
  bool FuncGPIBT_R(Agent *ai, Agent *aj, bool is_back_flow);
  struct Neighbor {
    int dist;
    Node *node;
    Neighbor(int _dist, Node *_node) : dist(_dist), node(_node) {}
    Neighbor() : dist(-1), node(nullptr) {}
  };
  std::shared_ptr<GPIBT_B> gpiibt_b_;
  bool set_initial_solution_;
  bool CPSAT_v2();

 public:
  static const std::string SOLVER_NAME;
  GPIBT_R(MAPF_Instance *_P, bool _use_guidance, bool enable_swap_action,
         bool set_initial_solution, const std::string &_reduce_solver_name);
  virtual ~GPIBT_R() {}
  virtual bool ComputeOneStep();
  virtual void PreProcess() override;
};
}  // namespace gpibt

