#include "pibt.hpp"

#include <algorithm>
#include <cstddef>
#include <string>
#include <fstream>

namespace gpibt {

const std::string PIBT::SOLVER_NAME = "PIBT";

PIBT::PIBT(MAPF_Instance* _P, bool _use_guidance, bool _enable_swap_action,
           bool _set_initial_solution)
    : PIBT_Base(_P, _use_guidance, _enable_swap_action, _set_initial_solution),
      private_MT_(_P->GetSeed()+1) {
  solver_name_ = PIBT::SOLVER_NAME;
  enable_swap_action_ = _enable_swap_action;
}

bool is_output = false;

bool PIBT::ComputeOneStep() {
  // static int time_step = 0;
  // auto compare = [&](Agent* a, const Agent* b) {
  //   if (a->is_reached_goal != b->is_reached_goal)
  //     return a->is_reached_goal < b->is_reached_goal;
  //   // if (a->elapsed != b->elapsed) return a->elapsed > b->elapsed;
  //   // use initial distance
  //   if (a->init_d != b->init_d) return a->init_d > b->init_d;
  //   return a->tie_breaker > b->tie_breaker;
  // };
  // std::sort(A_.begin(), A_.end(), compare);
  SortAgentsByPrioirty(&A_);
  highest_priority_agent_ = A_.front();
  for (auto a : A_) {
    // if the agent has next location, then skip
    if (a->v_next == nullptr) {
      // puts("---start---");
      // determine its next location
      if (!FuncPIBT(a)) {
        return false;
      }
    }
    is_output = false;
  }
  // if (time_step_ == 2101) {
  //   std::cout << "highest_priority_agent_id:" << highest_priority_agent_->id
  //             << std::endl;
  //   for (auto &a : A_) {
  //     int t = -1;
  //     // if (a->task != nullptr) t = a->task->id;
  //     std::cout << a->id << " "
  //               << " (" << a->v_now->pos.x << ", " << a->v_now->pos.y << ") ->("
  //               << a->v_next->pos.x << ", " << a->v_next->pos.y << ") ...";
  //     if (!a->is_reached_goal) {
  //       std::cout << " (" << a->g->pos.x << ", " << a->g->pos.y << ")";
  //     }
  //     std::cout << std::endl;
  //   }
  //   // exit(0);
  // }

  
  // puts("********************Finish one step*********************");
  return true;
}

bool PIBT::FuncPIBT(Agent* ai, Agent* aj) {
  // compare two nodes
  auto compare = [&](Node* const v, Node* const u) {
    int d_v = GetPIBTHeuristic(ai, v);
    int d_u = GetPIBTHeuristic(ai, u);
    if (d_v != d_u) return d_v < d_u;
    // tie break
    if (occupied_now_[v->id] != nullptr && occupied_now_[u->id] == nullptr)
      return false;
    if (occupied_now_[v->id] == nullptr && occupied_now_[u->id] != nullptr)
      return true;
    return false;
  };

  // get candidates
  Nodes C = ai->v_now->neighbor;
  C.push_back(ai->v_now);
  // randomize
  std::shuffle(C.begin(), C.end(), private_MT_);
  // sort
  std::sort(C.begin(), C.end(), compare);
  // emulate swap
  Agent* swap_agent = nullptr;
  if (enable_swap_action_) {
    IsSwapRequiredAndPossible(ai, occupied_now_, occupied_next_, C,
                              &swap_agent);
    if (swap_agent != nullptr) {
      std::reverse(C.begin(), C.end());
    }
  }

  auto swap_operation = [&]() {
    if (swap_agent != nullptr &&                 // swap_agent exists
        swap_agent->v_next == nullptr &&         // not decided
        occupied_next_[ai->v_now->id] == nullptr  // free
    ) {
      // pull swap_agent
      occupied_next_[ai->v_now->id] = swap_agent;
      swap_agent->v_next = ai->v_now;
    }
  };

  for (size_t k = 0; k < C.size(); ++k) {
    auto u = C[k];
    // avoid conflicts
    if (is_output) {
      std::cout << "try " << LocationStr(ai->v_now) + "->" + LocationStr(u)
                << std::endl;
    }
    if (occupied_next_[u->id] != nullptr) continue;
    // if (j != NO_AGENT && Q_to[j] == Q_from[i]) continue;
    if (occupied_now_[u->id] != nullptr &&
        occupied_now_[u->id]->v_next == ai->v_now)
      continue;
    // if (aj != nullptr && u == aj->v_now) continue;
    
    // reserve
    occupied_next_[u->id] = ai;
    ai->v_next = u;
    if (is_output) {
      std::cout << "set (" << ai->v_now->pos.x << " " << ai->v_now->pos.y
                << ")->(" << u->pos.x << " " << u->pos.y << ")" << std::endl;
    }
    auto ak = occupied_now_[u->id];
    if (ak != nullptr && ak->v_next == nullptr) {
      if (is_output) {
        std::cout << "inhert " << LocationStr(ak->v_now) << std::endl;
      }
      if (!FuncPIBT(ak, ai)) {
        if (is_output) {
          std::cout << "fail" << std::endl;
        }
        // std::cout << "fail" << std::endl;
        continue;  // replanning
      }
    }
    // success to plan next one step
    if (enable_swap_action_ && k == 0) {
      swap_operation();
      if (swap_agent != nullptr) {
        // std::cout << "pull swap agent (" << swap_agent->v_now->pos.x << " "
        // << swap_agent->v_now->pos.y << ") <-> (" << swap_agent->v_next->pos.x
        // << " " << swap_agent->v_next->pos.y << ")" << std::endl;
      }
    }
    return true;
  }

  // failed to secure node
  occupied_next_[ai->v_now->id] = ai;
  ai->v_next = ai->v_now;
  return false;
}

void PIBT::IsSwapRequiredAndPossible(Agent* agent_i, const Agents& occupied_now,
                                     const Agents& occupied_next,
                                     const Nodes& C, Agent** swap_agent) {
  auto best_next_node = C[0];
  // agent-j occupying the desired vertex for agent-i
  const auto agent_j = occupied_now[best_next_node->id];
  if (agent_j != nullptr && agent_j != agent_i &&  // j exists
      agent_j->v_next == nullptr &&  // j does not decide next location
      IsSwapRequired(agent_i, agent_j, agent_i->v_now,
                     agent_j->v_now) &&               // swap required
      IsSwapPossible(agent_j->v_now, agent_i->v_now)  // swap possible
  ) {
    *swap_agent = agent_j;
    return;
  }

  // for clear operation, c.f., push & swap
  if (best_next_node != agent_i->v_now) {
    for (auto u : agent_i->v_now->neighbor) {
      const auto agent_u = occupied_now[u->id];
      if (agent_u != nullptr &&   // k exists
          best_next_node != u &&  // this is for clear operation
          IsSwapRequired(agent_u, agent_i, agent_i->v_now,
                         best_next_node) &&  // emulating from one step ahead
          IsSwapPossible(best_next_node, agent_i->v_now)) {
        *swap_agent = agent_u;
        return;
      }
    }
  }
  *swap_agent = nullptr;
}

bool PIBT::IsSwapRequired(Agent* agent_pusher, Agent* agent_puller,
                          Node* v_pusher_origin, Node* v_puller_origin) {
  // std::cout << "  agent_pusher: (" << agent_pusher->v_now->pos.x << " " <<
  // agent_pusher->v_now->pos.y << ") -> (" << agent_pusher->g->pos.x << " " <<
  // agent_pusher->g->pos.y << ")" << std::endl; std::cout << "  agent_puller:
  // (" << agent_puller->v_now->pos.x << " " << agent_puller->v_now->pos.y << ")
  // -> (" << agent_puller->g->pos.x << " " << agent_puller->g->pos.y << ")" <<
  // std::endl;
  auto v_pusher = v_pusher_origin;
  auto v_puller = v_puller_origin;
  Node* tmp = nullptr;
  while (PathDist(v_puller, agent_pusher->current_target) <
         PathDist(v_pusher, agent_pusher->current_target)) {
    // puts("IsSwapRequired: pull");
    // std::cout << "v_pusher: " << v_pusher->pos.x << " " << v_pusher->pos.y <<
    // std::endl; std::cout << "v_puller: " << v_puller->pos.x << " " <<
    // v_puller->pos.y << std::endl;
    auto n = v_puller->neighbor.size();
    // remove agents who need not to move
    for (auto u : v_puller->neighbor) {
      // std::cout << "nei: (" << u->pos.x << " " << u->pos.y << ")" <<
      // std::endl;
      const auto agent_u = occupied_now_[u->id];
      if (u == v_pusher || (u->neighbor.size() == 1 && agent_u != nullptr &&
                            agent_u->current_target == u)) {
        --n;
        // std::cout << "remove: (" << u->pos.x << " " << u->pos.y << ")" <<
        // std::endl;
      } else {
        tmp = u;
      }
    }
    if (n >= 2) return false;  // able to swap at v_l
    if (n <= 0) break;
    v_pusher = v_puller;
    v_puller = tmp;
  }
  //  puts("IsSwapRequired: pull finish");

  // std::cout << "  agent_pusher: (" << agent_pusher->v_now->pos.x << " " <<
  // agent_pusher->v_now->pos.y << ") -> (" << agent_pusher->g->pos.x << " " <<
  // agent_pusher->g->pos.y << ")" << std::endl; std::cout << "  agent_puller:
  // (" << agent_puller->v_now->pos.x << " " << agent_puller->v_now->pos.y << ")
  // -> (" << agent_puller->g->pos.x << " " << agent_puller->g->pos.y << ")" <<
  // std::endl; std::cout << "  v_pusher: " << v_pusher->pos.x << " " <<
  // v_pusher->pos.y << std::endl; std::cout << "  v_puller: " <<
  // v_puller->pos.x << " " << v_puller->pos.y << std::endl; bool boo1 =
  // pathDist(v_pusher, agent_puller->g) <
  //         pathDist(v_puller, agent_puller->g);
  // bool boo2 = pathDist(v_pusher, agent_pusher->g) == 0;
  // bool boo3 = pathDist(v_puller, agent_pusher->g) <
  //         pathDist(v_pusher, agent_pusher->g);
  // std::cout << boo1 << " " << boo2 << " " << boo3 << std::endl;
  return (PathDist(v_pusher, agent_puller->current_target) <
          PathDist(v_puller, agent_puller->current_target)) &&
         (PathDist(v_pusher, agent_pusher->current_target) == 0 ||
          PathDist(v_puller, agent_pusher->current_target) <
              PathDist(v_pusher, agent_pusher->current_target));
}

bool PIBT::IsSwapPossible(Node* v_pusher_origin, Node* v_puller_origin) {
  // simulate pull
  auto v_pusher = v_pusher_origin;
  auto v_puller = v_puller_origin;
  Node* tmp = nullptr;
  while (v_puller != v_pusher_origin) {  // avoid loop
    auto n = v_puller->neighbor.size();
    for (auto u : v_puller->neighbor) {
      const auto agent_u = occupied_now_[u->id];
      if (u == v_pusher || (u->neighbor.size() == 1 && agent_u != nullptr &&
                            agent_u->current_target == u)) {
        --n;
      } else {
        tmp = u;
      }
    }
    if (n >= 2) return true;  // able to swap at v_next
    if (n <= 0) return false;
    v_pusher = v_puller;
    v_puller = tmp;
  }
  return false;
}

}  // namespace gpibt

