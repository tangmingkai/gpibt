#include <cassert>
#include <cstddef>
#include <ostream>
#include <algorithm>
#include <fstream>
#include "gpiibt_b.hpp"

namespace gpibt {

const std::string GPIBT_B::SOLVER_NAME = "GPIBT-B";

GPIBT_B::GPIBT_B(MAPF_Instance *_P, bool _use_guidance, bool _enable_swap_action,
               bool _set_initial_solution)
    : PIBT_Base(_P, _use_guidance, _enable_swap_action, _set_initial_solution),
      private_MT_(_P->GetSeed()+1) {
  solver_name_ = GPIBT_B::SOLVER_NAME;
}

// std::ofstream fout("output_compared.txt");

bool GPIBT_B::ComputeOneStep() {
  // auto compare_agent = [&](const Agent *a, const Agent *b) {
  //   if (a->is_reached_goal != b->is_reached_goal)
  //     return a->is_reached_goal < b->is_reached_goal;

  //   // if (a->elapsed != b->elapsed) return a->elapsed > b->elapsed;
  //   // use initial distance
  //   return a->tie_breaker > b->tie_breaker;
  // };

  // std::sort(A_.begin(), A_.end(), compare_agent);
  SortAgentsByPrioirty(&A_);
  // if (time_step_ == 0) {
  //   fout << "*********************time_step " << time_step_
  //        << "*********************" << std::endl;
  //   for (auto a : A_) {
  //     fout << a->id << " " << LocationStr(a->v_now) << " " << LocationStr(a->g)
  //          << " " << LocationStr(a->current_target) << " " << a->is_reached_goal
  //          << " " << a->init_d << " " << a->tie_breaker << std::endl;
  //   }
  // }
  auto compare_neighbor = [&](const Neighbor &v, const Neighbor &u) {
    if (v.dist != u.dist) return v.dist < u.dist;
    // tie breakex
    if (occupied_now_[v.node->id] != nullptr &&
        occupied_now_[u.node->id] == nullptr)
      return false;
    if (occupied_now_[v.node->id] == nullptr &&
        occupied_now_[u.node->id] != nullptr)
      return true;
    // randomize
    return false;
  };

  agent_neighbors_.clear();
  agent_neighbors_.resize(A_.size());
  // chosen_neighbor_ids_.clear();
  // chosen_neighbor_ids_.resize(A_.size(), -1);
  for (auto &a : A_) {
    // get candidates
    Nodes C = a->v_now->neighbor;
    C.push_back(a->v_now);
    agent_neighbors_[a->id].reserve(C.size());
    for (auto &c : C) {
      auto dist = GetPIBTHeuristic(a, c);
      agent_neighbors_[a->id].push_back(Neighbor(dist, c));
    }
    // randomize
    std::shuffle(agent_neighbors_[a->id].begin(), agent_neighbors_[a->id].end(),
                 private_MT_);
    // sort
    std::sort(agent_neighbors_[a->id].begin(), agent_neighbors_[a->id].end(),
              compare_neighbor);
    // if (time_step_ == 0) {
    //   fout << "agent " << a->id << " " << LocationStr(a->v_now) << " ";
    //   for (auto c :agent_neighbors_[a->id]) {
    //     fout << LocationStr(c.node) << " ";
    //   }
    //   fout << std::endl;
    // }
    if (a->is_reached_goal) {
      for (auto &c : agent_neighbors_[a->id]) {
        c.dist = 0;
      }
    }
  }
  constant_agents_.clear();
  constant_agents_.resize(A_.size(), false);
  for (auto a : A_) {
    constant_agents_[a->id] = a->v_next != nullptr;
  }
  highest_priority_agent_ = A_.front();
  highest_priority_agent_id = A_.front()->id;
  attempted_neighbour_num.clear();
  attempted_neighbour_num.resize(A_.size(), 0);
  for (auto a : A_) {
    // puts("111");
    // if the agent has next location, then skip
    if (a->v_next == nullptr) {
      // determine its next location
      // bool is_limited_mode = P_->GetG()->getNodesSize() == P_->GetNum() &&
      //                         a == highest_priority_agent_;
      bool is_limited_mode = a == highest_priority_agent_;
      if (!FuncGPIBT_B(a, nullptr, false, is_limited_mode)) {
        return false;
      }
      // puts("\n\n");
      assert(a->v_next != nullptr);
    }
  }
  // if (time_step_ == 161) {
  //   std::cout << "highest_priority_agent_id:" << highest_priority_agent_id
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
  bool have_move = false;
  bool have_task = false;
  for (auto &a : A_) {
    if (a->is_reached_goal) {
      have_task = true;
    }
    if (a->v_next == nullptr) {
      // determine its next location
      // std::cout << "No next location " << a->id << " " << " (" <<
      // a->v_now->pos.x << ", " << a->v_now->pos.y << ")" << std::endl;
    } else {
      if (a->v_now != a->v_next) have_move = true;
    }
    assert(a->v_next != nullptr);
  }
    // if (time_step_ == 0) {
    // fout << "*********************time_step " << time_step_
    //      << "*********************" << std::endl;
    // for (auto a : A_) {
    //   fout << a->id << " " << LocationStr(a->v_now) << " " << LocationStr(a->v_next)
    //        << std::endl;
    // }
  // }
  // if (have_task && !have_move) {
  //   std::cout << "highest_priority_agent_id:" << highest_priority_agent_id
  //             << std::endl;
  //   for (auto &a : A) {
  //     int t = -1;
  //     if (a->task != nullptr) t = a->task->id;
  //     std::cout << a->id << " "
  //               << " (" << a->v_now->pos.x << ", " << a->v_now->pos.y << ") ->("
  //               << a->v_next->pos.x << ", " << a->v_next->pos.y << ") ..."
  //               << " (" << a->g->pos.x << ", " << a->g->pos.y << ")" << t
  //               << std::endl;
  //   }
  //   exit(0);
  // }
  // assert(!have_task || have_move);
  return true;
}

bool GPIBT_B::FuncGPIBT_B(Agent *ai, Agent *aj, bool is_back_flow,
                        bool is_limited_mode) {
  Agent *swap_agent = nullptr;
  if (enable_swap_action_ && attempted_neighbour_num[ai->id] == 0) {
    IsSwapRequiredAndPossible(ai, occupied_now_, occupied_next_,
                              agent_neighbors_[ai->id], &swap_agent);
    if (swap_agent != nullptr) {
        // reverse vertex scoring
      std::reverse(agent_neighbors_[ai->id].begin(),
      agent_neighbors_[ai->id].end());
    }
  }
  assert(aj == nullptr || aj->v_next == ai->v_now);
  // if (time_step_ == 0) {
  //     fout << "GPIBT_B Agent: " << ai->id << " " << LocationStr(ai->v_now) << " ";
  //     for (auto neighbor : agent_neighbors_[ai->id]) {
  //       fout << LocationStr(neighbor.node) << " ";
  //     }
  //     fout << std::endl;
  //   }
  for (; attempted_neighbour_num[ai->id] < agent_neighbors_[ai->id].size();) {
    auto &neighbor = agent_neighbors_[ai->id][attempted_neighbour_num[ai->id]];
    // std::cout << "trying " << LocationStr(ai->v_now) << " -> "
    //             << LocationStr(neighbor.node) << std::endl;
    if (is_back_flow) {
      int check_neighbor_id =
          attempted_neighbour_num[ai->id] - 1;  // must be non-negative
      // if (chosen_neighbor_ids_[ai->id] != -1)
      //   check_neighbor_id = chosen_neighbor_ids_[ai->id];
      auto &check_neighbor = agent_neighbors_[ai->id][check_neighbor_id];
      // int slack_para = 0;  //
      // int slack = ai->id == highest_priority_agent_id ? 0 : slack_para;
      if (check_neighbor.dist < neighbor.dist) {
        // puts("return false 2");
        return false;
      }
    }
    attempted_neighbour_num[ai->id]++;
    // if (aj != nullptr && neighbor.node == aj->v_now) {
    //   // puts("continue");
    //   continue;
    // }
    if (occupied_now_[neighbor.node->id] != nullptr &&
        occupied_now_[neighbor.node->id]->v_next == ai->v_now)
      continue;
    if (occupied_next_[neighbor.node->id] == nullptr) {
      // puts("Not backflow");
      // std::cout << neighbor.node->id << std::endl;
      occupied_next_[neighbor.node->id] = ai;
      ai->v_next = neighbor.node;
      // std::cout << "set1 " << LocationStr(ai->v_now) << " -> "
      //           << LocationStr(neighbor.node) << std::endl;
      auto ak = occupied_now_[neighbor.node->id];
      if (ak != nullptr && ak->v_next == nullptr) {
        if (!FuncGPIBT_B(ak, ai, false, is_limited_mode)) {
          // if (ai->v_next == neighbor.node) {
          //   // occupied_next[ai->v_next->id] = nullptr; // because the ak is
          //   // wait;
          //   ai->v_next = nullptr;
          //   // std::cout << "undo set2 " << LocationStr(ai->v_now) << " to -1"
          //   //           << std::endl;
            
          // }
          continue;
        }
      }
    } else {
      if (ai->is_reached_goal || is_limited_mode) {
        continue;
      }
      // puts("BackFlow");
      auto back_flow_agent = occupied_next_[neighbor.node->id];
      if (constant_agents_[back_flow_agent->id]) {
        continue;
      }
      assert(back_flow_agent == nullptr ||
             back_flow_agent->v_next == neighbor.node);
      // std::cout << "before 1 " << LocationStr(back_flow_agent->v_now) << " to "
      //           << LocationStr(back_flow_agent->v_next) << std::endl;
      occupied_next_[back_flow_agent->v_next->id] = nullptr;
      back_flow_agent->v_next = nullptr;
      // std::cout << "set3 " << LocationStr(back_flow_agent->v_now) << " to -1"
      //           << std::endl;
      occupied_next_[neighbor.node->id] = ai;
      ai->v_next = neighbor.node;
      // std::cout << "set4 " << LocationStr(ai->v_now) << " to "
      //           << LocationStr(neighbor.node) << std::endl;
      auto parent_back_flow_agent = occupied_next_[back_flow_agent->v_now->id];
      if (!FuncGPIBT_B(back_flow_agent, parent_back_flow_agent, true,
                      is_limited_mode)) {
        // if (ai->v_next == neighbor.node) {
        //   occupied_next[neighbor.node->id] = nullptr;
        //   ai->v_next = nullptr;
        // }
        // if (ai->v_next == neighbor.node) {
        occupied_next_[ai->v_next->id] = nullptr;
        ai->v_next = nullptr;
          // std::cout << "undo set5 " << LocationStr(ai->v_now) << " to -1"
          //           << std::endl;
        // }
        // occupied_next_[back_flow_agent->v_next->id] = nullptr;

        occupied_next_[back_flow_agent->v_now->id] =
              parent_back_flow_agent;
        back_flow_agent->v_next = neighbor.node;
        occupied_next_[back_flow_agent->v_next->id] = back_flow_agent;

        // std::cout << "undo set6 " << LocationStr(back_flow_agent->v_now)
        //           << " to (" << LocationStr(back_flow_agent->v_next) << ")"
        //           << std::endl;
        continue;
      }
    }
    // chosen_neighbor_ids_[ai->id] = attempted_neighbour_num[ai->id] - 1;
    if (enable_swap_action_ && attempted_neighbour_num[ai->id] == 1) {
      if (swap_agent != nullptr &&                 // swap_agent exists
        swap_agent->v_next == nullptr &&            // not decided
        occupied_next_[ai->v_now->id] == nullptr  // free
        ) {
        // pull swap_agent
        occupied_next_[ai->v_now->id] = swap_agent;
        swap_agent->v_next = ai->v_now;
      }
    }
    return true;
  }

  // failed to secure node
  // if (!is_back_flow) {
    ai->v_next = ai->v_now;
    // std::cout << "set7 " << LocationStr(ai->v_now) << " to "
    //           << LocationStr(ai->v_next) << std::endl;
    occupied_next_[ai->v_next->id] = ai;
    // chosen_neighbor_ids_[ai->id] = agent_neighbors_[ai->id].size();
  // }
  // puts("return false 2");
  return false;
}

void GPIBT_B::IsSwapRequiredAndPossible(Agent *agent_i,
                                       const Agents &occupied_now,
                                       const Agents &occupied_next,
                                       const Neighbors &C,
                                       Agent **swap_agent) {
  auto best_next_node = C[0].node;
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
          // agent_u->v_next == nullptr &&  // ?
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

bool GPIBT_B::IsSwapRequired(Agent *agent_pusher, Agent *agent_puller,
                                 Node *v_pusher_origin, Node *v_puller_origin) {
  auto v_pusher = v_pusher_origin;
  auto v_puller = v_puller_origin;
  Node *tmp = nullptr;
  while (PathDist(v_puller, agent_pusher->current_target) <
         PathDist(v_pusher, agent_pusher->current_target)) {
    auto n = v_puller->neighbor.size();
    // remove agents who need not to move
    for (auto u : v_puller->neighbor) {
      const auto agent_u = occupied_now_[u->id];
      if (u == v_pusher || (u->neighbor.size() == 1 && agent_u != nullptr &&
                            agent_u->current_target == u)) {
        --n;
      } else {
        tmp = u;
      }
    }
    if (n >= 2) return false;  // able to swap at v_l
    if (n <= 0) break;
    v_pusher = v_puller;
    v_puller = tmp;
  }

  return (PathDist(v_pusher, agent_puller->current_target) <
          PathDist(v_puller, agent_puller->current_target)) &&
         (PathDist(v_pusher, agent_pusher->current_target) == 0 ||
          PathDist(v_puller, agent_pusher->current_target) <
              PathDist(v_pusher, agent_pusher->current_target));
}

bool GPIBT_B::IsSwapPossible(Node *v_pusher_origin, Node *v_puller_origin) {
  // simulate pull
  auto v_pusher = v_pusher_origin;
  auto v_puller = v_puller_origin;
  Node *tmp = nullptr;
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
