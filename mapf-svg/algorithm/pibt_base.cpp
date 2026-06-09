#include "pibt_base.hpp"
#include "util.hpp"
#include <set>
#include <queue>
#include <unordered_map>
#include <cassert>
#include <algorithm>
#include <fstream>
// const std::string PIBT_Base::SOLVER_NAME = "PIBT_Base";

namespace gpibt {
PIBT_Base::PIBT_Base(MAPF_Instance* _P, bool _use_guidance,
                     bool _enable_swap_action, bool _set_initial_solution)
    : MAPF_Solver(_P, _use_guidance, _enable_swap_action,
                  _set_initial_solution),
      occupied_now_(Agents(G_->getNodesSize(), nullptr)),
      occupied_next_(Agents(G_->getNodesSize(), nullptr)) {
  // solver_name = PIBT_Base::SOLVER_NAME;
  max_guidance_length_ = 1;
}

bool PIBT_Base::SetAndcomputeOneStep(Agents* _A, int _time_step) {
  bool succ = true;
  time_step_ = _time_step;
  if (is_preprocess_ == false) PreProcess();

  A_ = *_A;
  for (auto a : A_) {
    occupied_now_[a->v_now->id] = a;
    if (a->v_next != nullptr) {
      if (occupied_next_[a->v_next->id] != nullptr) {
        succ = false;
        break;
      }
      auto b = occupied_now_[a->v_next->id];
      if (b != nullptr && b != a && b->v_next == a->v_now) {
        succ = false;
        break;
      }
      occupied_next_[a->v_next->id] = a;
    }
  }
  if (succ)
    succ = ComputeOneStep();
  for (auto a : A_) {
    occupied_now_[a->v_now->id] = nullptr;
    if (a->v_next != nullptr) {
      occupied_next_[a->v_next->id] = nullptr;
    }
  }
  if (succ) {
    bool is_error = false;
    for (int i = 0; i < A_.size(); i++) {
      for (int j = i + 1; j < A_.size(); j++) {
        if (A_[i]->v_next == A_[j]->v_next) {
          puts("has vertex conflict");
          std::cout << A_[i]->id << " " << LocationStr(A_[i]->v_now) << " -> " << LocationStr(A_[i]->v_next) << std::endl;
          std::cout << A_[j]->id << " " << LocationStr(A_[j]->v_now) << " -> " << LocationStr(A_[j]->v_next) << std::endl;
          is_error = true;
        }
        if (A_[i]->v_next == A_[j]->v_now && A_[j]->v_next == A_[i]->v_now) {
          puts("has edge conflict");
          std::cout << A_[i]->id << " " << LocationStr(A_[i]->v_now) << " -> " << LocationStr(A_[i]->v_next) << std::endl;
          std::cout << A_[j]->id << " " << LocationStr(A_[j]->v_now) << " -> " << LocationStr(A_[j]->v_next) << std::endl;
          is_error = true;
        }
      }
    }
    if (is_error) {
      puts("");
      for (auto &a:A_){
        std::cout << a->id << " " << LocationStr(a->v_now) << " -> " << LocationStr(a->v_next) << std::endl;
      }
    }
    assert(!is_error);
  }
  return succ;
}


double PIBT_Base::GetObjectiveGain(const Agents& A,
                                const Agent* highest_priority_agent) {
  double res = 0;
  // std::cout << std::endl;
  for (auto &a : A) {
    if (a->is_reached_goal) continue;
    int priority_weight = highest_priority_agent == a
                              ? A.size() * max_guidance_length_ * 2 * 2
                              : 1;
    int dist_gain =
        GetPIBTHeuristic(a, a->v_now) - GetPIBTHeuristic(a, a->v_next);
    // double tie_breaking_weight = a->tie_breaker;
    int tie_breaking_weight = 1;
    // std::cout << a->id << ": " << priority_weight << " " << dist_gain << " "
    //           << tie_breaking_weight << " "
    //           << priority_weight * dist_gain * tie_breaking_weight << " "
    //           << max_guidance_length_ << std::endl;
    res += priority_weight * dist_gain * tie_breaking_weight;
  }
  return res;
}

double PIBT_Base::GetWeightedObjectiveGain(const Agents& A,
                                const Agent* highest_priority_agent) {
  double res = 0;
  // std::cout << std::endl;
  for (auto &a : A) {
    if (a->is_reached_goal) continue;
    int priority_weight = highest_priority_agent == a
                              ? A.size() * max_guidance_length_ * 2 * 2
                              : 1;
    int dist_gain =
        GetPIBTHeuristic(a, a->v_now) - GetPIBTHeuristic(a, a->v_next);
    double tie_breaking_weight = a->tie_breaker;
    // int tie_breaking_weight = 1;
    // std::cout << a->id << ": " << priority_weight << " " << dist_gain << " "
    //           << tie_breaking_weight << " "
    //           << priority_weight * dist_gain * tie_breaking_weight << " "
    //           << max_guidance_length_ << std::endl;
    res += priority_weight * dist_gain * tie_breaking_weight;
  }
  return res;
}

// double PIBT_Base::GetNextObjective(const Agents& A,
//                                 const Agent* highest_priority_agent) {
//   double res = 0;
//   // std::cout << std::endl;
//   for (auto &a : A) {
//     if (a->is_reached_goal) continue;
//     int priority_weight = highest_priority_agent == a
//                               ? A.size() * max_guidance_length_ * 2 * 2
//                               : 1;
//     int dist_next =
//         GetPIBTHeuristic(a, a->v_next);
//     // double tie_breaking_weight = a->tie_breaker;
//     int tie_breaking_weight = 1;
//     // std::cout << a->id << ": " << priority_weight << " " << dist_gain << " "
//     //           << tie_breaking_weight << " "
//     //           << priority_weight * dist_gain * tie_breaking_weight << " "
//     //           << max_guidance_length_ << std::endl;
//     res += priority_weight * dist_next * tie_breaking_weight;
//   }
//   return res;
// }

int PIBT_Base::GetCurrentPotential(const Agents& A) {
  int res = 0;
  for (auto a : A) {
    if (a->is_reached_goal) continue;
    res += PathDist(a->v_now, a->g);
  }
  return res;
}

int PIBT_Base::GetNextPotential(const Agents& A) {
  int res = 0;
  for (auto a : A) {
    if (a->is_reached_goal) continue;
    res += PathDist(a->v_next, a->g);
  }
  return res;
}

int PIBT_Base::GetPotentialGain(const Agents& A) {
  return GetCurrentPotential(A)- GetNextPotential(A);
}

bool PIBT_Base::CompareTwoAgents(const Agent* a, const Agent* b) {
  bool a_next_determined = a->v_next != nullptr;
  bool b_next_determined = b->v_next != nullptr;
  if (a_next_determined != b_next_determined)
    return a_next_determined < b_next_determined;
  if (a->is_reached_goal != b->is_reached_goal)
    return a->is_reached_goal < b->is_reached_goal;
  // int d_v = PathDist(a->v_now, a->current_target);
  // int d_u = PathDist(b->v_now, b->current_target);
  // if (d_v != d_u) return d_v < d_u;
  if (a->init_d != b->init_d) return a->init_d > b->init_d;
  // if (a->elapsed != b->elapsed) return a->elapsed > b->elapsed;
  // use initial distance
  return a->tie_breaker > b->tie_breaker;
}
void PIBT_Base::SortAgentsByPrioirty(Agents *A) {
  std::sort(A->begin(), A->end(), CompareTwoAgents);
}
void PIBT_Base::SortAgentsByID(Agents *A) {
  std::sort(A->begin(), A->end(),
         [](const Agent* a1, const Agent* a2) { return a1->id < a2->id; });
}

PIBT_Base::Agent* PIBT_Base::HighestPriorityAgent(Agents* A) {
  return *std::min_element(A->begin(), A->end(),
                           CompareTwoAgents);
}

void PIBT_Base::Run() {
  time_step_ = 0;
  A_.clear();
  occupied_now_.clear();
  occupied_now_.resize(G_->getNodesSize(), nullptr);
  // initialize
  tie_breakers_.clear();
  double step = 1.0 / (2.0 * P_->GetNum()) - 1e-9;
  for (int i = 0; i < P_->GetNum(); i++) {
    double offset = 1 + ((2.0 * i + 1)/ (2.0 * P_->GetNum()));
    tie_breakers_.push_back(
        offset + GetRandomFloat(-step, step, MT_));
  }
  std::shuffle(tie_breakers_.begin(), tie_breakers_.end(), *MT_);
  bool has_no_reached_goal = false;
  for (int i = 0; i < P_->GetNum(); ++i) {
    Node* s = P_->GetStart(i);
    Node* g = P_->GetGoal(i);
    int d = PathDist(s, g);
    Agent* a = new Agent{i,                 // id
                         s,                 // current location
                         nullptr,           // next location
                         g,                 // goal
                         0,                 // elapsed
                         d,                 // dist from s -> g
                         tie_breakers_[i],  // tie-breaker
                         g,                 // current_target
                         s == g};           // is_reached_goal
    A_.push_back(a);
    occupied_now_[s->id] = a;
    has_no_reached_goal |= s != g;
  }
  solution_.Add(P_->GetConfigStart());
  InitGuidance(A_);
  potentials_.clear();
  objective_gains_.clear();
  weighted_objective_gains_.clear();
  potentials_.emplace_back(GetCurrentPotential(A_));
  if (has_no_reached_goal) {
    // main loop
    while (true) {
      Info(" ", "elapsed:", GetSolverElapsedTime(), ", timestep:", time_step_);

      Time::time_point start = Time::now();
      occupied_next_.clear();
      occupied_next_.resize(G_->getNodesSize(), nullptr);
      for (auto& a : A_) {
        if (a->is_reached_goal) a->current_target = a->v_now;
      }
      Refine(A_);

      auto compare_agent = [&](const Agent* a, const Agent* b) {
        if (a->is_reached_goal != b->is_reached_goal)
          return a->is_reached_goal < b->is_reached_goal;
        if (a->elapsed != b->elapsed) return a->elapsed > b->elapsed;
        // use initial distance
        return a->tie_breaker > b->tie_breaker;
      };
      // std::cout << "\n \n" << timestep << std::endl;
      bool res = ComputeOneStep();
      running_times_.emplace_back(GetElapsedTime(start));
      if (OverCompTime())
        break;
      if (!res) {
        for (uint i = 0; i < A_.size(); ++i) {
          A_[i]->v_next = A_[i]->v_now;
          occupied_next_[A_[i]->v_next->id] = A_[i];
        }
      }

      objective_gains_.emplace_back(
          GetObjectiveGain(A_, highest_priority_agent_));
      weighted_objective_gains_.emplace_back(
          GetWeightedObjectiveGain(A_, highest_priority_agent_));
      // acting
      bool check_goal_cond = true;
      Config config(P_->GetNum(), nullptr);
      // std::cout << std::endl;
      bool have_move = false;
      int remain_agent_num = 0;
      for (auto &a : A_) {
        // clear
        if (occupied_now_[a->v_now->id] == a)
          occupied_now_[a->v_now->id] = nullptr;
        occupied_next_[a->v_next->id] = nullptr;
        // set next location
        config[a->id] = a->v_next;
        occupied_now_[a->v_next->id] = a;
        a->is_reached_goal |= a->v_next == a->g;
        // if (!reached_goal[a->id]) {
        //   std::cout << a->id << " " << reached_goal[a->id] << " ("
        //             << a->v_now->pos.x << ", " << a->v_now->pos.y << ") -> ("
        //             << a->v_next->pos.x << ", " << a->v_next->pos.y << ") :: ("
        //             << a->g->pos.x << ", " << a->g->pos.y << ")" << std::endl;
        // }
        // check goal condition
        check_goal_cond &= a->is_reached_goal;
        if (!a->is_reached_goal) remain_agent_num += 1;
        // check_goal_cond &= (a->v_next == a->g);
        // update priority
        a->elapsed = (a->is_reached_goal) ? 0 : a->elapsed + 1;
        if (a->v_now!= a->v_next) {
          have_move = true;
        }
        // reset params
        a->v_now = a->v_next;
        a->v_next = nullptr;
      }

      potentials_.emplace_back(GetCurrentPotential(A_));
      // update plan
      solution_.Add(config);

      time_step_++;

      // success
      if (check_goal_cond) {
        solved_ = true;
        break;
      }
      // std::cout << time_step_ << " " << remain_agent_num << std::endl;
      // assert(have_move);

      // failed
      if (time_step_ >= max_timestep_ || OverCompTime()) {
        break;
      }
    }
  } else {
    solved_ = true;
  }

  // memory clear
  for (auto& a : A_) delete a;
}

void PIBT_Base::InitGuidance(const std::vector<Agent*>& agents) {
  if (!use_guidance_) return;
  guidance_paths_.clear();
  node_occupy_.clear();
  edge_occupy_.clear();
  bfs_que_.clear();
  bfs_head_.clear();
  heuristic_table_.clear();

  auto G = P_->GetG();
  bfs_que_.resize(agents.size());
  bfs_head_.resize(agents.size());
  heuristic_table_.resize(
      agents.size(), std::vector<HeuristicTableElement>(
                         P_->GetG()->getNodesSize(),
                         HeuristicTableElement(infinity_, infinity_, false)));
  node_occupy_.resize(G->getNodesSize(), 0);
  edge_occupy_.resize(G->getNodesSize());

  for (auto& node : G->getV()) {
    edge_occupy_[node->id].resize(node->neighbor.size(), 0);
  }

  guidance_paths_.resize(agents.size());
  for (auto a : agents) {
    guidance_paths_[a->id].push_back({a->g, 0});
    node_occupy_[a->g->id]++;
  }
  max_guidance_length_ = 1;
  Replan(agents);
}

void PIBT_Base::Refine(
    const std::vector<Agent*>& agents) {
  if (!use_guidance_) return;
  std::vector<Agent*> selected_agents;
  for (int i = 0; i < 3; i++) {
    RandomSelect(10, &selected_agents);
    Replan(selected_agents);
  }
  max_guidance_length_ = 1;
  for (auto &a : A_) {
    max_guidance_length_ = std::max(
        max_guidance_length_, static_cast<int>(guidance_paths_[a->id].size()));
  }
}

void PIBT_Base::Replan(const std::vector<Agent*>& selected_agents) {
  if (!use_guidance_) return;
  for (auto& a : selected_agents) {
    auto &guidance_path = guidance_paths_[a->id];
    for (int i = 0; i + 1 < static_cast<int>(guidance_path.size());
         i++) {
      edge_occupy_[guidance_path[i].first->id][guidance_path[i].second]--;
    }
    for (int i = 0; i < static_cast<int>(guidance_path.size()); i++) {
      node_occupy_[guidance_path[i].first->id]--;
    }
    SearchGuidancePath(a, node_occupy_, edge_occupy_, &guidance_path);
    InitHeuristicTableSearch(a, guidance_path);
    // if (a->id == 0) {
    //   puts("new_guidance_path");
    //   for (int i = 0; i < static_cast<int>(guidance_path.size())-1; i++) {
    //     int id = guidance_path[i].second;
    //     int neighbor_id = guidance_path[i].second;
    //     auto current_node = guidance_path[i].first;
    //     auto next_node = current_node->neighbor[neighbor_id];
    //     std::cout << i << " "
    //               << ": (" << current_node->pos.x << ", " << current_node->pos.y
    //               << ") -> (" << next_node->pos.x << ", " << next_node->pos.y
    //               << ")" << std::endl;
    //   }
    //   puts("");
    // }
    for (int i = 0; i + 1 < static_cast<int>(guidance_path.size());
         i++) {
      edge_occupy_[guidance_path[i].first->id][guidance_path[i].second]++;
    }
    for (int i = 0; i < static_cast<int>(guidance_path.size()); i++) {
      node_occupy_[guidance_path[i].first->id]++;
      // if (node_occupy[guidance_path[i].first->id]!=1) {
      //   puts("not ok");
      //   getchar();
      // }
    }
  }
}

void PIBT_Base::SearchGuidancePath(
    Agent* agent,
    const std::vector<int> &node_occupy,
    const std::vector<std::vector<int>> &edge_occupy,
    std::vector<std::pair<Node*, int>>* guidance_path) {
  // Priority queue for focal search
  std::priority_queue<NodeState*, std::vector<NodeState*>, CompareF> open;
  std::priority_queue<NodeState*, std::vector<NodeState*>, CompareJam> focal;
  std::unordered_map<Node*, NodeState*> node_states;

  // Initialize start node
  Node* start = agent->v_now;
  Node* goal = agent->current_target;

  int h_start = PathDist(start, goal);
  NodeState* root = new NodeState(start, 0, h_start, std::make_pair(0, 0), 0);
  open.push(root);
  node_states[start] = root;

  int f_min = h_start;
  const float FOCAL_W = 1.0;  // Focal search bound multiplier
  int f_bound = f_min * FOCAL_W;

  NodeState* goal_node = nullptr;

  while (!open.empty() || !focal.empty()) {
    // Update focal list
    while (!open.empty() && open.top()->f <= f_bound) {
      // if (agent->id == 0) {
      //   std::cout << open.top()->g << " " << open.top()->h << " " <<
      //   open.top()->f << std::endl;
      // }
      focal.push(open.top());
      open.pop();
    }

    if (focal.empty()) {
      // Update f_min and f_bound
      if (!open.empty()) {
        f_min = open.top()->f;
        f_bound = f_min * FOCAL_W;
        continue;
      }
      break;
    }

    NodeState* curr = focal.top();
    focal.pop();
    curr->closed = true;
    if (curr->node == goal) {
      goal_node = curr;
      break;
    }

    // Expand neighbors
    int id = -1;
    for (Node* next : curr->node->neighbor) {
      id++;
      int new_g = curr->g + 1;
      int new_h = PathDist(next, agent->current_target);
      int id2 = -1;
      for (Node* next2 : next->neighbor) {
        id2++;
        if (next2 == curr->node) {
          break;
        }
      }
      std::pair<int, int> edge_cost;

      edge_cost.first =
          edge_occupy[curr->node->id][id] * edge_occupy[next->id][id2];
      edge_cost.second = 1 + node_occupy[next->id] / 2;

      std::pair<int, int> new_cost =
          std::make_pair(curr->cost.first + edge_cost.first,
                         curr->cost.second + edge_cost.second);
      // std::cout << curr->cost.first << " " << curr->cost.second << " "
      //           << new_cost.first << " " << new_cost.second << std::endl;
      double new_tie_breaker =
          curr->tie_breaker +
          (0.5 * (double)node_occupy[next->id] / (double)P_->GetNum() +
           0.5 * (double)edge_occupy[next->id][id2] / (double)P_->GetNum());
      // Create or update node
      auto it = node_states.find(next);
      if (it == node_states.end()) {
        NodeState* next_state =
            new NodeState(next, new_g, new_h, new_cost, new_tie_breaker);
        next_state->parent = curr;
        next_state->parent_neighbor_id = id;
        node_states[next] = next_state;
        if (next_state->f <= f_bound)
          focal.push(next_state);
        else
          open.push(next_state);
      } else {
        NodeState* existing = it->second;
        if (CanReplace(new_g, new_cost, new_tie_breaker, existing)) {
          existing->g = new_g;
          existing->h = new_h;
          existing->f = new_g + new_h;
          existing->parent = curr;
          existing->cost = new_cost;
          existing->tie_breaker = new_tie_breaker;
          existing->parent_neighbor_id = id;
        }
      }
    }
  }

  // Reconstruct path if goal was found
  if (goal_node != nullptr) {
    guidance_path->clear();
    // std::cout << "start:" << start->id << " " << goal->id << std::endl;
    // std::cout << " goal_cost:" << goal_node->cost.first << " "
    //           << goal_node->cost.second << " " << " "
    //           << goal_node->cost.first - goal_node->g << " "
    //           << goal_node->cost.second - goal_node->g << std::endl;
    NodeState* curr = goal_node;
    std::vector<std::pair<Node*, int>> reverse_path;
    int neigbor_id = -1;
    while (curr != nullptr) {
      reverse_path.push_back({curr->node, neigbor_id});
      neigbor_id = curr->parent_neighbor_id;
      curr = curr->parent;
    }

    // Reverse the path to get start->goal order
    for (auto it = reverse_path.rbegin(); it != reverse_path.rend(); ++it) {
      guidance_path->push_back(*it);
    }
  }

  // Cleanup
  for (auto& pair : node_states) {
    delete pair.second;
  }
}

void PIBT_Base::RandomSelect(int num, std::vector<Agent*>* selected_agents) {
  if (num >= A_.size()) {
    *selected_agents = A_;
    return;
  }
  int i = 0;
  std::set<int> selected;
  selected_agents->clear();
  while (i < num) {
    int index = rand() % A_.size();
    // check if index is already in agents

    if (selected.find(index) != selected.end()) {
      continue;
    }
    selected.insert(index);
    // std::cout << index << std::endl;
    selected_agents->emplace_back(A_[index]);
    i += 1;
  }
}

int PIBT_Base::GetPIBTHeuristic(Agent* ai, Node* vj) {
  if (!use_guidance_) {
    return PathDist(vj, ai->current_target);
  } else {
    UpdateHeuristicTable(ai, vj);
    auto h = heuristic_table_[ai->id][vj->id];
    return max_guidance_length_ * h.dis2guidance + h.guidance2goal;
  }
}


void PIBT_Base::InitHeuristicTableSearch(
    Agent* ai, const std::vector<std::pair<Node*, int>>& guidance_path) {
  auto &table = heuristic_table_[ai->id];
  auto &que = bfs_que_[ai->id];
  auto &head = bfs_head_[ai->id];
  for (auto n : que) {
    table[n->id] = HeuristicTableElement(infinity_, infinity_, false);
  }
  que.clear();
  head = 0;
  for (int i = 0; i< guidance_path.size(); i++) {
    auto v = guidance_path[i].first;
    table[v->id] = HeuristicTableElement(
        0, static_cast<int>(guidance_path.size()) - i - 1, false);
    que.push_back(v);
  }
}

void PIBT_Base::UpdateHeuristicTable(Agent* ai, Node* vj) {
  auto &table = heuristic_table_[ai->id];
  if (table[vj->id].is_closed == true) return;
  auto &que = bfs_que_[ai->id];
  auto &head = bfs_head_[ai->id];
  while (head < que.size() && table[vj->id].is_closed == false) {
    Node* current = que[head];
    for (auto &neighbor : current->neighbor) {
      if (table[neighbor->id].dis2guidance == infinity_) {
        que.push_back(neighbor);
      }
      if ((table[neighbor->id].dis2guidance >
           table[current->id].dis2guidance + 1) ||
          ((table[neighbor->id].dis2guidance ==
            table[current->id].dis2guidance + 1) &&
           (table[neighbor->id].guidance2goal >
            table[current->id].guidance2goal))) {
        table[neighbor->id].dis2guidance = table[current->id].dis2guidance + 1;
        table[neighbor->id].guidance2goal = table[current->id].guidance2goal;
      }
    }
    head++;
    table[current->id].is_closed = true;
  }
}
};  // namespace gpibt
