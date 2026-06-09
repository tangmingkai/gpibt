#include "plan.hpp"
#include <algorithm>
namespace gpibt {
Config Plan::Get(const int t) const {
  const int configs_size = configs_.size();
  if (!(0 <= t && t < configs_size)) Halt("invalid timestep");
  return configs_[t];
}

Node* Plan::Get(const int t, const int i) const {
  if (Empty()) Halt("invalid operation");
  if (!(0 <= t && t < (int)configs_.size())) Halt("invalid timestep");
  if (!(0 <= i && i < (int)configs_[0].size())) 
  {
    std::cout << i << std::endl;
    Halt("invalid agent id");
  }
  return configs_[t][i];
}

Path Plan::GetPath(const int i) const {
  Path path;
  int makespan = GetMakespan();
  for (int t = 0; t <= makespan; ++t) path.push_back(Get(t, i));
  return path;
}

Config Plan::Last() const {
  if (Empty()) Halt("invalid operation");
  return configs_[GetMakespan()];
}

Node* Plan::Last(const int i) const {
  if (Empty()) Halt("invalid operation");
  if (i < 0 || (int)configs_[0].size() <= i) Halt("invalid operation");
  return configs_[GetMakespan()][i];
}

void Plan::Clear() { configs_.clear(); }

void Plan::Add(const Config& c) {
  if (!configs_.empty() && configs_.at(0).size() != c.size()) {
    Halt("invalid operation");
  }
  configs_.push_back(c);
}

bool Plan::Empty() const { return configs_.empty(); }

int Plan::Size() const { return configs_.size(); }

int Plan::GetMakespan() const { return Size() - 1; }

// int Plan::getPathCost(const int i) const {
//   const int makespan = getMakespan();
//   const Node* g = get(makespan, i);
//   int c = makespan;
//   while (c > 0 && get(c - 1, i) == g) --c;
//   return c;
// }

int Plan::GetSOC(const std::vector<int>& reach_goal_time) const {
  // int makespan = getMakespan();
  // if (makespan <= 0) return 0;
  // int num_agents = get(0).size();
  int soc = 0;
  for (int i = 0; i < reach_goal_time.size(); ++i) soc += reach_goal_time[i];
  return soc;
}

Plan Plan::operator+(const Plan& other) const {
  // check validity
  Config c1 = Last();
  Config c2 = other.Get(0);
  const int c1_size = c1.size();
  const int c2_size = c2.size();
  if (c1_size != c2_size) Halt("invalid operation");
  for (int i = 0; i < c1_size; ++i) {
    if (c1[i] != c2[i]) Halt("invalid operation.");
  }
  // merge
  Plan new_plan;
  new_plan.configs_ = configs_;
  for (int t = 1; t < other.Size(); ++t) new_plan.Add(other.Get(t));
  return new_plan;
}

void Plan::operator+=(const Plan& other) {
  if (configs_.empty()) {
    configs_ = other.configs_;
    return;
  }
  // check validity
  if (!SameConfig(Last(), other.Get(0))) Halt("invalid operation");
  // merge
  for (int t = 1; t < other.Size(); ++t) Add(other.Get(t));
}

bool Plan::Validate(MAPF_Instance* P) const {
  return Validate(P->GetConfigStart(), P->GetConfigGoal());
}

void Plan::GetReachGoalTime(const Config& goals,
                            std::vector<int>* reach_goal_time) const {
  reach_goal_time->clear();
  int max_time = configs_.size();
  int num_agents = Get(0).size();
  reach_goal_time->resize(Get(0).size(), max_time);
  
  for (int i = 0; i < num_agents; ++i) {
    for (int t = 0; t < max_time; ++t) {
      if (configs_[t][i] == goals[i]) {
        reach_goal_time->at(i) = t;
        break;
      }
    }
  }
}

bool Plan::Validate(const Config& starts, const Config& goals) const {
  if (configs_.empty()) return false;
  if (!SameConfig(starts, Get(0))) {
    Warn("validation, invalid starts");
    return false;
  }
  // check goal

  std::vector<bool> reached_goal(starts.size(), false);
  for (int t = 0; t < configs_.size(); ++t) {
    for (int i = 0; i < configs_[t].size(); ++i) {
      if (configs_[t][i] == goals[i]) reached_goal[i] = true;
    }
  }
  for (int i = 0; i < reached_goal.size(); i++)
    if (!reached_goal[i]) {
      Warn("validation, invalid goal");
      return false;
    }
  return ValidateConflict();
}

bool Plan::ValidateConflict() const {
  // check conflicts and continuity
  int num_agents = Get(0).size();
  for (int t = 1; t <= GetMakespan(); ++t) {
    if ((int)configs_[t].size() != num_agents) {
      Warn("validation, invalid size");
      return false;
    }
    
    for (int i = 0; i < num_agents; ++i) {
      Node* v_i_t = Get(t, i);
      Node* v_i_t_1 = Get(t - 1, i);
      Nodes cands = v_i_t_1->neighbor;
      cands.push_back(v_i_t_1);
      if (!InArray(v_i_t, cands)) {
        Warn("validation, invalid move at t=" + std::to_string(t));
        return false;
      }
    }
  }
  int max_v_id = -1;
  for (int t = 0; t <= GetMakespan(); ++t) {
    for (int i = 0; i< num_agents; i++) {
      max_v_id = std::max(max_v_id, configs_[t][i]->id);
    }
  }
  std::vector<int> occ_next(max_v_id + 1, -1);
  for (int t = 0; t < GetMakespan() - 1; ++t) {
    for (int i = 0; i < num_agents; ++i) {
      Node* v_i_t = Get(t, i);
      Node* v_i_t_1 = Get(t + 1, i);
      if (occ_next[v_i_t_1->id] != -1) {
        Warn("validation, vertex conflict at v=" + std::to_string(v_i_t->id) +
               ", t=" + std::to_string(t));
          std::cout << "t=" << t << "--agent_" << i << ":(" << v_i_t->pos.x
                    << "," << v_i_t->pos.y << ") vertex conflict with agent_"
                    << occ_next[v_i_t_1->id] << std::endl;
          return false;
      }
      if (occ_next[v_i_t->id] != -1) {
        Node* v_j_1 = Get(t, occ_next[v_i_t->id]);
        if (v_j_1 == v_i_t_1) {
          Warn("validation, edge conflict at v=" + std::to_string(v_i_t->id) +
                ", t=" + std::to_string(t));
            std::cout << "t=" << t << "--agent_" << i << ":(" << v_i_t->pos.x
                      << "," << v_i_t->pos.y << ") edge conflict with agent_"
                      << occ_next[v_i_t_1->id] << std::endl;
            return false;
        }
      }
      occ_next[v_i_t_1->id] = i;
    }
    for (int i = 0; i < num_agents; ++i) {
      occ_next[Get(t + 1, i)->id] = -1;
    }
  }
  return true;
}

int Plan::GetMaxConstraintTime(const int id, Node* s, Node* g, Graph* G) const {
  const int makespan = GetMakespan();
  const int dist = G->pathDist(s, g);
  const int num = configs_[0].size();
  for (int t = makespan - 1; t >= dist; --t) {
    for (int i = 0; i < num; ++i) {
      if (i != id && Get(t, i) == g) return t;
    }
  }
  return 0;
}

int Plan::GetMaxConstraintTime(const int id, MAPF_Instance* P) const {
  return GetMaxConstraintTime(id, P->GetStart(id), P->GetGoal(id), P->GetG());
}

void Plan::Halt(const std::string& msg) const {
  std::cout << "error@Plan: " << msg << std::endl;
  this->~Plan();
  std::exit(1);
}

void Plan::Warn(const std::string& msg) const {
  std::cout << "warn@Plan: " << msg << std::endl;
}
};  // namespace gpibt
