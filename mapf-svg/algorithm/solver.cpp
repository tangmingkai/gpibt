#include "solver.hpp"

#include <omp.h>

#include <fstream>
#include <iomanip>
#include <algorithm>

namespace gpibt {
MinimumSolver::MinimumSolver(Problem* _P, bool _use_guidance,
                             bool _enable_swap_action,
                             bool _set_initial_solution)
    : solver_name_(""),
      G_(_P->GetG()),
      MT_(_P->GetMT()),
      max_timestep_(_P->GetMaxTimestep()),
      max_comp_time_(_P->GetMaxCompTime()),
      solved_(false),
      comp_time_(0),
      verbose_(false),
      log_short_(false),
      use_guidance_(_use_guidance),
      enable_swap_action_(_enable_swap_action),
      set_initial_solution_(_set_initial_solution) {}

void MinimumSolver::Solve() {
  Start();
  Exec();
  End();
}

void MinimumSolver::Start() { t_start_ = Time::now(); }

void MinimumSolver::End() { comp_time_ = GetSolverElapsedTime(); }

// -------------------------------
// utilities for time
// -------------------------------

int MinimumSolver::GetSolverElapsedTime() const {
  return GetElapsedTime(t_start_);
}

int MinimumSolver::GetRemainedTime() const {
  return std::max(0, max_comp_time_ - GetSolverElapsedTime());
}

bool MinimumSolver::OverCompTime() const {
  return GetSolverElapsedTime() >= max_comp_time_;
}

// -------------------------------
// utilities for debug
// -------------------------------
void MinimumSolver::Info() const {
  if (verbose_) std::cout << std::endl;
}

void MinimumSolver::Halt(const std::string& msg) const {
  std::cout << "error@" << solver_name_ << ": " << msg << std::endl;
  this->~MinimumSolver();
  std::exit(1);
}

void MinimumSolver::Warn(const std::string& msg) const {
  std::cout << "warn@ " << solver_name_ << ": " << msg << std::endl;
}

// -----------------------------------------------
// base class with utilities
// -----------------------------------------------

MAPF_Solver::MAPF_Solver(MAPF_Instance* _P, bool _use_guidance,
                         bool _enable_swap_action, bool _set_initial_solution)
    : MinimumSolver(_P, _use_guidance, _enable_swap_action,
                    _set_initial_solution),
      P_(_P),
      LB_soc_(0),
      LB_makespan_(0),
      is_preprocess_(false) {}

MAPF_Solver::~MAPF_Solver() {}

// -------------------------------
// main
// -------------------------------
void MAPF_Solver::PreProcess() {
  if (is_preprocess_ == true)
    return;
  CreateDistanceTable();
  preprocessing_comp_time_ = GetSolverElapsedTime();
  Info("preprocessing done, elapsed: ", preprocessing_comp_time_);
  is_preprocess_ = true;
}

void MAPF_Solver::SharePreprocessResult(
    std::shared_ptr<std::vector<std::vector<int>>> distance_table) {
  distance_table_ = distance_table;
  is_preprocess_ = true;
}

std::shared_ptr<std::vector<std::vector<int>>>
MAPF_Solver::GetPreprocessResult() {
  return distance_table_;
}

void MAPF_Solver::Exec() {
  // create distance table
  PreProcess();
  Run();
  solution_.GetReachGoalTime(P_->GetConfigGoal(), &reach_goal_time_);
}

// -------------------------------
// utilities for problem instance
// -------------------------------
void MAPF_Solver::ComputeLowerBounds() {
  LB_soc_ = 0;
  LB_makespan_ = 0;

  for (int i = 0; i < P_->GetNum(); ++i) {
    int d = PathDist(P_->GetStart(i), P_->GetGoal(i));
    LB_soc_ += d;
    if (d > LB_makespan_) LB_makespan_ = d;
  }
}

int MAPF_Solver::GetLowerBoundSOC() {
  if (LB_soc_ == 0) ComputeLowerBounds();
  return LB_soc_;
}

int MAPF_Solver::GetLowerBoundMakespan() {
  if (LB_makespan_ == 0) ComputeLowerBounds();
  return LB_makespan_;
}

// -------------------------------
// utilities for solution representation
Paths MAPF_Solver::PlanToPaths(const Plan& plan) {
  int num_agents = plan.Get(0).size();
  Paths paths(num_agents);
  int makespan = plan.GetMakespan();
  for (int i = 0; i < num_agents; ++i) {
    Path path;
    for (int t = 0; t <= makespan; ++t) {
      path.push_back(plan.Get(t, i));
    }
    paths.Insert(i, path);
  }
  return paths;
}

Plan MAPF_Solver::PathsToPlan(const Paths& paths) {
  Plan plan;
  if (paths.Empty()) return plan;
  int makespan = paths.GetMakespan();
  int num_agents = paths.Size();
  for (int t = 0; t <= makespan; ++t) {
    Config c;
    for (int i = 0; i < num_agents; ++i) {
      c.push_back(paths.Get(i, t));
    }
    plan.Add(c);
  }
  return plan;
}


// -------------------------------
// print
// -------------------------------
void MAPF_Solver::PrintResult() {
  std::cout << "solved=" << solved_ << ", solver=" << std::right << std::setw(8)
            << solver_name_ << ", comp_time(ms)=" << std::right << std::setw(8)
            << GetCompTime() << ", soc=" << std::right << std::setw(6)
            << solution_.GetSOC(reach_goal_time_) << " (LB=" << std::right
            << std::setw(6) << GetLowerBoundSOC() << ")"
            << ", makespan=" << std::right << std::setw(4)
            << solution_.GetMakespan() << " (LB=" << std::right << std::setw(6)
            << GetLowerBoundMakespan() << ")" << std::endl;
}


// -------------------------------
// log
// -------------------------------
void MAPF_Solver::MakeLog(const std::string& logfile) {
  std::ofstream log;
  log.open(logfile, std::ios::out);
  MakeLogBasicInfo(log);
  MakeLogSolution(log);
  log.close();
}

void MAPF_Solver::MakeLogBasicInfo(std::ofstream& log) {
  Grid* grid = reinterpret_cast<Grid*>(P_->GetG());
  log << "problem_name: MAPF-SVG" << "\n";
  log << "map_file: " << grid->getMapFileName() << "\n";
  log << "solver: " << solver_name_ << "\n";
  log << "solved: " << solved_ << "\n";
  log << "soc: " << solution_.GetSOC(reach_goal_time_) << "\n";
  log << "lb_soc: " << GetLowerBoundSOC() << "\n";
  log << "makespan: " << solution_.GetMakespan() << "\n";
  log << "lb_makespan: " << GetLowerBoundMakespan() << "\n";
  log << "comp_time: " << GetCompTime() << "\n";
  log << "preprocessing_comp_time: " << preprocessing_comp_time_ << "\n";
  log << "seed: " << P_->GetSeed() << "\n";
}

void MAPF_Solver::MakeLogSolution(std::ofstream& log) {
  if (log_short_) return;

  // Output starts
  log << "starts: ";
  for (int i = 0; i < P_->GetNum(); ++i) {
    Node* v = P_->GetStart(i);
    log << "(" << v->pos.x << "," << v->pos.y << "),";
  }
  log << "\n";
  log << "\n";
  log << "\n";
  if (!objective_gains_.empty()) {
    log << "objective_gain: ";
    for (auto& gain : objective_gains_) {
      log << gain << ";";
    }
    log << "\n";
  }
  if (!weighted_objective_gains_.empty()) {
    log << "weighted_objective_gain: ";
    for (auto& gain : weighted_objective_gains_) {
      log << gain << ";";
    }
    log << "\n";
  }
  if (!potentials_.empty()) {
    log << "potentials=";
    for (auto& potential : potentials_) {
      log << potential << ";";
    }
    log << "\n";
  }

  if (!running_times_.empty()) {
    log << "running_times: ";
    for (auto& running_time : running_times_) {
      log << running_time << ";";
    }
    log << "\n";
  }

  if (!tie_breakers_.empty()) {
    log << "tie_breaker: ";
    for (auto& tie_breaker : tie_breakers_) {
      log << std::fixed << std::setprecision(9) << tie_breaker << ";";
    }
    log << "\n";
  }
  // Output tasks
  log << "task:\n";
  for (int i = 0; i < P_->GetNum(); ++i) {
    auto reach_goal_time = reach_goal_time_[i] < 0 ? -1 : reach_goal_time_[i];
    log << i << ":[" << P_->GetStart(i)->pos.x << "," << P_->GetStart(i)->pos.y
        << "]->"
        << "[" << P_->GetGoal(i)->pos.x << "," << P_->GetGoal(i)->pos.y << "],"
        << "appear=" << 0 << ","
        << "assigned=" << 0 << ","
        << "finished=" << reach_goal_time << "\n";
  }

  // Output solution - one agent per line
  log << "solution:\n";
  for (int i = 0; i < P_->GetNum(); ++i) {
    log << "  " << i << ": ";
    int makespan = solution_.GetMakespan();
    for (int t = 0; t < makespan; ++t) {
      auto v = solution_.Get(t, i);
      log << "(" << v->pos.x << "," << v->pos.y << ")->";
    }
    log << "\n";
  }
}

// void MAPF_Solver::makeLogSolution(std::ofstream& log) {
//   if (log_short) return;
//   log << "starts=";
//   for (int i = 0; i < P->getNum(); ++i) {
//     Node* v = P->getStart(i);
//     log << "(" << v->pos.x << "," << v->pos.y << "),";
//   }
//   log << "\ngoals=";
//   for (int i = 0; i < P->getNum(); ++i) {
//     Node* v = P->getGoal(i);
//     log << "(" << v->pos.x << "," << v->pos.y << "),";
//   }
//   log << "\n";
//   log << "solution=\n";
//   for (int t = 0; t <= solution.getMakespan(); ++t) {
//     log << t << ":";
//     auto c = solution.get(t);
//     for (auto v : c) {
//       log << "(" << v->pos.x << "," << v->pos.y << "),";
//     }
//     log << "\n";
//   }
// }

// -------------------------------
// distance
// -------------------------------

int MAPF_Solver::PathDist(Node* const s, Node* const g) const {
  return (*distance_table_)[s->id][g->id];
}

// int MAPF_Solver::pathDist(const int i, Node* const s) const {
//   return 
// }

// int MAPF_Solver::pathDist(const int i) const {
//   return pathDist(i, P->getStart(i));
// }

void MAPF_Solver::CreateDistanceTable() {
  distance_table_ = std::make_shared<std::vector<std::vector<int>>>(
      G_->getNodesSize(), std::vector<int>(G_->getNodesSize(), max_timestep_));
  const int nodes_num = G_->getNodesSize();
  const auto& nodes = G_->getV();
  #pragma omp parallel for
  for (int i = 0; i < nodes_num; ++i) {
    std::fill((*distance_table_)[i].begin(), (*distance_table_)[i].end(),
              G_->getNodesSize());
    (*distance_table_)[i][i] = 0;
  }
  #pragma omp parallel for schedule(dynamic)
  for (size_t start_idx = 0; start_idx < nodes.size(); ++start_idx) {
    Node* start_node = nodes[start_idx];
    if (!start_node) continue;

    std::vector<Node*> que;
    std::vector<bool> visited(nodes_num, false);
    que.reserve(nodes_num);

    que.push_back(start_node);
    visited[start_node->id] = true;
    (*distance_table_)[start_node->id][start_node->id] = 0;

    for (size_t head = 0; head < que.size(); ++head) {
      Node* current = que[head];
      int current_dist = (*distance_table_)[start_node->id][current->id];
      for (auto neighbor : current->neighbor) {
        if (!visited[neighbor->id]) {
          visited[neighbor->id] = true;
          que.push_back(neighbor);
          (*distance_table_)[start_node->id][neighbor->id] = current_dist + 1;
        }
      }
    }
  }
}

// void MAPF_Solver::createDistanceTable() {
//   puts("create 111");
//   const int nodes_num = G->getNodesSize();
//   const auto& nodes = G->getV();
// puts("create 222");
//   std::cout << distance_table.size() << " " << nodes_num << std::endl;
//   #pragma omp parallel for
//   for (int i = 0; i < nodes_num; ++i) {
//     std::fill(distance_table[i].begin(), distance_table[i].end(),
//               G->getNodesSize());
//     distance_table[i][i] = 0;
//   }
//   puts("create 333");
//   #pragma omp parallel for schedule(dynamic)
//   for (size_t start_idx = 0; start_idx < nodes.size(); ++start_idx) {
//     Node* start_node = nodes[start_idx];
//     if (!start_node) continue;

//     std::vector<Node*> que;
//     std::vector<bool> visited(nodes_num, false);
//     que.reserve(nodes_num);

//     que.push_back(start_node);
//     visited[start_node->id] = true;
//     distance_table[start_node->id][start_node->id] = 0;

//     for (size_t head = 0; head < que.size(); ++head) {
//       Node* current = que[head];
//       int current_dist = distance_table[start_node->id][current->id];
//       for (auto neighbor : current->neighbor) {
//         if (!visited[neighbor->id]) {
//           visited[neighbor->id] = true;
//           que.push_back(neighbor);
//           distance_table[start_node->id][neighbor->id] = current_dist + 1;
//         }
//       }
//     }
//   }
// }

}  // namespace gpibt



