#pragma once
#include <getopt.h>

#include <chrono>
#include <functional>
#include <string>
#include <memory>
#include <queue>
#include <utility>
#include <unordered_map>
#include <vector>

#include "paths.hpp"
#include "plan.hpp"
#include "problem.hpp"
#include "util.hpp"

namespace gpibt {

class MinimumSolver {
 protected:
  std::string solver_name_;  // solver name
  Graph* const G_;           // graph
  std::mt19937* const MT_;   // seed for randomness
  const int max_timestep_;   // maximum makespan
  const int max_comp_time_;  // time limit for computation, ms
  Plan solution_;            // solution
  std::vector<int> reach_goal_time_;
  bool solved_;              // success -> true, failed -> false (default)

 private:
  int comp_time_;             // computation time
  Time::time_point t_start_;  // when to start solving

 protected:
  bool verbose_;    // true -> print additional info
  bool log_short_;  // true -> cannot visualize the result, default: false
  // parameters
  bool use_guidance_;          // use guidance or not
  bool enable_swap_action_;    // enable swap action or not
  bool set_initial_solution_;  // set initial solution or not

  // -------------------------------
  // utilities for time
 public:
  int GetRemainedTime() const;  // get remained time
  bool OverCompTime() const;    // check time limit
  void Start();
  void End();
  // -------------------------------
  // utilities for debug
 protected:
  // print debug info (only when verbose=true)
  void Info() const;
  template <class Head, class... Tail>
  void Info(Head&& head, Tail&&... tail) const {
    if (!verbose_) return;
    std::cout << head << " ";
    Info(std::forward<Tail>(tail)...);
  }
  void Halt(const std::string& msg) const;  // halt program
  void Warn(const std::string& msg) const;  // just printing msg

  // -------------------------------
  // utilities for solver options
 public:
  virtual void SetParams(int argc, char* argv[]) {}
  void SetVerbose(bool _verbose) { verbose_ = _verbose; }
  void SetLogShort(bool _log_short) { log_short_ = _log_short; }

 public:
  virtual void Solve();  // call start -> run -> end

 protected:
  virtual void Exec() {}  // main

 public:
  explicit MinimumSolver(Problem* _P, bool _use_guidance,
                         bool _enable_swap_action, bool _set_initial_solution);
  virtual ~MinimumSolver() {}

  // getter
  Plan GetSolution() const { return solution_; }
  bool Succeed() const { return solved_; }
  std::string GetSolverName() const { return solver_name_; }
  int GetMaxTimestep() const { return max_timestep_; }
  int GetCompTime() const { return comp_time_; }
  int GetSolverElapsedTime() const;  // get elapsed time from start
};

// -----------------------------------------------
// base class with utilities
// -----------------------------------------------
class MAPF_Solver : public MinimumSolver {
 protected:
  MAPF_Instance* const P_;  // problem instance

 private:
  // useful info
  int LB_soc_;       // lower bound of soc
  int LB_makespan_;  // lower bound of makespan

  // distance to goal
 protected:
  using DistanceTable = std::vector<std::vector<int>>;   // [node_id][node_id]
  std::shared_ptr<DistanceTable> distance_table_;        // distance table
  int preprocessing_comp_time_;      // computation time
  bool is_preprocess_;
  std::vector<double> tie_breakers_;  // For output
  std::vector<double> potentials_;
  std::vector<double> objective_gains_;
  std::vector<double> weighted_objective_gains_;
  std::vector<double> running_times_;
  
  // -------------------------------
  // main
 private:
  void Exec();
  

 protected:
  virtual void Run() {}  // main

  // -------------------------------
  // utilities for problem instance
 public:
  int GetLowerBoundSOC();       // get trivial lower bound of sum-of-costs
  int GetLowerBoundMakespan();  // get trivial lower bound of makespan
  static Paths PlanToPaths(const Plan& plan);   // plan -> paths
  static Plan PathsToPlan(const Paths& paths);  // paths -> plan
  virtual void MakeLog(const std::string& logfile = "./result.txt");
  void PrintResult();

 private:
  void ComputeLowerBounds();  // compute lb_soc and lb_makespan


 protected:
  virtual void MakeLogBasicInfo(std::ofstream& log);
  virtual void MakeLogSolution(std::ofstream& log);
  void CreateDistanceTable();         // compute distance table


  // -------------------------------
  // utilities for distance
 public:
  int PathDist(Node* const s, Node* const g) const;
  // int PathDist(const int i,
  //              Node* const s) const;  // get path distance between s -> g_i
  // int pathDist(const int i) const;    // get path distance between s_i -> g_i
  virtual void PreProcess();
  void SharePreprocessResult(
      std::shared_ptr<std::vector<std::vector<int>>> distance_table);
  std::shared_ptr<std::vector<std::vector<int>>> GetPreprocessResult();

 protected:
  // used for checking conflicts
  void UpdatePathTable(const Paths& paths, const int id);
  void ClearPathTable(const Paths& paths);
  void UpdatePathTableWithoutClear(const int id, const Path& p,
                                   const Paths& paths);
  static constexpr int NIL = -1;

 public:
  explicit MAPF_Solver(MAPF_Instance* _P, bool _use_guidance,
                       bool _enable_swap_action, bool _set_initial_solution);
  virtual ~MAPF_Solver();

  MAPF_Instance* GetP() { return P_; }
};

}  // namespace gpibt
