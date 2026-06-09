#include <cassert>
#include <cstddef>
#include <ostream>
#include <thread>
#include <utility>

#include "gpiibt_r.hpp"
#include "ortools/linear_solver/linear_solver.h"
#include "ortools/sat/cp_model.h"
#include "ortools/sat/cp_model_solver.h"

namespace gpibt {

const std::string GPIBT_R::SOLVER_NAME = "GPIBT-R";

GPIBT_R::GPIBT_R(MAPF_Instance *_P, bool _use_guidance, bool enable_swap_action,
               bool set_initial_solution,
               const std::string &_reduce_solver_name)
    : PIBT_Base(_P, _use_guidance, enable_swap_action, set_initial_solution) {
  solver_name_ = GPIBT_R::SOLVER_NAME;
  set_initial_solution_ = set_initial_solution;
  if (set_initial_solution_) {
    gpiibt_b_ = std::make_shared<GPIBT_B>(_P, _use_guidance, enable_swap_action,
                                       set_initial_solution);
  }
}

void GPIBT_R::PreProcess() {
  if (is_preprocess_ == true)
    return;
  CreateDistanceTable();
  preprocessing_comp_time_ = GetSolverElapsedTime();
  if (set_initial_solution_) {
    gpiibt_b_->SharePreprocessResult(distance_table_);
  }
  Info("preprocessing done, elapsed: ", preprocessing_comp_time_);
  is_preprocess_ = true;
}

bool GPIBT_R::ComputeOneStep() {
  highest_priority_agent_ = HighestPriorityAgent(&A_);
  return CPSAT_v2();
}

// Use Objective
bool GPIBT_R::CPSAT_v2() {
  std::unique_ptr<operations_research::sat::CpModelBuilder> cp_model =
      std::make_unique<operations_research::sat::CpModelBuilder>();
  std::vector<std::vector<operations_research::sat::BoolVar>> x(A_.size());
  operations_research::sat::DoubleLinearExpr objective;
  for (uint i = 0; i < A_.size(); ++i) {
    if (occupied_now_[A_[i]->v_now->id] != A_[i]) {
      if (occupied_now_[A_[i]->v_now->id] != nullptr) {
        std::cout << A_[i]->id << " occupied_now: ";
        std::cout << occupied_now_[A_[i]->v_now->id]->id << std::endl;
      } else {
        std::cout << "NULL" << std::endl;
      }
    }
    assert(occupied_now_[A_[i]->v_now->id] == A_[i]);
    assert(A_[i]->v_next == nullptr);
    auto neighbors = A_[i]->v_now->neighbor;
    // int dist_to_g = pathDist(A[i]->v_now, A[i]->g);
    int current_heuristic = GetPIBTHeuristic(A_[i], A_[i]->v_now);
    int priority_weight = highest_priority_agent_ == A_[i]
                              ? A_.size() * max_guidance_length_ * 2 * 2
                              : 1;
    x[A_[i]->id].resize(neighbors.size() + 1);
    bool is_idle = A_[i]->is_reached_goal;
    for (uint j = 0; j < x[i].size(); ++j) {
      x[A_[i]->id][j] = cp_model->NewBoolVar();

      int dist_weight =
          (j == neighbors.size() || is_idle)
              ? 0
              : GetPIBTHeuristic(A_[i], neighbors[j]) - current_heuristic;
      double tie_breading_weight = A_[i]->tie_breaker;
      // int tie_breading_weight = 1;
      objective.AddExpression(priority_weight * dist_weight * x[A_[i]->id][j],
                              tie_breading_weight);

      // std::cout << "agent " << A_[i]->id << " action " << j << " weight "
      //           << priority_weight * dist_weight * tie_breading_weight
      //           << ": " << priority_weight << " " << dist_weight << " "
      //           << tie_breading_weight << std::endl;
    }
    // only one action can be performed at a time
    cp_model->AddExactlyOne(x[A_[i]->id]);
    if (A_[i]->v_next != nullptr) {
      bool is_wait = true;
      for (int j = 0; j < neighbors.size(); ++j) {
        if (neighbors[j] == A_[i]->v_next) {
          cp_model->AddEquality(x[A_[i]->id][j], 1);
          is_wait = false;
          break;
        }
      }
      if (is_wait) {
        assert(A_[i]->v_now == A_[i]->v_next);
        cp_model->AddEquality(x[A_[i]->id][neighbors.size()], 1);
      }
    }
  }

  //  for (uint i = 0; i < A_.size(); ++i) {
  //   if (A_[i]->v_next != nullptr) {
  //     ***
  //   }
  //  }
  cp_model->Minimize(objective);
  // vertex conflict
  for (uint i = 0; i < A_.size(); ++i) {
    auto &neighbors_i = A_[i]->v_now->neighbor;
    int id_i = A_[i]->id;
    auto &wait_action_var_i = x[id_i].back();
    for (uint j = 0; j < neighbors_i.size(); ++j) {
      auto *v_j = neighbors_i[j];
      auto &neighbors_j = v_j->neighbor;
      // one-hop
      auto agent_j = occupied_now_[v_j->id];
      if (agent_j != nullptr && id_i < agent_j->id) {
        int id_j = agent_j->id;
        uint j_to_i;
        for (j_to_i = 0; j_to_i < neighbors_j.size(); ++j_to_i) {
          if (neighbors_j[j_to_i] == A_[i]->v_now) {
            break;
          }
        }
        assert(j_to_i != neighbors_j.size());
        auto &wait_action_var_j = x[id_j].back();
        // wait
        cp_model->AddLessOrEqual(x[id_i][j] + wait_action_var_j, 1);
        cp_model->AddLessOrEqual(wait_action_var_i + x[id_j][j_to_i], 1);
        // edge_conflict
        cp_model->AddLessOrEqual(x[id_i][j] + x[id_j][j_to_i], 1);
        // puts("add edge");
      }

      // two-hop
      for (uint k = 0; k < neighbors_j.size(); ++k) {
        auto agent_k = occupied_now_[neighbors_j[k]->id];
        if (agent_k != nullptr && id_i < agent_k->id) {
          auto &neighbors_k = neighbors_j[k]->neighbor;
          int id_k = agent_k->id;
          uint k_to_j;
          for (k_to_j = 0; k_to_j < neighbors_k.size(); ++k_to_j) {
            if (neighbors_k[k_to_j] == v_j) {
              break;
            }
          }
                  // puts("add vertex");
          assert(k_to_j < neighbors_k.size());
          // vertex_conflict
          cp_model->AddLessOrEqual(x[id_i][j] + x[id_k][k_to_j], 1);
        }
      }
    }
  }
  // Solving part.
  operations_research::sat::Model model;
  operations_research::sat::SatParameters parameters;
  parameters.set_random_seed(P_->GetSeed() + 1);
  parameters.set_num_workers(
      std::max<int>(std::thread::hardware_concurrency(), 1));
  model.Add(NewSatParameters(parameters));
  // std::cout << model.parameters().num_search_workers() << std::endl;
  // operations_research::sat::SatParameters parameters;
  parameters.set_max_time_in_seconds(GetRemainedTime()/1000.0);
  model.Add(NewSatParameters(parameters));
  if (set_initial_solution_) {
    gpiibt_b_->SetAndcomputeOneStep(&A_, time_step_);
    for (auto &a : A_) {
      int action_id = -1;
      if (a->v_next == a->v_now) {
        action_id = x[a->id].size() - 1;
      } else {
        for (uint j = 0; j < a->v_now->neighbor.size(); ++j) {
          if (a->v_now->neighbor[j] == a->v_next) {
            action_id = j;
            break;
          }
        }
      }
      assert(action_id != -1);
      for (uint j = 0; j < x[a->id].size(); ++j) {
        if (static_cast<int>(j) == action_id) {
          cp_model->AddHint(x[a->id][j], 1);
        } else {
          cp_model->AddHint(x[a->id][j], 0);
        }
      }
    }
  } else {
    for (auto &a : A_) {
      for (uint j = 0; j < x[a->id].size(); ++j) {
        if (static_cast<int>(j+ 1) == x[a->id].size()) {
          cp_model->AddHint(x[a->id][j], 1);
        } else {
          cp_model->AddHint(x[a->id][j], 0);
        }
      }
    }
  }
  auto response =
      operations_research::sat::SolveCpModel(cp_model->Build(), &model);
  // auto* params = model.GetOrCreate<operations_research::sat::SatParameters>();
  if (response.status() != operations_research::sat::CpSolverStatus::OPTIMAL &&
      response.status() != operations_research::sat::CpSolverStatus::FEASIBLE) {
    // std::cout << "Time Step:" << time_step_ << ", infeasible" << std::endl;
    // for (uint i = 0; i < A_.size(); ++i) {
    //   A_[i]->v_next = A_[i]->v_now;
    //   occupied_next_[A_[i]->v_next->id] = A_[i];
    // }
    // puts("No solution found.");
    // exit(0);
    return false;
  }
  // if (response.status() == operations_research::sat::CpSolverStatus::OPTIMAL)
  // {
  //   std::cout << "Time Step:" << time_step_ << ", optimal" << std::endl;
  // } else {
  //   std::cout << "Time Step:" << time_step_ << ", feasible" << std::endl;
  // }
  // if (response.status() == operations_research::sat::CpSolverStatus::OPTIMAL)
  // {
  //     std::cout << "Solution found." << std::endl;
  //     std::cout << "Objective value = " << response.objective_value() <<
  //     std::endl;
  // } else {
  //     double gap = (response.objective_value() -
  //     response.best_objective_bound()) / response.objective_value();
  //     std::cout << "Gap = " << gap << std::endl;
  // }

  for (uint i = 0; i < A_.size(); ++i) {
    int id_i = A_[i]->id;
    for (uint j = 0; j < x[id_i].size(); ++j) {
      if (SolutionBooleanValue(response, x[id_i][j]) == true) {
        if (j == x[id_i].size() - 1) {
          A_[i]->v_next = A_[i]->v_now;
        } else {
          A_[i]->v_next = A_[i]->v_now->neighbor[j];
        }
        // assert(occupied_next_[A_[i]->v_next->id] == nullptr ||
        //        occupied_next_[A_[i]->v_next->id] == A_[i]);
        occupied_next_[A_[i]->v_next->id] = A_[i];
      }
    }
  }
  // puts("Solution found.");
  return true;
}

}  // namespace gpibt
