#pragma once
#include <vector>
#include <string>
#include "problem.hpp"

/*
 * array of configurations
 */
namespace gpibt {
struct Plan {
 private:
  Configs configs_;  // main

 public:
  ~Plan() {}

  // timestep -> configuration
  Config Get(const int t) const;

  // timestep, agent -> location
  Node* Get(const int t, const int i) const;

  // path
  Path GetPath(const int i) const;

  // path cost
  int GetPathCost(const int i) const;

  // last configuration
  Config Last() const;

  // last configuration
  Node* Last(const int i) const;

  // become empty
  void Clear();

  // add new configuration to the last
  void Add(const Config& c);

  // whether configs are empty
  bool Empty() const;

  // configs.size
  int Size() const;

  // size - 1
  int GetMakespan() const;

  // sum of cost
  int GetSOC(const std::vector<int>& reach_goal_time) const;

  void GetReachGoalTime(const Config& goals,
                        std::vector<int>* reach_goal_time) const;

  // join with other plan
  Plan operator+(const Plan& other) const;
  void operator+=(const Plan& other);

  // check the plan is valid or not
  bool Validate(MAPF_Instance* P) const;
  bool Validate(const Config& starts, const Config& goals) const;
  bool ValidateConflict() const;

  // when updating a single path,
  // the path should be longer than this value to avoid conflicts
  int GetMaxConstraintTime(const int id, Node* s, Node* g, Graph* G) const;
  int GetMaxConstraintTime(const int id, MAPF_Instance* P) const;

  // error
  void Halt(const std::string& msg) const;
  void Warn(const std::string& msg) const;
};

using Plans = std::vector<Plan>;
};  // namespace gpibt
