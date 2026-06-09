#pragma once
#include <vector>
#include <string> 

#include "problem.hpp"
namespace gpibt {

/*
 * array of path
 */

struct Paths {
 private:
  std::vector<Path> paths_;  // main
  int makespan_;

 public:
  Paths() {}
  explicit Paths(int num_agents);
  ~Paths() {}

  // agent -> path
  Path Get(int i) const;

  // agent, timestep -> location
  Node* Get(int i, int t) const;

  // return last node
  Node* Last(int i) const;

  // whether paths are empty
  bool Empty() const;

  // whether a path of a_i is empty
  bool Empty(int i) const;

  // insert new path
  void Insert(int i, const Path& path);

  // clear
  void Clear(int i);

  // return paths.size
  int Size() const;

  // joint with other paths
  void operator+=(const Paths& other);

  // get maximum length
  int GetMaxLengthPaths() const;

  // makespan
  int GetMakespan() const;

  // cost of paths[i]
  int CostOfPath(int i) const;

  // sum of cost
  int GetSOC() const;

  // formatting
  void Format();
  void Shrink();

  // =========================
  // for CBS
  // check conflicted
  bool Conflicted(int i, int j, int t) const;

  // count conflicts for all
  int CountConflict() const;

  // count conflict within a subset of agents
  int CountConflict(const std::vector<int>& sample) const;

  // count conflict with one path
  int CountConflict(int id, const Path& path) const;

  // error
  void Halt(const std::string& msg) const;
  void Warn(const std::string& msg) const;
};
};  // namespace gpibt
