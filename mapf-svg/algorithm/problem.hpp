#pragma once
#include <graph.hpp>
#include <random>
#include <vector>
#include <string>

#include "util.hpp"
namespace gpibt {

using Config = std::vector<Node*>;  // < loc_0[t], loc_1[t], ... >
using Configs = std::vector<Config>;

// check two configurations are same or not
[[maybe_unused]] static bool SameConfig(const Config& config_i,
                                        const Config& config_j) {
  if (config_i.size() != config_j.size()) return false;
  const int size_i = config_i.size();
  for (int k = 0; k < size_i; ++k) {
    if (config_i[k] != config_j[k]) return false;
  }
  return true;
}

[[maybe_unused]] static int GetPathCost(const Path& path) {
  int cost = path.size() - 1;
  auto itr = path.end() - 1;
  while (itr != path.begin() && *itr == *(itr - 1)) {
    --cost;
    --itr;
  }
  return cost;
}

class Problem {
 protected:
  std::string instance_;  // instance name
  Graph* G_;              // graph
  std::mt19937* MT_;      // seed
  int seed_;
  Config config_s_;        // initial configuration
  Config config_g_;        // goal configuration
  int num_agents_;         // number of agents
  int max_timestep_;       // timestep limit
  int max_comp_time_;      // comp_time limit, ms
  std::string task_file_;  // task file

  // utilities
  void Halt(const std::string& msg) const;
  void Warn(const std::string& msg) const;

 public:
  Problem() { MT_ = nullptr; }
  explicit Problem(const std::string& _instance) : instance_(_instance) {
    MT_ = nullptr;
  }
  Problem(std::string _instance, Graph* _G, std::mt19937* _MT, Config _config_s,
          Config _config_g, int _num_agents, int _max_timestep,
          int _max_comp_time, uint _seed);
  ~Problem() {}

  Graph* GetG() { return G_; }
  int GetNum() { return num_agents_; }
  std::mt19937* GetMT() { return MT_; }
  uint GetSeed() { return seed_; }
  Node* GetStart(int i) const;  // return start of a_i
  Node* GetGoal(int i) const;   // return  goal of a_i
  Config GetConfigStart() const { return config_s_; }
  Config GetConfigGoal() const { return config_g_; }
  int GetMaxTimestep() { return max_timestep_; }
  int GetMaxCompTime() { return max_comp_time_; }
  std::string GetInstanceFileName() { return instance_; }
  std::string GetTaskFileName() { return task_file_; }

  void SetMaxCompTime(const int t) { max_comp_time_ = t; }
};

class MAPF_Instance : public Problem {
 private:
  const bool instance_initialized_;  // for memory manage

  // set starts and goals randomly
  void SetRandomStartsGoals();

  // set well-formed instance
  void SetWellFormedInstance();

 public:
  explicit MAPF_Instance(const std::string& _instance);
  // MAPF_Instance(MAPF_Instance* P, Config _config_s, Config _config_g,
  //               int _max_comp_time, int _max_timestep);
  // MAPF_Instance(MAPF_Instance* P, int _max_comp_time);
  MAPF_Instance(std::string _instance, Graph* _G, std::mt19937* _MT,
                Config _config_s, Config _config_g, int _num_agents,
                int _max_timestep, int _max_comp_time, uint _seed);
  ~MAPF_Instance();

  bool IsInitializedInstance() const { return instance_initialized_; }

  // used when making new instance file
  void MakeScenFile(const std::string& output_file);

  void PrintInfo();
};
};  // namespace gpibt

