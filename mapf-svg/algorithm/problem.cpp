#include "problem.hpp"

#include <fstream>
#include <iomanip>
#include <regex>
#include "default_params.hpp"

#include "util.hpp"

namespace gpibt {
Problem::Problem(std::string _instance, Graph* _G, std::mt19937* _MT,
                 Config _config_s, Config _config_g, int _num_agents,
                 int _max_timestep, int _max_comp_time, uint _seed)
    : instance_(_instance),
      G_(_G),
      MT_(_MT),
      config_s_(_config_s),
      config_g_(_config_g),
      num_agents_(_num_agents),
      max_timestep_(_max_timestep),
      max_comp_time_(_max_comp_time),
      seed_(_seed) {}

Node* Problem::GetStart(int i) const {
  if (!(0 <= i && i < (int)config_s_.size())) Halt("invalid index");
  return config_s_[i];
}

Node* Problem::GetGoal(int i) const {
  if (!(0 <= i && i < (int)config_g_.size())) Halt("invalid index");
  return config_g_[i];
}

void Problem::Halt(const std::string& msg) const {
  std::cout << "error@Problem: " << msg << std::endl;
  this->~Problem();
  std::exit(1);
}

void Problem::Warn(const std::string& msg) const {
  std::cout << "warn@Problem: " << msg << std::endl;
}

// -------------------------------------------
// MAPF

MAPF_Instance::MAPF_Instance(const std::string& _instance)
    : Problem(_instance), instance_initialized_(true) {
  // read instance file
  std::ifstream file(_instance);
  if (!file) Halt("file " + _instance + " is not found.");

  std::string line;
  std::smatch results;
  std::regex r_comment = std::regex(R"(#.+)");
  std::regex r_map = std::regex(R"(map_file=(.+))");
  std::regex r_task = std::regex(R"(task_file=(.+))");
  std::regex r_agents = std::regex(R"(agents=(\d+))");
  std::regex r_seed = std::regex(R"(seed=(\d+))");
  std::regex r_max_timestep = std::regex(R"(max_timestep=(\d+))");
  std::regex r_max_comp_time = std::regex(R"(max_comp_time=(\d+))");
  std::regex r_sg = std::regex(R"((\d+),(\d+),(\d+),(\d+))");

  while (getline(file, line)) {
    // for CRLF coding
    if (*(line.end() - 1) == 0x0d) line.pop_back();
    // comment
    if (std::regex_match(line, results, r_comment)) {
      continue;
    }
    // read map
    if (std::regex_match(line, results, r_map)) {
      G_ = new Grid(results[1].str());
      continue;
    }
    // read task
    if (std::regex_match(line, results, r_task)) {
      task_file_ = results[1].str();
      continue;
    }
    // set agent num
    if (std::regex_match(line, results, r_agents)) {
      num_agents_ = std::stoi(results[1].str());
      continue;
    }
    // set random seed
    if (std::regex_match(line, results, r_seed)) {
      seed_ = std::stoi(results[1].str());
      MT_ = new std::mt19937(seed_);
      continue;
    }
    // set max timestep
    if (std::regex_match(line, results, r_max_timestep)) {
      max_timestep_ = std::stoi(results[1].str());
      continue;
    }
    // set max computation time
    if (std::regex_match(line, results, r_max_comp_time)) {
      max_comp_time_ = std::stoi(results[1].str());
      continue;
    }
    // read initial/goal nodes
    if (std::regex_match(line, results, r_sg) &&
        (int)config_s_.size() < num_agents_) {
      int x_s = std::stoi(results[1].str());
      int y_s = std::stoi(results[2].str());
      int x_g = std::stoi(results[3].str());
      int y_g = std::stoi(results[4].str());
      if (!G_->existNode(x_s, y_s)) {
        Halt("start node (" + std::to_string(x_s) + ", " + std::to_string(y_s) +
             ") does not exist, invalid scenario");
      }
      if (!G_->existNode(x_g, y_g)) {
        Halt("goal node (" + std::to_string(x_g) + ", " + std::to_string(y_g) +
             ") does not exist, invalid scenario");
      }

      Node* s = G_->getNode(x_s, y_s);
      Node* g = G_->getNode(x_g, y_g);
      config_s_.push_back(s);
      config_g_.push_back(g);
    }
  }

  // set default value not identified params
  if (MT_ == nullptr) {
    seed_ = DEFAULT_SEED;
    MT_ = new std::mt19937(DEFAULT_SEED);
  }
  if (max_timestep_ == 0) max_timestep_ = DEFAULT_MAX_TIMESTEP;
  if (max_comp_time_ == 0) max_comp_time_ = DEFAULT_MAX_COMP_TIME;

  // check starts/goals
  if (num_agents_ != config_s_.size()) Halt("invalid number of agents");
}

MAPF_Instance::MAPF_Instance(std::string _instance, Graph* _G,
                             std::mt19937* _MT, Config _config_s,
                             Config _config_g, int _num_agents,
                             int _max_timestep, int _max_comp_time, uint seed)
    : Problem(_instance, _G, _MT, _config_s, _config_g, _num_agents,
              _max_timestep, _max_comp_time, seed),
      instance_initialized_(false) {}

// MAPF_Instance::MAPF_Instance(MAPF_Instance* P, Config _config_s,
//                              Config _config_g, int _max_comp_time,
//                              int _max_timestep)
//     : Problem(P->GetInstanceFileName(), P->GetG(), P->GetMT(), _config_s,
//               _config_g, P->GetNum(), _max_timestep, _max_comp_time),
//       instance_initialized_(false) {}

// MAPF_Instance::MAPF_Instance(MAPF_Instance* P, int _max_comp_time)
//     : Problem(P->GetInstanceFileName(), P->GetG(), P->GetMT(),
//               P->GetConfigStart(), P->GetConfigGoal(), P->GetNum(),
//               P->GetMaxTimestep(), _max_comp_time),
//       instance_initialized_(false) {}

MAPF_Instance::~MAPF_Instance() {
  if (instance_initialized_) {
    if (G_ != nullptr) delete G_;
    if (MT_ != nullptr) delete MT_;
  }
}

void MAPF_Instance::PrintInfo() {
  Grid* grid = reinterpret_cast<Grid*>(G_);
  std::cout << "  agents=" << num_agents_ << std::right << std::setw(3)
            << ",seed=0"
            << ",random_problem=0"
            << ",max_timestep=" << max_timestep_ << std::right << std::setw(5)
            << ",max_comp_time=" << max_comp_time_ << std::right << std::setw(5)
            << ",map_file=" << grid->getMapFileName() << std::endl;
}

void MAPF_Instance::MakeScenFile(const std::string& output_file) {
  Grid* grid = reinterpret_cast<Grid*>(G_);
  std::ofstream log;
  log.open(output_file, std::ios::out);
  log << "map_file=" << grid->getMapFileName() << "\n";
  log << "agents=" << num_agents_ << "\n";
  log << "seed=0\n";
  log << "random_problem=0\n";
  log << "max_timestep=" << max_timestep_ << "\n";
  log << "max_comp_time=" << max_comp_time_ << "\n";
  for (int i = 0; i < num_agents_; ++i) {
    log << config_s_[i]->pos.x << "," << config_s_[i]->pos.y << ","
        << config_g_[i]->pos.x << "," << config_g_[i]->pos.y << "\n";
  }
  log.close();
}
};  // namespace gpibt
