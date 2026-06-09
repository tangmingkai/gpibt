#include "paths.hpp"

namespace gpibt {
Paths::Paths(int num_agents) {
  std::vector<Path> tmp(num_agents, Path(0));
  paths_ = tmp;
  makespan_ = 0;
}

Path Paths::Get(int i) const {
  const int paths_size = paths_.size();
  if (!(0 <= i && i < paths_size)) Halt("invalid index");
  return paths_[i];
}

Node* Paths::Get(int i, int t) const {
  if (!(0 <= i && i < (int)paths_.size()) || !(0 <= t && t <= makespan_)) {
    Halt("invalid index, i=" + std::to_string(i) + ", t=" + std::to_string(t));
  }
  return paths_[i][t];
}

Node* Paths::Last(int i) const { return Get(i, makespan_); }

bool Paths::Empty() const { return paths_.empty(); }

bool Paths::Empty(const int i) const {
  const int paths_size = paths_.size();
  if (!(0 <= i && i < paths_size))
    Halt("invalid index, i=" + std::to_string(i));
  return paths_[i].empty();
}

void Paths::Insert(int i, const Path& path) {
  const int paths_size = paths_.size();
  if (!(0 <= i && i < paths_size)) Halt("invalid index");
  if (path.empty()) Halt("path must not be empty");
  const int old_len = paths_[i].size();
  paths_[i] = path;
  const int path_size = path.size();
  if (path_size - 1 == GetMakespan()) return;
  Format();                           // align each path size
  if (path_size < old_len) Shrink();  // cutoff additional configs
  makespan_ = GetMaxLengthPaths();     // update makespan
}

void Paths::Clear(int i) { paths_[i].clear(); }

int Paths::Size() const { return paths_.size(); }

void Paths::operator+=(const Paths& other) {
  if (other.Empty()) return;
  if (paths_.size() != other.paths_.size()) Halt("invalid operation");
  const int paths_size = paths_.size();
  if (makespan_ == 0) {  // empty
    for (int i = 0; i < paths_size; ++i) Insert(i, other.Get(i));
  } else {
    std::vector<Path> new_paths(paths_size);
    for (int i = 0; i < paths_size; ++i) {
      if (paths_[i].empty() || *(paths_[i].end() - 1) != other.paths_[i][0]) {
        Halt("invalid operation");
      }
      Path tmp;
      // former
      for (int t = 0; t <= GetMakespan(); ++t) {
        tmp.push_back(Get(i, t));
      }
      // later
      for (int t = 1; t <= other.GetMakespan(); ++t) {
        tmp.push_back(other.Get(i, t));
      }
      new_paths[i] = tmp;
    }
    for (int i = 0; i < paths_size; ++i) Insert(i, new_paths[i]);
  }
}

// this func is used when updating makespan
int Paths::GetMaxLengthPaths() const {
  int max_val = 0;
  for (auto p : paths_) {
    if (p.empty()) continue;
    const int p_size = p.size();
    max_val = (p_size - 1 > max_val) ? p_size - 1 : max_val;
  }
  return max_val;
}

int Paths::GetMakespan() const { return makespan_; }

int Paths::CostOfPath(int i) const {
  const int paths_size = paths_.size();
  if (!(0 <= i && i < paths_size)) {
    Halt("invalid index " + std::to_string(i));
  }
  return GetPathCost(Get(i));
}

int Paths::GetSOC() const {
  int soc = 0;
  const int paths_size = paths_.size();
  for (int i = 0; i < paths_size; ++i) soc += CostOfPath(i);
  return soc;
}

void Paths::Format() {
  const int paths_size = paths_.size();
  int len = GetMaxLengthPaths();
  for (int i = 0; i < paths_size; ++i) {
    if (paths_[i].empty()) continue;
    int p_size = paths_[i].size();
    while (p_size - 1 != len) {
      paths_[i].push_back(*(paths_[i].end() - 1));
      ++p_size;
    }
  }
}

void Paths::Shrink() {
  const int paths_size = paths_.size();
  while (true) {
    bool shrinkable = true;
    for (auto p : paths_) {
      if (p.size() <= 1 || *(p.end() - 1) != *(p.end() - 2)) {
        shrinkable = false;
        break;
      }
    }
    if (!shrinkable) break;

    // remove additional configuration
    for (int i = 0; i < paths_size; ++i) {
      paths_[i].resize(paths_[i].size() - 1);
    }
  }
}

bool Paths::Conflicted(int i, int j, int t) const {
  // vertex conflict
  if (Get(i, t) == Get(j, t)) return true;
  // swap conflict
  if (Get(i, t) == Get(j, t - 1) && Get(j, t) == Get(i, t - 1)) return true;
  return false;
}

int Paths::CountConflict() const {
  int cnt = 0;
  int num_agents = Size();
  int makespan = GetMakespan();
  for (int i = 0; i < num_agents; ++i) {
    for (int j = i + 1; j < num_agents; ++j) {
      for (int t = 1; t <= makespan; ++t) {
        if (Conflicted(i, j, t)) ++cnt;
      }
    }
  }
  return cnt;
}

int Paths::CountConflict(const std::vector<int>& sample) const {
  int cnt = 0;
  int makespan = GetMakespan();
  const int sample_size = sample.size();
  for (int i = 0; i < sample_size; ++i) {
    for (int j = i + 1; j < sample_size; ++j) {
      for (int t = 1; t <= makespan; ++t) {
        if (Conflicted(sample[i], sample[j], t)) ++cnt;
      }
    }
  }
  return cnt;
}

int Paths::CountConflict(int id, const Path& path) const {
  int cnt = 0;
  int makespan = GetMakespan();
  int num_agents = Size();
  const int path_size = path.size();
  for (int i = 0; i < num_agents; ++i) {
    if (i == id) continue;
    for (int t = 1; t < path_size; ++t) {
      if (t > makespan) {
        if (path[t] == Get(i, makespan)) {
          ++cnt;
          break;
        }
        continue;
      }
      // vertex conflict
      if (Get(i, t) == path[t]) {
        ++cnt;
        continue;
      }
      // swap conflict
      if (Get(i, t) == path[t - 1] && Get(i, t - 1) == path[t]) ++cnt;
    }
  }
  return cnt;
}

void Paths::Halt(const std::string& msg) const {
  std::cout << "error@Paths: " << msg << std::endl;
  this->~Paths();
  std::exit(1);
}

void Paths::Warn(const std::string& msg) const {
  std::cout << "warn@Paths: " << msg << std::endl;
}
};  // namespace gpibt

