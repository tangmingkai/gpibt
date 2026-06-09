#pragma once

#include <memory>
#include <string>

#include "gpiibt_b/gpiibt_b.hpp"
#include "gpiibt_r/gpiibt_r.hpp"
namespace gpibt {
static std::unique_ptr<PIBT_Base> GetSolver(const std::string solver_name,
                                       MAPF_Instance* P, bool verbose, int argc,
                                       char* argv[], bool use_guidance,
                                       bool enable_swap_action,
                                       bool set_initial_solution) {
  std::unique_ptr<PIBT_Base> solver;
  std::cout << "solver:" << solver_name << std::endl;
  if (solver_name == "GPIBT-B") {
    solver =
        std::make_unique<GPIBT_B>(P, use_guidance, false, set_initial_solution);
  } else if (solver_name == "GPIBT-R") {
    solver = std::make_unique<GPIBT_R>(P, use_guidance, false,
                                set_initial_solution, "CPSAT-v2");
  } else {
    std::cout << "warn@mapd: "
              << "unknown solver name, " + solver_name << std::endl;
    std::exit(1);
  }
  solver->SetParams(argc, argv);
  solver->SetVerbose(verbose);
  return solver;
}
};  // namespace gpibt
