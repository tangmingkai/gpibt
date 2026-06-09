#include <getopt.h>

#include <iostream>
#include <random>
#include <vector>

#include "algorithm/default_params.hpp"
#include "algorithm/problem.hpp"
// #include <tp.hpp>
#include "algorithm/algorithm_list.hpp"
// #include <glog/logging.h>

using namespace gpibt;

void PrintHelp();

int main(int argc, char* argv[]) {
  // google::InitGoogleLogging(argv[0]);
  // google::InstallFailureSignalHandler();
  std::string instance_file = "";
  std::string output_file = DEFAULT_OUTPUT_FILE;
  std::string solver_name;
  bool verbose = false;
  char* argv_copy[argc + 1];
  for (int i = 0; i < argc; ++i) argv_copy[i] = argv[i];

  struct option longopts[] = {
      {"instance", required_argument, 0, 'i'},
      {"output", required_argument, 0, 'o'},
      {"solver", required_argument, 0, 's'},
      {"verbose", no_argument, 0, 'v'},
      {"help", no_argument, 0, 'h'},
      {"time-limit", required_argument, 0, 't'},
      {"log-short", no_argument, 0, 'l'},
      {"enable-guidance", no_argument, 0, 'G'},
      {"enable-swap-action", no_argument, 0, 'S'},
      {"set-initial-solution", no_argument, 0, 'I'},
      {0, 0, 0, 0},
  };
  bool log_short = false;
  int max_comp_time = -1;
  bool use_guidance = false;
  bool enable_swap_action = false;
  bool set_initial_solution = false;
  // command line args
  int opt, longindex;
  opterr = 0;  // ignore getopt error
  while ((opt = getopt_long(argc, argv, "i:o:s:vht:lG:S:I:", longopts,
                            &longindex)) != -1) {
    switch (opt) {
      case 'i':
        instance_file = std::string(optarg);
        break;
      case 'o':
        output_file = std::string(optarg);
        break;
      case 's':
        solver_name = std::string(optarg);
        break;
      case 'v':
        verbose = true;
        break;
      case 'h':
        PrintHelp();
        return 0;
      case 'l':
        log_short = true;
        break;
      case 'T':
        max_comp_time = std::atoi(optarg);
        break;
      case 'G':
        use_guidance = std::atoi(optarg);
        break;
      case 'S':
        enable_swap_action = std::atoi(optarg);
        break;
      case 'I':
        set_initial_solution = std::atoi(optarg);
        break;
      default:
        break;
    }
  }

  if (instance_file.length() == 0) {
    std::cout << "specify instance file using -i [INSTANCE-FILE], e.g.,"
              << std::endl;
    std::cout << "> ./mapd -i ../instance/sample.txt" << std::endl;
    return 0;
  }

  // set problem
  auto P = MAPF_Instance(instance_file);
  // set max computation time (otherwise, use param in instance_file)
  if (max_comp_time != -1) P.SetMaxCompTime(max_comp_time);

  // solve
  auto solver = GetSolver(solver_name, &P, verbose, argc, argv_copy,
                          use_guidance, false, set_initial_solution);

  solver->SetLogShort(log_short);
  std::cout << "start solve "  << output_file << std::endl;
  solver->Solve();
  puts("finish solve");
  if (solver->Succeed() && !solver->GetSolution().Validate(&P)) {
    std::cout << "error@mapd: invalid results" << std::endl;
    return 0;
  }
  puts("finish check");
  solver->PrintResult();
  puts("finish print result");
  // output result
  solver->MakeLog(output_file);
  if (verbose) {
    std::cout << "save result as " << output_file << std::endl;
  }
  std::cout << "finish" << " " << output_file << std::endl;
  return 0;
}

void PrintHelp() {
  std::cout
      << "\nUsage: ./mapd [OPTIONS] [SOLVER-OPTIONS]\n"
      << "\n**instance file is necessary to run MAPD simulator**\n\n"
      << "  -i --instance [FILE_PATH]     instance file path\n"
      << "  -o --output [FILE_PATH]       ouptut file path\n"
      << "  -v --verbose                  print additional info\n"
      << "  -h --help                     help\n"
      << "  -d --use-distance-table       use pre-computed distance table\n"
      << "  -s --solver [SOLVER_NAME]     solver, choose from the below\n"
      << "  -T --time-limit [INT]         max computation time (ms)\n"
      << "  -L --log-short                use short log\n";
}
