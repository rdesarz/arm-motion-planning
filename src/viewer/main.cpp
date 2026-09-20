#include <charconv>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string_view>

#include "amp/core/simulation.hpp"
#include "amp/viewer/viewer.hpp"

namespace {

struct Options {
  std::filesystem::path scene = AMP_DEFAULT_SCENE_PATH;
  bool headless = false;
  std::size_t steps = 1000;
};

void print_usage(const std::string_view executable) {
  std::cout << "Usage: " << executable << " [--headless] [--steps N] [scene.xml]\n"
            << "\n"
            << "Without --headless, opens an interactive MuJoCo viewer.\n";
}

std::expected<Options, std::string> parse_options(const int argc, char* argv[]) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string_view argument = argv[index];
    if (argument == "--headless") {
      options.headless = true;
    } else if (argument == "--steps") {
      if (++index >= argc) {
        return std::unexpected("--steps requires a positive integer");
      }
      const std::string_view value = argv[index];
      const auto result = std::from_chars(value.data(), value.data() + value.size(), options.steps);
      if (result.ec != std::errc{} || result.ptr != value.data() + value.size() ||
          options.steps == 0) {
        return std::unexpected("Invalid --steps value: " + std::string(value));
      }
    } else if (argument == "--help" || argument == "-h") {
      print_usage(argv[0]);
      std::exit(EXIT_SUCCESS);
    } else if (!argument.empty() && argument.front() == '-') {
      return std::unexpected("Unknown option: " + std::string(argument));
    } else {
      options.scene = argument;
    }
  }
  return options;
}

}  // namespace

int main(int argc, char* argv[]) {
  const auto options = parse_options(argc, argv);
  if (!options) {
    std::cerr << "Error: " << options.error() << "\n";
    print_usage(argv[0]);
    return EXIT_FAILURE;
  }

  auto simulation = amp::Simulation::load(options->scene);
  if (!simulation) {
    std::cerr << "Error: " << simulation.error() << "\n";
    return EXIT_FAILURE;
  }

  const auto& model = simulation->model();
  std::cout << "Loaded SO-101 scene: " << simulation->scene_path() << "\n"
            << "Model dimensions: nq=" << model.nq << ", nv=" << model.nv << ", nu=" << model.nu
            << ", bodies=" << model.nbody << "\n";

  if (options->headless) {
    simulation->step(options->steps);
    std::cout << "Completed " << options->steps
              << " steps; simulation time=" << simulation->data().time << " s\n";
    return EXIT_SUCCESS;
  }

  if (const auto result = amp::run_viewer(*simulation); !result) {
    std::cerr << "Error: " << result.error() << "\n";
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
