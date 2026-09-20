#include "amp/core/simulation.hpp"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>

namespace {

bool require(const bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << "\n";
  }
  return condition;
}

}  // namespace

int main() {
  bool passed = true;

  const auto missing = amp::Simulation::load(
      std::filesystem::path(AMP_TEST_SCENE_PATH).parent_path() / "does-not-exist.xml");
  passed &= require(!missing.has_value(), "a missing scene must return an error");

  auto simulation = amp::Simulation::load(AMP_TEST_SCENE_PATH);
  passed &= require(simulation.has_value(), "the pinned SO-101 scene must load");
  if (!simulation) {
    std::cerr << simulation.error() << "\n";
    return EXIT_FAILURE;
  }

  passed &= require(simulation->model().nq == 6, "SO-101 must have six generalized coordinates");
  passed &= require(simulation->model().nv == 6, "SO-101 must have six degrees of freedom");
  passed &= require(simulation->model().nu == 6, "SO-101 must have six actuators");
  passed &=
      require(simulation->controls().size() == 6, "all six actuator controls must be exposed");

  const double initial_time = simulation->data().time;
  simulation->step(10);
  passed &=
      require(simulation->data().time > initial_time, "stepping must advance simulation time");
  passed &=
      require(std::isfinite(simulation->data().energy[0]), "potential energy must stay finite");
  passed &= require(std::isfinite(simulation->data().energy[1]), "kinetic energy must stay finite");

  simulation->reset();
  passed &= require(simulation->data().time == 0.0, "reset must restore time to zero");

  return passed ? EXIT_SUCCESS : EXIT_FAILURE;
}
