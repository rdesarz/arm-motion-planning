#include <benchmark/benchmark.h>

#include "so101_traj_planner/core/minimum_time_reach_planner.hpp"

namespace {

void benchmark_minimum_time_search(benchmark::State& state) {
  auto planner = so101_traj_planner::MinimumTimeReachPlanner::load(
      AMP_BENCHMARK_ROBOT_PATH, static_cast<std::size_t>(state.range(0)));
  if (!planner) {
    state.SkipWithError(planner.error().message);
    return;
  }

  const so101_traj_planner::MinimumTimeReachRequest request{
      .q_start = {0.0, 0.0, 0.0, 0.0, 0.0, 0.25},
      .target_world_m = Eigen::Vector3d{0.31741606, -0.09770090, 0.26459835},
      .minimum_duration_s = 1.0,
      .maximum_duration_s = 2.0,
      .duration_resolution_s = 0.2,
  };

  std::int64_t evaluated_horizons = 0;
  for (auto _ : state) {
    const auto result = planner->plan(request);
    if (!result) {
      state.SkipWithError(result.error().message);
      break;
    }
    double duration_s = result->trajectory.knots.back().time_s;
    benchmark::DoNotOptimize(duration_s);
    benchmark::ClobberMemory();
    evaluated_horizons += static_cast<std::int64_t>(result->search_report.evaluated_horizon_count);
  }
  state.SetItemsProcessed(evaluated_horizons);
}

BENCHMARK(benchmark_minimum_time_search)
    ->Arg(1)
    ->Arg(2)
    ->Arg(4)
    ->UseRealTime()
    ->Unit(benchmark::kMillisecond);

}  // namespace

BENCHMARK_MAIN();
