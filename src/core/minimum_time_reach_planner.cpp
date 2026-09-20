#include "so101_traj_planner/core/minimum_time_reach_planner.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <thread>
#include <utility>
#include <vector>

namespace so101_traj_planner {

namespace {

constexpr std::size_t kMaximumCandidateHorizonCount = 1000;
constexpr std::size_t kMaximumWorkerCount = 32;

PlanningError error(const PlanningErrorCode code, std::string message) {
  return PlanningError{.code = code, .message = std::move(message)};
}

std::expected<std::vector<double>, PlanningError> candidate_durations(
    const MinimumTimeReachRequest& request) {
  if (!std::isfinite(request.minimum_duration_s) || request.minimum_duration_s <= 0.0) {
    return std::unexpected(error(PlanningErrorCode::invalid_request,
                                 "minimum_duration_s must be finite and strictly positive"));
  }
  if (!std::isfinite(request.maximum_duration_s) ||
      request.maximum_duration_s < request.minimum_duration_s) {
    return std::unexpected(
        error(PlanningErrorCode::invalid_request,
              "maximum_duration_s must be finite and no smaller than minimum_duration_s"));
  }
  if (!std::isfinite(request.duration_resolution_s) || request.duration_resolution_s <= 0.0) {
    return std::unexpected(error(PlanningErrorCode::invalid_request,
                                 "duration_resolution_s must be finite and strictly positive"));
  }
  if (!request.target_world_m.allFinite()) {
    return std::unexpected(
        error(PlanningErrorCode::invalid_request, "target_world_m must contain finite values"));
  }
  if (!std::ranges::all_of(request.q_start,
                           [](const double value) { return std::isfinite(value); })) {
    return std::unexpected(
        error(PlanningErrorCode::invalid_request, "q_start must contain finite values"));
  }

  const double interval = request.maximum_duration_s - request.minimum_duration_s;
  const double estimated_count = std::ceil(interval / request.duration_resolution_s) + 1.0;
  if (!std::isfinite(estimated_count) ||
      estimated_count > static_cast<double>(kMaximumCandidateHorizonCount)) {
    return std::unexpected(error(PlanningErrorCode::invalid_request,
                                 "duration search exceeds the 1000-candidate resource limit"));
  }

  std::vector<double> durations;
  durations.reserve(static_cast<std::size_t>(estimated_count));
  for (std::size_t index = 0; index < kMaximumCandidateHorizonCount; ++index) {
    const double duration =
        request.minimum_duration_s + static_cast<double>(index) * request.duration_resolution_s;
    if (duration > request.maximum_duration_s) {
      break;
    }
    durations.push_back(duration);
  }

  const double comparison_scale = std::max(1.0, std::abs(request.maximum_duration_s));
  const double comparison_tolerance =
      16.0 * std::numeric_limits<double>::epsilon() * comparison_scale;
  if (durations.empty() || request.maximum_duration_s - durations.back() > comparison_tolerance) {
    durations.push_back(request.maximum_duration_s);
  } else {
    durations.back() = request.maximum_duration_s;
  }

  if (durations.size() > kMaximumCandidateHorizonCount) {
    return std::unexpected(error(PlanningErrorCode::invalid_request,
                                 "duration search exceeds the 1000-candidate resource limit"));
  }
  return durations;
}

}  // namespace

class MinimumTimeReachPlanner::Impl {
 public:
  explicit Impl(std::vector<AligatorReachPlanner> planners) : planners_(std::move(planners)) {}

  std::vector<AligatorReachPlanner> planners_;
};

MinimumTimeReachPlanner::MinimumTimeReachPlanner(std::unique_ptr<Impl> impl)
    : impl_(std::move(impl)) {}
MinimumTimeReachPlanner::~MinimumTimeReachPlanner() = default;
MinimumTimeReachPlanner::MinimumTimeReachPlanner(MinimumTimeReachPlanner&&) noexcept = default;
MinimumTimeReachPlanner& MinimumTimeReachPlanner::operator=(MinimumTimeReachPlanner&&) noexcept =
    default;

std::expected<MinimumTimeReachPlanner, PlanningError> MinimumTimeReachPlanner::load(
    const std::filesystem::path& robot_mjcf, std::size_t worker_count) {
  if (worker_count == 0) {
    worker_count =
        std::min(kMaximumWorkerCount,
                 static_cast<std::size_t>(std::max(1U, std::thread::hardware_concurrency())));
  } else if (worker_count > kMaximumWorkerCount) {
    return std::unexpected(
        error(PlanningErrorCode::invalid_request, "worker_count must not exceed 32"));
  }

  std::vector<AligatorReachPlanner> planners;
  planners.reserve(worker_count);
  for (std::size_t worker = 0; worker < worker_count; ++worker) {
    auto planner = AligatorReachPlanner::load(robot_mjcf);
    if (!planner) {
      return std::unexpected(planner.error());
    }
    planners.push_back(std::move(*planner));
  }
  return MinimumTimeReachPlanner(std::make_unique<Impl>(std::move(planners)));
}

std::expected<MinimumTimeReachResult, PlanningError> MinimumTimeReachPlanner::plan(
    const MinimumTimeReachRequest& request) const {
  auto durations = candidate_durations(request);
  if (!durations) {
    return std::unexpected(durations.error());
  }

  const auto started_at = std::chrono::steady_clock::now();
  const std::size_t worker_count = std::min(impl_->planners_.size(), durations->size());
  std::atomic_size_t next_index{0};
  std::atomic_size_t best_index{durations->size()};
  std::atomic_size_t evaluated_count{0};
  std::atomic_size_t validated_count{0};
  std::mutex result_mutex;
  std::optional<JointTrajectory> best_trajectory;
  std::optional<PlanningError> invalid_request_error;

  auto search_worker = [&](const std::size_t worker_index) {
    while (true) {
      const std::size_t candidate_index = next_index.fetch_add(1, std::memory_order_relaxed);
      if (candidate_index >= durations->size() ||
          candidate_index >= best_index.load(std::memory_order_acquire)) {
        break;
      }

      const ReachRequest fixed_duration_request{
          .q_start = request.q_start,
          .target_world_m = request.target_world_m,
          .duration_s = (*durations)[candidate_index],
      };
      auto candidate = impl_->planners_[worker_index].plan(fixed_duration_request);
      evaluated_count.fetch_add(1, std::memory_order_relaxed);
      if (!candidate) {
        if (candidate.error().code == PlanningErrorCode::invalid_request) {
          std::scoped_lock lock(result_mutex);
          if (!invalid_request_error) {
            invalid_request_error = candidate.error();
          }
        }
        continue;
      }

      validated_count.fetch_add(1, std::memory_order_relaxed);
      std::scoped_lock lock(result_mutex);
      if (candidate_index < best_index.load(std::memory_order_relaxed)) {
        best_trajectory = std::move(*candidate);
        best_index.store(candidate_index, std::memory_order_release);
      }
    }
  };

  std::vector<std::jthread> workers;
  workers.reserve(worker_count);
  for (std::size_t worker = 0; worker < worker_count; ++worker) {
    workers.emplace_back(search_worker, worker);
  }
  for (auto& worker : workers) {
    worker.join();
  }

  if (invalid_request_error) {
    return std::unexpected(*invalid_request_error);
  }
  if (!best_trajectory) {
    std::ostringstream message;
    message << "no validated trajectory was found in [" << request.minimum_duration_s << ", "
            << request.maximum_duration_s << "] s after evaluating "
            << evaluated_count.load(std::memory_order_relaxed)
            << " candidate horizons; individual solver failures are not infeasibility proofs";
    return std::unexpected(error(PlanningErrorCode::planning_failed, message.str()));
  }

  return MinimumTimeReachResult{
      .trajectory = std::move(*best_trajectory),
      .search_report =
          MinimumTimeSearchReport{
              .candidate_horizon_count = durations->size(),
              .evaluated_horizon_count = evaluated_count.load(std::memory_order_relaxed),
              .validated_horizon_count = validated_count.load(std::memory_order_relaxed),
              .worker_count = worker_count,
              .computation_time_s =
                  std::chrono::duration<double>(std::chrono::steady_clock::now() - started_at)
                      .count(),
          },
  };
}

}  // namespace so101_traj_planner
