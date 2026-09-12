#pragma once

#include <memory>

#include "amp/reach_planner.hpp"

namespace amp {

class AligatorReachPlanner final : public ReachPlanner {
 public:
  ~AligatorReachPlanner() override;
  AligatorReachPlanner(AligatorReachPlanner&&) noexcept;
  AligatorReachPlanner& operator=(AligatorReachPlanner&&) noexcept;
  AligatorReachPlanner(const AligatorReachPlanner&) = delete;
  AligatorReachPlanner& operator=(const AligatorReachPlanner&) = delete;

  [[nodiscard]] std::expected<JointTrajectory, PlanningError> plan(
      const ReachRequest& request) const override;

 private:
  friend class ReachPlannerFactory;
  class Impl;
  explicit AligatorReachPlanner(std::unique_ptr<Impl> impl);

  std::unique_ptr<Impl> impl_;
};

}  // namespace amp
